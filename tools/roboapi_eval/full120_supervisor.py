#!/usr/bin/env python3
"""Resumable multi-worker supervisor for the explicit 120 x 10 x 120 s protocol.

Completed official outcomes are never retried. Interrupted attempts are preserved
before fresh-session retries, and never converted into policy failures.
"""
from policies.roboapi.config import MODEL
import concurrent.futures
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import threading
import time
import traceback

from policies.roboapi.config import REPO, WORK, GROUP, RUN_NAME as NAME, TOKEN_FILE as TOKEN, BASE_URL

CONTROL = GROUP / 'control'
MAX_ATTEMPTS = 5
LOCK = threading.Lock()


def now():
    return datetime.now(timezone.utc).isoformat()


def read(path):
    return json.loads(Path(path).read_text())


def write(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + f'.{os.getpid()}.{threading.get_ident()}.tmp')
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
    tmp.replace(path)


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def process_token(pid):
    try:
        # /proc/stat comm may contain spaces; split after its final ')'.
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return None if fields[0] == 'Z' else fields[19]
    except (FileNotFoundError, ProcessLookupError):
        return None


def rows(base, partial=False):
    path = base / 'episode_results.jsonl'
    if not path.exists():
        return []
    result = []
    text = path.read_text()
    lines = text.splitlines()
    if partial and text and not text.endswith('\n'):
        lines = lines[:-1]
    for line in lines:
        if line.strip():
            result.append(json.loads(line))
    keys = [(item['env_name'], item['seed']) for item in result]
    if len(keys) != len(set(keys)):
        raise RuntimeError('Duplicate official task/seed records')
    return result


def validate_completed(base, items):
    """Fast admission check; the separate final audit checks every raw action."""
    import h5py
    allowed = set(read(base / 'manifest.json')['tasks'])
    for row in items:
        task, seed, steps = row['env_name'], row['seed'], row['episode_step']
        assert task in allowed and seed in range(10) and seed == row['run']
        assert row['model'] == MODEL and type(row['success']) is bool
        assert row['episode_length_limit_s'] == 120
        assert 0 < steps <= 3600 and (row['success'] or steps == 3600)
        assert math.isclose(row['duration'], steps / 30, abs_tol=1e-8)
        td = base / task
        summary = read(td / f'run_{seed}_api/summary.json')
        runtime = read(td / 'evaluation_runtime.json')
        assert runtime['passed'] and runtime['max_episode_steps'] == 3600
        assert runtime['episode_length_s'] == 120 and runtime['max_chunks'] == 90
        assert summary['model'] == MODEL and summary['max_chunks'] == 90
        assert summary['executed_steps'] == steps and summary['hold_steps'] == 0
        assert summary['requests'] == summary['responses'] == math.ceil(steps / 40)
        assert not [e for e in summary['errors'] if e.get('phase') != 'session_close']
        assert not (td / f'run_{seed}_error.json').exists()
        with h5py.File(td / f'run_{seed}.hdf5', 'r') as f:
            demo = f['data/demo_0']
            assert demo['actions'].shape == (steps, 8)
            assert bool(demo.attrs['success']) == row['success']


def session_check(summary, output, task, seed):
    """Check only this evaluator's session; preserve all actual HTTP evidence."""
    sid = summary.get('session_id')
    if not sid:
        return {'session_id': None, 'status': 'not_assigned_in_client_evidence'}
    import httpx
    with httpx.Client(timeout=15, trust_env=False,
                      headers={'Authorization': 'Bearer ' + TOKEN.read_text().strip()}) as client:
        evidence = {'method': 'GET', 'url': f'{BASE_URL}/sessions/{sid}',
                    'session_id': sid, 'task': task, 'seed': seed, 'trust_env': False}
        try:
            response = client.get(evidence['url'])
            evidence.update(http_status=response.status_code, response=response.json())
        except Exception as exc:
            evidence.update(http_status=None, response=None, error_type=type(exc).__name__)
        evidence['checked_at_utc'] = now()
        write(output, evidence)
        return evidence


def archive_interrupted(base, task, launch_log):
    scored = {r['seed'] for r in rows(base) if r['env_name'] == task}
    td = base / task
    if not td.exists():
        return []
    candidates = set()
    for p in td.iterdir():
        match = re.match(r'run_(\d+)(?:_api|_error\.json|\.hdf5)$', p.name)
        if match and int(match.group(1)) not in scored:
            candidates.add(int(match.group(1)))
    archived = []
    for seed in sorted(candidates):
        previous = list((base / 'interrupted_attempts').glob(f'{task}_seed{seed}_attempt*'))
        index = len(previous)
        dst = base / 'interrupted_attempts' / f'{task}_seed{seed}_attempt{index}'
        dst.mkdir(parents=True, exist_ok=False)
        # Never overwrite a prior attempt, including videos and partial HDF5.
        for p in list(td.iterdir()):
            if (re.match(rf'run_{seed}(?:\.|_)', p.name)
                    or p.name == f'log_{seed}_env0.json'
                    or re.search(rf'_{seed}(?:_viewport)?\.mp4$', p.name)):
                shutil.move(str(p), dst / p.name)
        for name in ('env_cfg.json', 'render_runtime.json', 'evaluation_runtime.json'):
            if (td / name).exists():
                shutil.copy2(td / name, dst / name)
        shutil.copy2(launch_log, dst / 'runtime.log')
        sp = dst / f'run_{seed}_api/summary.json'
        summary = read(sp) if sp.exists() else {}
        evidence = session_check(summary, dst / 'session_status_after_interruption.json', task, seed)
        metadata = {'task': task, 'seed': seed, 'attempt_index': index, 'model': MODEL,
                    'archived_at_utc': now(), 'included_in_policy_success_denominator': False,
                    'requests': summary.get('requests', 0), 'responses': summary.get('responses', 0),
                    'executed_steps': summary.get('executed_steps', 0),
                    'errors': summary.get('errors', []),
                    'server_session_status': (evidence.get('response') or {}).get('status', 'unknown'),
                    'partial_hdf5_note': 'On interruption the recorder may retain only a flushed prefix; execution receipts are recorded separately.',
                    'artifact_directory': str(dst.relative_to(base))}
        write(dst / 'interruption.json', metadata)
        write(dst / 'archive_inventory.json', {
            str(p.relative_to(dst)): {'sha256': digest(p), 'bytes': p.stat().st_size}
            for p in dst.rglob('*') if p.is_file()})
        archived.append(metadata)
    return archived


def worker(index, tasks, recovery_baseline=None):
    base = GROUP / 'workers' / f'worker_{index}'
    state_path = CONTROL / f'worker_{index}.json'
    deferred = []
    # If a previous supervisor died, adopt its still-live simulation process.
    old = read(state_path) if state_path.exists() else {}
    if old.get('child_pid') and old.get('child_start_token') and process_token(old['child_pid']) == old['child_start_token']:
        while process_token(old['child_pid']) == old['child_start_token']:
            old.update(status='adopting_live_child', heartbeat_utc=now())
            write(state_path, old)
            time.sleep(15)
    for task in tasks:
        launches = 0
        while True:
            current = rows(base)
            validate_completed(base, current)
            done = [r for r in current if r['env_name'] == task]
            if len(done) == 10:
                break
            seed = min(set(range(10)) - {r['seed'] for r in done})
            prior = list((base / 'interrupted_attempts').glob(f'{task}_seed{seed}_attempt*'))
            # A documented recovery phase grants a fresh bounded infrastructure
            # budget without deleting historical attempts or repeating scores.
            baseline = (recovery_baseline or {}).get((task, seed), 0)
            if len(prior) - baseline >= MAX_ATTEMPTS or launches >= 15:
                deferred.append({'task': task, 'seed': seed, 'reason': 'infrastructure_retry_budget_exhausted'})
                break
            if shutil.disk_usage(base).free < 20 * 1024 ** 3:
                raise RuntimeError('Local result disk below 20 GiB free; preserve data and await storage handling')
            # Preserve leftovers from a supervisor/process interruption before restart.
            if (base / task / f'run_{seed}_api').exists() or (base / task / f'run_{seed}_error.json').exists():
                oldlog = Path(old.get('log', ''))
                if not oldlog.is_file():
                    oldlog = CONTROL / 'recovery_note.txt'
                    oldlog.write_text('Recovered unscored artifacts after process disappearance.\n')
                archive_interrupted(base, task, oldlog)
                continue
            launches += 1
            log = CONTROL / 'runtime_logs' / f'worker{index}_{task}_{time.time_ns()}.log'
            log.parent.mkdir(parents=True, exist_ok=True)
            command = [str(REPO / 'policies/roboapi/run.sh'), '--model', MODEL,
                       '--task', task, '--num-runs', '10', '--seed', '0', '--episode-length-s', '120',
                       '--max-chunks', '90', '--video-mode', 'all', '--output-folder-name',
                       f'{NAME}/workers/worker_{index}']
            affinity_path = CONTROL / 'cpu_affinity_8workers.json'
            if affinity_path.exists():
                affinity = read(affinity_path)['workers'][index]['cpus']
                command = ['taskset', '-c', ','.join(map(str, affinity))] + command
            with log.open('wb') as handle:
                proc = subprocess.Popen(command, cwd=REPO, stdin=subprocess.DEVNULL,
                                        stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
            state = {'worker': index, 'task': task, 'status': 'running', 'started_at_utc': now(),
                     'child_pid': proc.pid, 'child_start_token': process_token(proc.pid),
                     'log': str(log), 'command': command, 'deferred': deferred}
            old = state
            write(state_path, state)
            while proc.poll() is None:
                state.update(heartbeat_utc=now(), scored_episodes=len(rows(base, partial=True)))
                write(state_path, state)
                time.sleep(15)
            state.update(status='validating', exit_code=proc.returncode,
                         child_pid=None, heartbeat_utc=now())
            write(state_path, state)
            current = rows(base)
            validate_completed(base, current)
            # A close-only error does not justify repeating an official result.
            for row in [r for r in current if r['env_name'] == task]:
                summary = read(base / task / f"run_{row['seed']}_api/summary.json")
                if summary['session_close_status'] != 'closed':
                    session_check(summary, base / 'session_lifecycle_checks' / f"{task}_seed{row['seed']}.json", task, row['seed'])
            interrupted = archive_interrupted(base, task, log)
            if len([r for r in current if r['env_name'] == task]) == 10:
                break
            if not interrupted:
                deferred.append({'task': task, 'seed': seed, 'reason': 'setup_or_process_failure', 'log': str(log)})
                break
            retries = len(list((base / 'interrupted_attempts').glob(f'{task}_seed{seed}_attempt*')))
            delay = min(600, 60 * 2 ** max(0, retries - 1))
            state.update(status='infrastructure_cooldown', retry_after_seconds=delay,
                         interruptions=interrupted, heartbeat_utc=now())
            write(state_path, state)
            time.sleep(delay)
    state = {'worker': index, 'status': 'rollouts_complete_pending_audit' if not deferred else 'incomplete_requires_attention',
             'scored_episodes': len(rows(base)), 'deferred': deferred, 'heartbeat_utc': now(), 'child_pid': None}
    write(state_path, state)
    return state


def main():
    CONTROL.mkdir(parents=True, exist_ok=True)
    with (CONTROL / 'supervisor.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        protocol = read(GROUP / 'protocol.json')
        assert protocol['model'] == MODEL and protocol['planned_rollouts'] == 1200
        assert protocol['episode_length_s'] == 120 and protocol['max_infrastructure_attempts_per_slot'] == MAX_ATTEMPTS
        write(CONTROL / 'supervisor.json', {'pid': os.getpid(), 'start_token': process_token(os.getpid()),
              'status': 'running', 'started_at_utc': now(), 'protocol_sha256': digest(GROUP / 'protocol.json')})
        with concurrent.futures.ThreadPoolExecutor(max_workers=protocol['workers']) as pool:
            futures = {pool.submit(worker, i, tasks): i for i, tasks in enumerate(protocol['worker_tasks'])}
            results = []
            for future in concurrent.futures.as_completed(futures):
                index = futures[future]
                try:
                    result = future.result()
                except BaseException as exc:
                    result = {'worker': index, 'status': 'requires_attention', 'error_type': type(exc).__name__,
                              'error': str(exc), 'traceback': traceback.format_exc(), 'heartbeat_utc': now()}
                    write(CONTROL / f'worker_{index}.json', result)
                results.append(result)
                print(json.dumps(result, ensure_ascii=False), flush=True)
        write(CONTROL / 'supervisor.json', {'pid': os.getpid(), 'status': 'terminal', 'finished_at_utc': now(),
                                           'workers': results, 'final_acceptance': False})


if __name__ == '__main__':
    main()
