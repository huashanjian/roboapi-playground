#!/usr/bin/env python3
"""Collect the full result only after1200 unique outcomes and current worker audits.

This is an evaluation gate; it does not create data backups.
It makes no API calls and never retries or scores an interrupted attempt.
"""
from policies.roboapi.config import MODEL
import csv
import json
import math
from pathlib import Path

import numpy as np

from tools.roboapi_eval.full120_supervisor import GROUP, CONTROL, read, rows, digest, process_token, now, write


def require(condition, message):
    if not condition:
        raise ValueError(message)


def validate_coverage(records, task_names):
    require(len(task_names) == len(set(task_names)) == 120, 'Expected exactly120 official task names')
    expected = {(task, seed) for task in task_names for seed in range(10)}
    keys = [(r['env_name'], r['seed']) for r in records]
    require(len(records) == len(set(keys)) == 1200 and set(keys) == expected,
            'Full120x10 coverage is not complete and unique')
    for row in records:
        require(row['model'] == MODEL, 'Wrong model')
        require(type(row['seed']) is int and row['run'] == row['seed'], 'Invalid seed/run identity')
        require(type(row['success']) is bool, 'Success must be an official boolean')
        require(row['episode_length_limit_s'] == 120, 'Wrong rollout timeout')
        steps = row['episode_step']
        require(type(steps) is int and 0 < steps <= 3600, 'Wrong control-step range')
        require(row['success'] or steps == 3600, 'Failure did not reach the two-minute horizon')
        require(row['dt'] == 1/30 and math.isclose(row['duration'], steps/30, abs_tol=1e-8),
                'Simulation duration inconsistent with control steps')


def latency_stats(values):
    a = np.asarray(values, dtype=float)
    require(len(a) > 0 and np.isfinite(a).all() and (a >= 0).all(), 'Invalid raw HTTP timings')
    return {'requests':len(a), 'mean_ms':float(a.mean()*1000),
            'median_ms':float(np.median(a)*1000), 'p95_ms':float(np.percentile(a,95)*1000)}


def main():
    protocol = read(GROUP/'protocol.json')
    # Recovery helpers can outlive the original supervisor. Final acceptance
    # must wait for these writers too, and disclose retry-budget extensions.
    recovery_controls = list(CONTROL.glob('recovery_worker_*.json'))
    waiter_path = CONTROL / 'worker1_recovery_waiter.json'
    if waiter_path.exists():
        recovery_controls.append(waiter_path)
    for path in recovery_controls:
        recovery = read(path)
        pid, token = recovery.get('pid'), recovery.get('start_token')
        require(not (pid and token and process_token(pid) == token),
                f'Recovery helper still live: {path.name}')
        require(recovery.get('status') == 'terminal', f'Unresolved recovery: {path.name}')
    recovery_phases = [read(path) for path in sorted(
        (GROUP/'scheduling_revisions').glob('recovery_*/recovery.json'))]
    records, audits, identities, request_ids, interrupted = [], [], [], [], []
    raw_latency = {task:[] for task in protocol['tasks']}
    for index, assigned in enumerate(protocol['worker_tasks']):
        base = GROUP/'workers'/f'worker_{index}'
        state = read(CONTROL/f'worker_{index}.json')
        pid, token = state.get('child_pid'), state.get('child_start_token')
        require(not (pid and token and process_token(pid) == token), f'Worker{index} still has a live simulation')
        require(state['status'] == 'rollouts_complete_pending_audit', f'Worker{index} did not finish normally')
        audit_path = base/'validation/final_acceptance_audit.json'
        audit = read(audit_path)
        require(audit['final_acceptance'] is True and audit['audit_passed'] is True
                and audit['audited_episodes'] == audit['expected_episodes'] == len(assigned) * 10,
                f'Worker{index} final audit has not passed all assigned episodes')
        require(audit['worker_index'] == index and audit['model'] == MODEL, 'Audit identity mismatch')
        require(audit['audit_script_sha256'] == digest(Path(__file__).with_name('audit_full120_worker.py')),
                'Final audit uses an older auditor')
        require(audit['ledger_snapshot_sha256'] == digest(base/'episode_results.jsonl'), 'Stale ledger audit')
        metadata = read(base/'run_metadata.json')
        require(digest(base/'manifest.json') == metadata['manifest_sha256']
                == audit['protocol']['manifest_sha256'], 'Stale manifest audit')
        require(digest(GROUP/'protocol.json') == metadata['protocol_sha256'], 'Protocol changed')
        items = rows(base)
        require({r['env_name'] for r in items} == set(assigned), 'Worker task assignment mismatch')
        records.extend(items)
        audits.append({'worker':index,'path':str(audit_path),'sha256':digest(audit_path)})
        interrupted.extend(audit['interrupted_attempts'])
        for episode in audit['episodes']:
            identities.append(episode['session_identity_sha256'])
        identities.extend(a['session_identity_sha256'] for a in audit['interrupted_attempts'] if a['session_identity_sha256'])
        for row in items:
            api = base/row['env_name']/f"run_{row['run']}_api"
            n = 0
            for request_path in sorted(api.glob('http_*_request.json')):
                request = read(request_path)
                if request['path'] != '/v1/chat/completions':
                    continue
                response = read(request_path.with_name(request_path.name.replace('_request','_response')))
                require(response['http_status'] == 200, 'Unexpected HTTP error in scored episode')
                raw_latency[row['env_name']].append(response['http_latency_s'])
                n += 1
            require(n == math.ceil(row['episode_step']/40), 'Raw response count mismatch')
            request_ids.extend(read(p)['request_id'] for p in api.glob('chunk_*.json'))
    validate_coverage(records, protocol['tasks'])
    require(len(identities) == len(set(identities)), 'Session reused across workers/attempts')
    require(len(request_ids) == len(set(request_ids)), 'Inference request ID reused across workers')
    table = []
    for task in protocol['tasks']:
        selected = [r for r in records if r['env_name'] == task]
        k = sum(r['success'] for r in selected)
        table.append({'task':task,'successes':k,'rollouts':10,'success_rate_percent':10*k,
                      'rollout_timeout_s':120,**latency_stats(raw_latency[task])})
    successes = sum(r['success'] for r in records)
    result = {'created_at_utc':now(),'model':MODEL,'tasks':120,'rollouts':1200,
              'successes':successes,'success_rate_percent':100*successes/1200,
              'episode_length_limit_s':120,'protocol_sha256':digest(GROUP/'protocol.json'),
              'latency':latency_stats([v for values in raw_latency.values() for v in values]),
              'interrupted_attempts':len(interrupted),'interruption_details':interrupted,
              'infrastructure_recovery_phases':recovery_phases,
              'worker_audits':audits,'per_task':table,'episodes':records,
              'evaluation_validated':True,'automatic_backup_enabled':False,
              'limitations':['Seeds repeat official fixed layouts, rather than randomized layouts.',
                             'Custom120s timeout and calibrated single-fisheye/continuous-gripper setup differ from paper defaults.',
                             'HTTP timing includes network and body transfer, not only server computation.',
                             'Infrastructure attempts are preserved separately; no official result is invented for them.']}
    write(GROUP/'RESULTS.json', result)
    with (GROUP/'task_success_rates.csv').open('w',encoding='utf-8-sig',newline='') as handle:
        writer = csv.DictWriter(handle,fieldnames=list(table[0]),lineterminator='\n')
        writer.writeheader()
        writer.writerows(table)
    print(json.dumps({k:result[k] for k in ('tasks','rollouts','successes','success_rate_percent','latency','interrupted_attempts','evaluation_validated','automatic_backup_enabled')},indent=2))


if __name__ == '__main__':
    main()
