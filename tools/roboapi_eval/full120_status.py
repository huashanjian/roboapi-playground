#!/usr/bin/env python3
"""Read live process identity and publish partial coverage without inventing scores."""
from policies.roboapi.config import MODEL
import csv
import json
from pathlib import Path

from tools.roboapi_eval.full120_supervisor import GROUP, CONTROL, read, rows, process_token, now, write


def main():
    protocol = read(GROUP / 'protocol.json')
    records, workers, interruptions = [], [], []
    for index, assigned in enumerate(protocol['worker_tasks']):
        base = GROUP / 'workers' / f'worker_{index}'
        items = rows(base, partial=True)
        assert all(r['env_name'] in assigned and r['model'] == MODEL for r in items)
        records.extend(items)
        state_path = CONTROL / f'worker_{index}.json'
        state = read(state_path) if state_path.exists() else {'status': 'not_started'}
        pid, token = state.get('child_pid'), state.get('child_start_token')
        state['child_live_verified'] = bool(pid and token and process_token(pid) == token)
        state['scored_episodes_current'] = len(items)
        task = state.get('task')
        if task:
            done = {r['seed'] for r in items if r['env_name'] == task}
            for seed in sorted(set(range(10)) - done):
                summary = base / task / f'run_{seed}_api/summary.json'
                if summary.exists():
                    try:
                        d = read(summary)
                        state['active_episode'] = {'task': task, 'seed': seed, **{k:d.get(k) for k in
                            ('status', 'requests', 'responses', 'executed_steps', 'errors')}}
                    except json.JSONDecodeError:
                        state['active_episode'] = {'task': task, 'seed': seed, 'status': 'summary_being_written'}
                    break
        workers.append(state)
        for path in (base / 'interrupted_attempts').glob('*/interruption.json'):
            interruptions.append(read(path))
    keys = {(r['env_name'], r['seed']) for r in records}
    assert len(keys) == len(records)
    expected = {(task, seed) for task in protocol['tasks'] for seed in range(10)}
    assert keys <= expected
    table = []
    for task in protocol['tasks']:
        selected = [r for r in records if r['env_name'] == task]
        n, k = len(selected), sum(r['success'] for r in selected)
        table.append({'task': task, 'completed': n, 'planned': 10, 'successes': k,
                      'success_rate_percent': 100*k/n if n else None,
                      'coverage_complete': n == 10, 'episode_length_limit_s': 120})
    supervisor = read(CONTROL / 'supervisor.json')
    pid, token = supervisor.get('pid'), supervisor.get('start_token')
    supervisor['live_verified'] = bool(pid and token and process_token(pid) == token)
    recoveries = []
    for path in sorted(CONTROL.glob('recovery_worker_*.json')):
        recovery = read(path)
        pid, token = recovery.get('pid'), recovery.get('start_token')
        recoveries.append({'worker': recovery['worker'], 'pid': pid,
                           'status': recovery['status'],
                           'live_verified': bool(pid and token and process_token(pid) == token)})
    result = {'checked_at_utc': now(), 'planned_tasks': 120, 'planned_rollouts': 1200,
              'completed_rollouts': len(records), 'completed_tasks': sum(t['coverage_complete'] for t in table),
              'successes': sum(r['success'] for r in records), 'coverage_complete': keys == expected,
              'final_acceptance': False, 'interrupted_attempts': len(interruptions),
              'supervisor': supervisor, 'recovery_helpers': recoveries,
              'workers': workers, 'tasks': table}
    write(GROUP / 'progress.json', result)
    with (GROUP / 'task_success_rates_partial.csv').open('w', encoding='utf-8-sig', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(table[0]))
        writer.writeheader()
        writer.writerows(table)
    compact = {k:result[k] for k in ('checked_at_utc', 'completed_rollouts', 'completed_tasks', 'successes', 'interrupted_attempts')}
    compact['supervisor_live'] = supervisor['live_verified']
    compact['live_recovery_helpers'] = sum(r['live_verified'] for r in recoveries)
    compact['workers'] = [{k:w.get(k) for k in ('worker', 'status', 'task', 'child_pid', 'child_live_verified', 'scored_episodes_current', 'active_episode', 'error')} for w in workers]
    print(json.dumps(compact, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
