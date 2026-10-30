"""Bounded recovery of a terminal worker's unscored slots only."""
import argparse
import fcntl
import os
from pathlib import Path
from tools.roboapi_eval import full120_supervisor as s


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--worker', type=int, required=True)
    args = parser.parse_args()
    index = args.worker
    protocol = s.read(s.GROUP / 'protocol.json')
    assert 0 <= index < protocol['workers']
    with (s.CONTROL / f'recovery_worker_{index}.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        state = s.read(s.CONTROL / f'worker_{index}.json')
        assert state['status'] == 'incomplete_requires_attention', state['status']
        assert not state.get('child_pid')
        base = s.GROUP / 'workers' / f'worker_{index}'
        records = s.rows(base)
        s.validate_completed(base, records)
        done = {(r['env_name'], r['seed']) for r in records}
        tasks = [t for t in protocol['worker_tasks'][index]
                 if any((t, seed) not in done for seed in range(10))]
        assert tasks
        baseline = {(t, seed): len(list((base / 'interrupted_attempts').glob(
            f'{t}_seed{seed}_attempt*'))) for t in tasks for seed in range(10)
            if (t, seed) not in done}
        phase = s.GROUP / 'scheduling_revisions' / (
            'recovery_' + s.now().replace(':', '-') + f'_worker{index}')
        phase.mkdir(parents=True, exist_ok=False)
        s.write(phase / 'previous_worker_state.json', state)
        evidence = {'started_at_utc': s.now(), 'pid': os.getpid(),
                    'start_token': s.process_token(os.getpid()), 'worker': index,
                    'status': 'running', 'tasks': tasks,
                    'additional_infrastructure_attempts_per_slot': s.MAX_ATTEMPTS,
                    'original_protocol_sha256': s.digest(s.GROUP / 'protocol.json'),
                    'supervisor_sha256': s.digest(Path(s.__file__)),
                    'note': 'Explicit infrastructure retry-budget extension after service recovery; evaluation settings unchanged; no scored outcome is retried.',
                    'missing_slots': [{'task': t, 'seed': seed, 'prior_attempts': count}
                                      for (t, seed), count in baseline.items()]}
        s.write(phase / 'recovery.json', evidence)
        s.write(s.CONTROL / f'recovery_worker_{index}.json', evidence)
        result = s.worker(index, tasks, recovery_baseline=baseline)
        evidence.update(status='terminal', finished_at_utc=s.now(), result=result)
        s.write(phase / 'recovery.json', evidence)
        s.write(s.CONTROL / f'recovery_worker_{index}.json', evidence)


if __name__ == '__main__':
    main()
