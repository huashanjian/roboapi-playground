#!/usr/bin/env python3
"""Freeze disjoint worker protocols before any formal API rollout."""
from policies.roboapi.config import MODEL
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import argparse
import subprocess
import tarfile

from tools.roboapi_eval.full120_supervisor import GROUP, NAME, REPO, WORK, MAX_ATTEMPTS, digest, write


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workers', type=int, choices=(1, 4, 6, 8, 12), default=8)
    args = parser.parse_args()
    assert not (GROUP / 'protocol.json').exists(), 'Protocol is already frozen; use a new run name'
    assert not (GROUP / 'workers').exists(), 'Worker evidence already exists; use a new run name'
    preflight = GROUP / 'preflight_final'
    check = json.loads((preflight / 'preflight.json').read_text())
    assert check['dynamic_checks_passed'] and check['api_requests'] == 0
    assert json.loads((preflight / 'runtime_setup.json').read_text())['passed']
    runtime = json.loads((preflight / 'BananaInBowlTask/evaluation_runtime.json').read_text())
    assert runtime['passed'] and runtime['max_episode_steps'] == 3600
    manifest = json.loads((preflight / 'manifest.json').read_text())
    adapter = REPO / 'policies/roboapi'
    assert all(digest(adapter / name) == sha for name, sha in manifest['adapter_sha256'].items())
    assert json.loads((WORK / 'full120_asset_validation.json').read_text())['passed']
    catalog = json.loads((REPO / 'robolab/tasks/_metadata/task_metadata.json').read_text())
    configs = json.loads((GROUP / 'catalog/official_task_configs.json').read_text())
    assert len(catalog) == len(configs) == 120
    assert set(configs) == {item['task_name'] for item in catalog}
    for item in catalog:
        relative = 'robolab/tasks/' + item['filename']
        committed = subprocess.check_output(['git', 'show', f"{manifest['robolab_commit']}:{relative}"], cwd=REPO)
        assert hashlib.sha256(committed).hexdigest() == digest(REPO / relative)
        assert configs[item['task_name']]['episode_length_s'] == float(item['episode_s'])
    first = ['BananaInBowlTask', 'RubiksCubeAndBananaTask', 'Stack3RubiksCubeTask']
    tasks = first + sorted(set(configs) - set(first))
    assigned = [tasks[i::args.workers] for i in range(args.workers)]
    assert len({(task, seed) for group in assigned for task in group for seed in range(10)}) == 1200
    protocol = {
        'created_at_utc': datetime.now(timezone.utc).isoformat(), 'model': MODEL,
        'task_count': 120, 'rollouts_per_task': 10, 'planned_rollouts': 1200,
        'tasks': tasks, 'worker_tasks': assigned, 'workers': args.workers, 'seeds': list(range(10)),
        'episode_length_s': 120, 'simulation_steps_limit': 3600,
        'max_chunks': 90, 'actions_per_chunk': 40, 'control_hz': 30, 'physics_hz': 240,
        'http_timeout_s': 120, 'http_timeout_is_distinct_from_episode_timeout': True,
        'success_termination': 'Retain official success termination; successful tasks may finish before120s.',
        'model_input': 'One calibrated wrist RGB image, instruction, tool0 pose and continuous measured gripper opening.',
        'client_uses_proxy': False, 'inference_request_retries': 0,
        'max_infrastructure_attempts_per_slot': MAX_ATTEMPTS,
        'infrastructure_policy': 'Preserve every unscored attempt; rerun fresh session and same seed with60/120/240/480s backoff (cap600s). After5 interrupted attempts retain slot as unresolved, continue other tasks; never count infrastructure errors as policy failures.',
        'scored_outcomes': 'Never repeat an already officially scored success or failure.',
        'fixed_layout_repetitions': True,
        'paper_comparison': 'Custom120s horizon and calibrated gripper/single-fisheye camera differ from paper defaults.',
        'official_task_configs_sha256': digest(GROUP / 'catalog/official_task_configs.json'),
        'task_metadata_sha256': digest(REPO / 'robolab/tasks/_metadata/task_metadata.json'),
        'asset_validation_sha256': digest(WORK / 'full120_asset_validation.json'),
        'preflight_sha256': digest(preflight / 'preflight.json'),
        'supervisor_sha256_at_launch': digest(REPO / 'tools/roboapi_eval/full120_supervisor.py'),
    }
    assert not (GROUP / 'protocol.json').exists(), 'Protocol is already frozen'
    write(GROUP / 'protocol.json', protocol)
    evidence = GROUP / 'asset_evidence'
    evidence.mkdir(exist_ok=True)
    for name in ('roboapi_full120_asset_manifest.json', 'full120_asset_validation.json', 'full120_asset_supplements.json'):
        shutil.copy2(WORK / name, evidence / name)
    write(GROUP / 'task_catalog.json', catalog)
    snapshot_members = {str(p): 'policies/roboapi/' + p.name for p in adapter.iterdir() if p.is_file()}
    for section in ('rig_data_sha256',):
        for name, sha in manifest[section].items():
            assert digest(name) == sha
            snapshot_members[name] = 'rig_inputs/' + name.lstrip('/')
    for p in Path(manifest['robot_usd_override']).parent.iterdir():
        if p.is_file():
            snapshot_members[str(p)] = 'rig_asset/' + p.name
    for item in catalog:
        p = REPO / 'robolab/tasks' / item['filename']
        snapshot_members[str(p)] = str(p.relative_to(REPO))
    archive = GROUP / 'evaluation_code.tar.gz'
    with tarfile.open(archive, 'w:gz') as tar:
        for source, member in sorted(snapshot_members.items()):
            tar.add(source, arcname=member)
    (GROUP / 'workers').mkdir(parents=True)
    manifest['preflight_only'] = False
    manifest['catalog_only'] = False
    manifest['seeds'] = list(range(10))
    for i, tasks_for_worker in enumerate(assigned):
        base = GROUP / 'workers' / f'worker_{i}'
        base.mkdir()
        manifest['tasks'] = tasks_for_worker
        write(base / 'manifest.json', manifest)
        shutil.copy2(archive, base / 'evaluation_code.tar.gz')
        write(base / 'run_metadata.json', {
            'frozen_at_utc': protocol['created_at_utc'], 'worker_index': i,
            'manifest_sha256': digest(base / 'manifest.json'),
            'code_snapshot_sha256': digest(archive), 'snapshot_members': snapshot_members,
            'protocol_path': str(GROUP / 'protocol.json'), 'protocol_sha256': digest(GROUP / 'protocol.json'),
            'preflight_path': str(preflight), 'preflight_sha256': protocol['preflight_sha256'],
            'official_task_configs_path': str(GROUP / 'catalog/official_task_configs.json'),
            'official_task_configs_sha256': protocol['official_task_configs_sha256'],
            'planned_unique_slots': len(tasks_for_worker) * 10,
            'changes_after_freeze': 'Any runtime/policy change requires a new output and new freeze.'})
    print(json.dumps({'group': str(GROUP), 'tasks': 120, 'rollouts': 1200,
                      'workers': args.workers, 'episode_length_s': 120, 'protocol_sha256': digest(GROUP / 'protocol.json')}))


if __name__ == '__main__':
    main()
