#!/usr/bin/env python3
"""Offline strict full-benchmark audit; 120s/90chunks, official criteria preserved."""
from __future__ import annotations

from policies.roboapi.config import MODEL

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tarfile

import h5py
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation


from policies.roboapi.config import WORK, REPO
TASKS = {}



def read(path):
    return json.loads(path.read_text())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def identity_digest(value):
    return hashlib.sha256(value.encode()).hexdigest()


def git_bytes(repo, commit, relative):
    return subprocess.check_output(['git', 'show', f'{commit}:{relative}'], cwd=repo)


def git_head(repo):
    return subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip()


def required(condition, label, failures):
    result = bool(condition)
    if not result:
        failures.append(label)
    return result


def model_identity_record(payload, *, expected_model, source, required_field=True):
    """Check an explicit identity, never infer it from a filename or policy class."""
    present = isinstance(payload, dict) and 'model' in payload
    actual = payload.get('model') if isinstance(payload, dict) else None
    matches = actual == expected_model if present else not required_field
    return {'source': source, 'expected_model': expected_model, 'observed_model': actual,
            'model_field_present': present, 'model_field_required': required_field,
            'matches': bool(matches)}


def audit_model_identity(directory, *, expected_model, summary, row=None, seed_record=None,
                         session_evidence_path=None):
    """Read archived identities only; HTTP error bodies need not contain a model.

    An absent model on an error response is explicitly recorded, never inferred
    to confirm model identity. Any model present on any response must match.
    Successful create/infer/session-GET responses must state their model.
    Missing HTTP responses remain the separate transport completeness check's
    responsibility, so a preserved network timeout can still be audited.
    """
    records = []
    resolved = summary.get('resolved_model') or expected_model

    def check(payload, source, required_field=True, expected=None):
        records.append(model_identity_record(payload, expected_model=expected or expected_model,
                                             source=source, required_field=required_field))

    check(summary, str(directory / 'summary.json'))
    if row is not None:
        check(row, 'episode_results.jsonl:official_episode_row')
    if seed_record is not None:
        for index, result in enumerate(seed_record.get('results', [])):
            check(result.get('episode_metadata'), f'seed_record:results[{index}].episode_metadata')
    for request_path in sorted(directory.glob('http_*_request.json')):
        request = read(request_path)
        if request.get('path') not in ('/v1/sessions', '/v1/chat/completions'):
            continue
        check(request.get('body'), str(request_path))
        response_path = request_path.with_name(request_path.name.replace('_request', '_response'))
        if response_path.exists():
            response = read(response_path)
            status = response.get('http_status')
            success = type(status) is int and 200 <= status < 300
            check(response.get('body'), str(response_path), required_field=success, expected=resolved)
    if session_evidence_path is not None and session_evidence_path.exists():
        evidence = read(session_evidence_path)
        server = evidence.get('response')
        status = evidence.get('http_status')
        check(server, str(session_evidence_path), required_field=status == 200, expected=resolved)
    return {'expected_model': expected_model, 'resolved_model': resolved, 'checks': records,
            'checked_model_fields': sum(record['model_field_present'] for record in records),
            'passed': all(record['matches'] for record in records)}


def jsonl_snapshot(path, partial=False):
    raw = path.read_bytes()
    rows, issues = [], []
    lines = raw.splitlines()
    for index, line in enumerate(lines):
        if not line.strip():
            continue
        try:
            row = json.loads(line)
            if not isinstance(row, dict):
                raise ValueError('expected object')
            rows.append(row)
        except (ValueError, TypeError):
            if partial and index == len(lines) - 1 and not raw.endswith(b'\n'):
                issues.append({'line': index + 1, 'status': 'incomplete_trailing_line_deferred'})
            else:
                issues.append({'line': index + 1, 'status': 'invalid_json_record'})
    return rows, hashlib.sha256(raw).hexdigest(), issues


def audit_protocol(base, repo, manifest, failures, *, expected_model):
    metadata = read(base / 'run_metadata.json')
    manifest_hash = digest(base / 'manifest.json')
    frozen_manifest_hash = metadata.get('manifest_sha256')
    required(isinstance(frozen_manifest_hash, str) and len(frozen_manifest_hash) == 64
             and manifest_hash == frozen_manifest_hash, 'frozen_manifest_hash_mismatch', failures)
    robolab_commit = manifest['robolab_commit']
    required(git_head(repo) == robolab_commit, 'robolab_commit_mismatch', failures)
    archive = base / metadata.get('code_snapshot_file', 'evaluation_code.tar.gz')
    archive_hash = digest(archive)
    expected_archive = metadata.get('code_snapshot_sha256')
    required(archive_hash == expected_archive, 'snapshot_archive_hash_mismatch', failures)
    member_map = metadata.get('snapshot_members', {})
    hashes = {}
    with tarfile.open(archive) as snapshot:
        members = {member.name.lstrip('./'): member for member in snapshot.getmembers() if member.isfile()}
        def check_file(path, expected, candidate=None):
            path = Path(path)
            choices = [member_map.get(str(path)), candidate]
            for root, prefix in ((repo, ''),):
                try:
                    choices.append(prefix + str(path.relative_to(root)))
                except ValueError:
                    pass
            choices.append(str(path).lstrip('/'))
            selected = next((name.lstrip('./') for name in choices if name and name.lstrip('./') in members), None)
            archived = hashlib.sha256(snapshot.extractfile(members[selected]).read()).hexdigest() if selected else None
            actual = digest(path)
            hashes[str(path)] = {'manifest_sha256': expected, 'current_sha256': actual,
                'snapshot_member': selected, 'snapshot_sha256': archived,
                'matches': actual == expected == archived}
        for name, expected in manifest['adapter_sha256'].items():
            check_file(repo / 'policies/roboapi' / name, expected, 'policies/roboapi/' + name)
        for group in ('rig_data_sha256',):
            for name, expected in manifest[group].items():
                check_file(Path(name), expected)
    required(all(hashes.values()) and all(value['matches'] for value in hashes.values()),
             'frozen_code_current_or_archive_hash_mismatch', failures)
    required({'client.py', 'registration.py', 'run.py', 'gripper_action.py', 'build_asset.py', 'camera.py', 'preflight.py'}
             <= set(manifest['adapter_sha256']), 'adapter_hash_coverage', failures)
    required({'transport.py', 'config.py', 'optics.py', 'gripper_response.py'}
             <= set(manifest['adapter_sha256']), 'self_contained_module_hash_coverage', failures)
    sources = ['robolab/core/task/conditionals.py', 'robolab/core/task/conditionals_state_machine.py',
               'robolab/core/task/subtask.py', 'robolab/core/environments/env.py',
               'robolab/core/logging/recorder_manager.py', 'robolab/core/logging/results.py',
               'robolab/eval/runner.py', 'robolab/eval/episode.py', 'robolab/robots/franka_high_pd.py']
    sources += ['robolab/tasks/benchmark/' + spec['source'] for spec in TASKS.values()]
    official = {}
    for relative in sources:
        current = digest(repo / relative)
        committed = hashlib.sha256(git_bytes(repo, robolab_commit, relative)).hexdigest()
        official[relative] = {'current_sha256': current, 'commit_sha256': committed, 'matches': current == committed}
    required(all(x['matches'] for x in official.values()), 'official_scoring_or_runner_code_drift', failures)
    assets = dict(manifest['asset_manifest']['input_assets'])
    assets[manifest['asset_manifest']['output_asset']] = manifest['asset_manifest']['output_sha256']
    for path_key, hash_key in [('calibration_path', 'calibration_sha256'), ('assembly_scene', 'assembly_scene_sha256')]:
        assets[manifest['camera'][path_key]] = manifest['camera'][hash_key]
    asset_hashes = {}
    for path, expected in assets.items():
        actual = digest(Path(path))
        asset_hashes[path] = {'manifest_sha256': expected, 'current_sha256': actual, 'matches': actual == expected}
    required(all(x['matches'] for x in asset_hashes.values()), 'rig_asset_or_camera_hash_drift', failures)
    required(manifest['asset_manifest']['control_body'] == 'tool0'
             and manifest['asset_manifest']['gripper_main_joint'] == 'positive_outer_joint'
             and manifest['asset_manifest']['gripper_main_joint_open_closed_rad'] == [0., -.79]
             and manifest['asset_manifest']['unresolved_relationships_or_shader_connections'] == [],
             'rig_asset_control_point_or_gripper_map', failures)
    required(manifest['tasks'] == list(TASKS) and manifest['seeds'] == list(range(10)), 'manifest_tasks_or_seeds', failures)
    required(manifest['ground_truth_to_policy'] is False and manifest['preflight_only'] is False
             and manifest['calibrated_rig'] is True, 'manifest_policy_input_or_preflight', failures)
    manifest_model = model_identity_record(manifest, expected_model=expected_model, source=str(base / 'manifest.json'))
    required(manifest_model['matches'], 'manifest_model_identity_mismatch', failures)
    required(manifest['model'] == expected_model and manifest['session_enabled'] is True
             and manifest['inference_retries'] == 0 and manifest['max_chunks'] == 90 and manifest['episode_length_s'] == 120 and manifest['official_task_timeouts'] is False,
             'manifest_api_execution_protocol', failures)
    required(manifest['physics_dt_s'] == 1/240 and manifest['action_dt_s'] == 1/30
             and manifest['frame_mapping']['policy_from_base'] == 'Rz(-90deg)'
             and manifest['frame_mapping']['flange_from_tool0'] == 'Rz(+90deg)'
             and manifest['frame_mapping']['formula'] == 'T_policy_tool=Rz(-90deg) @ T_base_tool0',
             'manifest_timing_and_frames', failures)
    return {'manifest_sha256': manifest_hash, 'frozen_manifest_sha256': frozen_manifest_hash,
            'freeze_anchor': 'run_metadata.json supplied by the evaluator; this is integrity checking, not an external timestamp/notarization',
            'robolab_commit': robolab_commit,
            'manifest_model_identity': manifest_model,
            'frozen_code_file_hashes': hashes, 'code_archive_sha256': archive_hash,
            'official_code_integrity': official,
            'asset_and_camera_hashes': asset_hashes}


def audit_task_config(base, task, failures):
    cfg = read(base / task / 'env_cfg.json')
    spec = TASKS[task]
    required(cfg['episode_length_s'] == spec['seconds'] and cfg['sim']['dt'] == 1 / 240
             and cfg['decimation'] == 8 and cfg['sim']['device'] == 'cpu',
             task + ':runtime_timing_or_device', failures)
    required(cfg['terminations'] == spec['terminations'],
             task + ':official_success_predicate_mismatch', failures)
    required(cfg['terminations']['time_out'] == {
        'func': 'isaaclab.envs.mdp.terminations:time_out', 'params': {}, 'time_out': True},
        task + ':official_timeout_predicate_mismatch', failures)
    required(cfg['gripper_closure_cfg'] == {
        'gripper': {'asset': 'robot', 'joint': 'positive_outer_joint', 'open': 0., 'closed': -.79}},
        task + ':robotiq_gripper_scoring_mapping', failures)
    arm, gripper = cfg['actions']['arm_action'], cfg['actions']['gripper_action']
    required(arm['body_name'] == 'tool0' and arm['body_offset'] is None
             and arm['joint_names'] == ['panda_joint[1-7]'] and arm['scale'] == 1.
             and arm['controller']['use_relative_mode'] is False, task + ':tool0_absolute_action_control', failures)
    required(gripper['class_type'] == 'policies.roboapi.gripper_action:CalibratedGripperAction'
             and cfg['contact_gripper'] == {'gripper': '{ENV_REGEX_NS}/robot/RobotiqLinkage/positive_tip'},
             task + ':continuous_gripper_action_and_contact_mapping', failures)
    return {'episode_length_s': cfg['episode_length_s'], 'physics_dt_s': cfg['sim']['dt'],
            'control_dt_s': cfg['sim']['dt'] * cfg['decimation'], 'device': cfg['sim']['device'],
            'terminations': cfg['terminations'], 'gripper_closure_cfg': cfg['gripper_closure_cfg']}


def left_z_pose(pose, angle_deg):
    pose = np.asarray(pose, dtype=np.float64)
    rotation = Rotation.from_euler('z', angle_deg, degrees=True)
    q = (rotation * Rotation.from_quat(pose[..., [4, 5, 6, 3]])).as_quat()
    return np.concatenate([rotation.apply(pose[..., :3]), q[..., [3, 0, 1, 2]]], axis=-1)


def audit_episode(base, row, *, expected_model):
    task, run, seed = row['env_name'], row['run'], row['seed']
    td = base / task
    d = td / f'run_{run}_api'
    summary, seed_record, log = read(d / 'summary.json'), read(td / f'run_{run}_seed.json'), read(td / f'log_{run}_env0.json')
    events, _, event_issues = jsonl_snapshot(d / 'events.jsonl')
    request_paths = sorted(d.glob('http_*_request.json'))
    response_paths = sorted(d.glob('http_*_response.json'))
    requests = [read(path) for path in request_paths]
    responses = [read(path) for path in response_paths]
    chats = [(path, request) for path, request in zip(request_paths, requests)
             if request['path'] == '/v1/chat/completions']
    chunks = [read(path) for path in sorted(d.glob('chunk_*.json'))]
    feedback = [event for event in events if event['event'] == 'execution_feedback']
    failures, checks = [], {}

    def ck(label, condition):
        checks[label] = required(condition, label, failures)

    steps, horizon = row['episode_step'], TASKS[task]['seconds'] * 30
    ck('official_boolean_result', type(row['success']) is bool)
    ck('official_step_duration', type(steps) is int and 0 < steps <= horizon
       and (row['success'] is True or steps == horizon)
       and math.isclose(row['duration'], steps / 30, abs_tol=1e-8) and row['dt'] == 1 / 30)
    n = math.ceil(steps / 40)
    full_chunks, remainder = divmod(steps, 40)
    expected_budget = full_chunks == summary['max_chunks']
    lifecycle_errors = [error for error in summary['errors'] if error.get('phase') == 'session_close']
    rollout_errors = [error for error in summary['errors'] if error.get('phase') != 'session_close']
    lifecycle_error_events = [event for event in events
                              if event.get('event') == 'error' and event.get('phase') == 'session_close']
    other_error_events = [event for event in events
                          if event.get('event') == 'error' and event.get('phase') != 'session_close']
    lifecycle_evidence_path = base / 'session_lifecycle_checks' / f'{task}_seed{seed}.json'
    model_identity = audit_model_identity(d, expected_model=expected_model, summary=summary,
        row=row, seed_record=seed_record, session_evidence_path=lifecycle_evidence_path)
    ck('model_identity_consistent', model_identity['passed'])
    lifecycle_evidence = None
    server_closed_confirmed = False
    if lifecycle_evidence_path.exists():
        evidence = read(lifecycle_evidence_path)
        server = evidence['response']
        checked_at = datetime.fromisoformat(evidence['checked_at_utc']).timestamp()
        server_closed_confirmed = (
            evidence['task'] == task and type(evidence['seed']) is int and evidence['seed'] == seed
            and evidence['session_id'] == summary['session_id'] and evidence['http_status'] == 200
            and evidence['method'] == 'GET' and server['id'] == summary['session_id']
            and server['model'] == summary.get('resolved_model', expected_model) and server['status'] == 'closed'
            and server['next_turn'] == n + 1 and server['last_request_id'] == chunks[-1]['request_id']
            and checked_at >= max(event['time'] for event in events))
        lifecycle_evidence = {'path': str(lifecycle_evidence_path), 'sha256': digest(lifecycle_evidence_path),
            'checked_at_utc': evidence['checked_at_utc'], 'method': evidence['method'],
            'http_status': evidence['http_status'], 'server_status': server['status'],
            'next_turn': server['next_turn'], 'session_and_last_request_match':
                evidence['session_id'] == server['id'] == summary['session_id']
                and server['last_request_id'] == chunks[-1]['request_id'],
            'valid_later_closed_confirmation': bool(server_closed_confirmed),
            'source': 'Parent evaluator archived this later server GET; this audit makes no network request.'}
    normal_close = (summary['session_close_status'] == 'closed'
                    and summary['status'] == ('budget_exhausted' if expected_budget else 'closed')
                    and not lifecycle_errors)
    recovered_close = (summary['session_close_status'] == 'unknown' and summary['status'] == 'failed'
                       and bool(lifecycle_errors) and server_closed_confirmed)
    session_closed_verified = normal_close or recovered_close
    ck('seed_record_consistent', type(seed) is int and seed == run
       and seed_record['seed'] == seed and len(seed_record['results']) == 1
       and seed_record['results'][0]['step'] == steps
       and type(seed_record['results'][0]['success']) is bool
       and seed_record['results'][0]['success'] == row['success']
       and seed_record['results'][0]['episode_metadata']['seed'] == seed
       and row['layout_randomized'] is False and row['model'] == expected_model)
    ck('summary_execution_counts', summary['requests'] == summary['responses'] == n
       and summary['executed_steps'] == steps and summary['hold_steps'] == 0
       and summary['chunks_completed'] == full_chunks and summary['max_chunks'] == 90
       and summary['budget_exhausted'] is expected_budget and summary['pending_step'] is False
       and len(summary['api_latency_s']) == len(summary['api_http_latency_s']) == n)
    ck('session_close_verified', summary['session_enabled'] is True
       and bool(summary['session_id']) and session_closed_verified
       and summary['end_reason'] == 'episode_reset')
    ck('client_close_error_requires_explicit_later_server_evidence',
       not lifecycle_errors or recovered_close)
    ck('no_inference_or_execution_errors', not rollout_errors
       and not (td / f'run_{run}_error.json').exists()
       and not other_error_events and not event_issues)
    ck('lifecycle_error_summary_matches_events',
       [{'phase': event['phase'], 'type': event['type']} for event in lifecycle_error_events] == lifecycle_errors)
    ck('raw_http_archive_complete_and_successful', len(requests) == len(responses) == n + 1
       and len(chats) == n and all(response['http_status'] == 200 for response in responses)
       and requests[0] == {'method': 'POST', 'path': '/v1/sessions', 'body': {'model': expected_model}}
       and responses[0]['body']['id'] == summary['session_id']
       and responses[0]['body']['status'] == 'open'
       and all(path.with_name(path.name.replace('_request', '_response')).exists()
               for path in request_paths))
    expected_feedback = [(40, True)] * full_chunks + ([(remainder, False)] if remainder else [])
    ck('actual_execution_feedback', len(events) == n + len(lifecycle_error_events) and len(feedback) == n
       and [(e['executed_steps'], e['completed']) for e in feedback] == expected_feedback
       and all(type(e['completed']) is bool for e in feedback))
    input_ok = image_ok = history_ok = chunk_ok = observation_ok = decoded_ok = transformed_ok = True
    ids, raw_gripper_clipped_values = [], 0
    for index, (path, request) in enumerate(chats):
        body = request['body']
        messages = body['messages']
        user = [message for message in messages if message['role'] == 'user']
        roles = [message['role'] for message in messages]
        input_ok &= request['method'] == 'POST' and set(body) == {'model', 'messages', 'metadata'}
        input_ok &= body['model'] == expected_model and set(body['metadata']) == {'session_id', 'turn'}
        input_ok &= body['metadata']['session_id'] == summary['session_id']
        input_ok &= int(body['metadata']['turn']) == index + 1 and len(user) == 1
        content = user[0]['content']
        texts = [item['text'] for item in content if item['type'] == 'text']
        images = [item['image_url']['url'] for item in content if item['type'] == 'image_url']
        state = json.loads(texts[0])
        input_ok &= set(user[0]) == {'role', 'content'}
        input_ok &= len(content) == 2 and len(texts) == len(images) == 1
        input_ok &= all(set(item) == ({'type', 'text'} if item['type'] == 'text'
                                      else {'type', 'image_url'}) for item in content)
        input_ok &= all(set(item['image_url']) == {'url'} for item in content if item['type'] == 'image_url')
        input_ok &= set(images[0]) == {'archived_image', 'original_media_type'}
        input_ok &= set(state) == {'instruction', 'state'} and state['instruction'] == row['instruction']
        input_ok &= len(state['state']) == 8 and bool(np.isfinite(state['state']).all())
        observation = read(d / f'observation_{index:04d}.json')
        pose = observation['pose_wxyz']
        expected_wire = pose[:3] + pose[4:7] + pose[3:4] + [observation['gripper_open']]
        observation_ok &= bool(np.allclose(state['state'], expected_wire, atol=1e-7, rtol=0))
        native = np.asarray(observation['native_tool_pose_wxyz'])
        expected_policy = left_z_pose(native, -90)
        observation_ok &= bool(np.allclose(pose, expected_policy, atol=1e-12, rtol=0))
        with Image.open(d / images[0]['archived_image']) as image, Image.open(d / f'observation_{index:04d}.png') as original:
            image_ok &= image.size == (640, 360) and image.mode == 'RGB'
            image_ok &= bool(np.array_equal(np.asarray(image), np.asarray(original)))
        history_ok &= roles == (['user'] if index == 0 else ['assistant', 'tool', 'user'])
        if index:
            assistant, tool = messages[:2]
            history_ok &= set(tool) == {'role', 'tool_call_id', 'content'}
            history_ok &= assistant == responses[index]['body']['choices'][0]['message']
            history_ok &= json.loads(tool['content']) == {'executed_steps': 40, 'status': 'completed'}
            history_ok &= tool['tool_call_id'] == assistant['tool_calls'][0]['id']
        chunk = chunks[index]
        chunk_ok &= len(chunk['arm_wxyz']) == len(chunk['gripper_open']) == len(chunk['sim_actions']) == 40
        chunk_ok &= chunk['action_dt_s'] == 1 / 30 and chunk['session_id'] == summary['session_id']
        chunk_ok &= chunk['session_turn'] == index + 1 and chunk['request_id'] == feedback[index]['request_id']
        response = read(path.with_name(path.name.replace('_request', '_response')))['body']
        chunk_ok &= response['request_id'] == chunk['request_id']
        chunk_ok &= response['metadata']['session_id'] == summary['session_id']
        chunk_ok &= response['metadata']['turn'] == str(index + 1)
        chunk_ok &= response['metadata']['next_turn'] == str(index + 2)
        message = response['choices'][0]['message']
        raw = np.asarray(json.loads(message['tool_calls'][0]['function']['arguments'])['output'], dtype=float)
        decoded_ok &= raw.shape == (40, 8) and bool(np.isfinite(raw).all())
        quaternion = raw[:, 3:7] / np.linalg.norm(raw[:, 3:7], axis=1, keepdims=True)
        decoded = np.column_stack([raw[:, :3], quaternion[:, [3, 0, 1, 2]]])
        decoded_ok &= bool(np.allclose(decoded, chunk['arm_wxyz'], atol=1e-12, rtol=0))
        decoded_ok &= bool(np.array_equal(np.clip(raw[:, 7], 0, 1), chunk['gripper_open']))
        decoded_ok &= bool(np.logical_and(raw[:, 7] >= -.01, raw[:, 7] <= 1.01).all())
        raw_gripper_clipped_values += int(np.logical_or(raw[:, 7] < 0, raw[:, 7] > 1).sum())
        arm = np.asarray(chunk['arm_wxyz']).copy()
        arm[:, 3:7] /= np.linalg.norm(arm[:, 3:7], axis=1, keepdims=True)
        tool = left_z_pose(arm, 90)
        expected_actions = np.column_stack([tool, chunk['gripper_open']]).astype(np.float32)
        sim_actions = np.asarray(chunk['sim_actions'])
        transformed_ok &= bool(np.allclose(expected_actions[:, :7], sim_actions[:, :7], atol=1e-7, rtol=0))
        transformed_ok &= bool(np.array_equal(expected_actions[:, 7], sim_actions[:, 7]))
        transformed_ok &= chunk['sim_actions_frame'] == 'robot-root tool0'
        ids.append(chunk['request_id'])
    ck('no_ground_truth_in_model_request_allowlist', input_ok)
    ck('wire_proprio_observation_and_frame_mapping', observation_ok)
    ck('single_original_640x360_RGB_image', image_ok)
    ck('history_only_model_output_and_execution_feedback', history_ok)
    ck('chunks_and_session_turns_consistent', len(chunks) == n and chunk_ok)
    ck('raw_response_decodes_to_archived_chunk', decoded_ok)
    ck('chunk_frame_conversion_and_gripper_mapping', transformed_ok)
    concatenated = np.concatenate([np.asarray(chunk['sim_actions']) for chunk in chunks])[:steps]
    with h5py.File(td / f'run_{run}.hdf5', 'r') as file:
        demo = file['data/demo_0']
        actions = demo['actions'][:]
        score = float(demo['subtask/score'][-1])
        max_score = float(np.max(demo['subtask/score'][:]))
        success = bool(demo.attrs['success'])
        action_error = float(np.max(np.abs(actions - concatenated))) if actions.shape == concatenated.shape else None
        ck('hdf5_actions_exact_match_executed_chunks', actions.shape == (steps, 8) and action_error == 0)
        ck('official_result_and_score_consistency', demo.attrs['num_samples'] == steps
           and success == row['success'] and type(log['success']) is bool
           and log['success'] == row['success'] and log['final_step'] == steps
           and math.isfinite(score) and row['score'] == score)
    ck('official_failure_reason_present', row['success'] is True
       or isinstance(row.get('reason'), str) and bool(row['reason'].strip()))
    videos = list(td.glob(f'*_{run}.mp4'))
    ck('video_artifact_present', len(videos) == 1 and videos[0].stat().st_size > 0)
    overview = list(td.glob(f'*_{run}_viewport.mp4'))
    ck('viewport_video_present', len(overview) == 1 and overview[0].stat().st_size > 0)
    runtime = read(td / 'evaluation_runtime.json')
    ck('two_minute_runtime_readback', runtime['passed'] and runtime['max_episode_steps'] == 3600
       and runtime['episode_length_s'] == 120 and runtime['max_chunks'] == 90
       and row.get('episode_length_limit_s') == 120)
    return ({'task': task, 'run': run, 'seed': seed, 'audit_passed': not failures,
             'checks': checks, 'failures': failures, 'success': row['success'],
             'model_identity': model_identity,
             'final_score': score, 'maximum_recorded_score': max_score,
             'official_failure_reason': row.get('reason'), 'control_steps': steps,
             'simulated_seconds': row['duration'], 'inference_requests': n,
             'http_200_count': len(responses), 'fully_executed_chunks': full_chunks,
             'partially_executed_last_chunk_steps': remainder, 'hold_steps': summary['hold_steps'],
             'session_close_status': summary['session_close_status'],
             'client_status': summary['status'],
             'session_closed_verified': bool(session_closed_verified),
             'session_close_evidence_kind': 'later_server_GET_after_client_close_error' if recovered_close else 'client_lifecycle_acknowledgement',
             'session_lifecycle_error_count': len(lifecycle_errors),
             'session_lifecycle_errors': lifecycle_errors,
             'later_server_session_evidence': lifecycle_evidence,
             'session_identity_sha256': identity_digest(summary['session_id']),
             'budget_reached_exactly_at_official_horizon': expected_budget and steps == horizon,
             'raw_gripper_values_clipped_within_documented_0_01_tolerance': raw_gripper_clipped_values,
             'action_max_absolute_difference': action_error,
             'official_event_counts': dict(Counter(event['name'] for event in log['events'])),
             'api_http_latency_s_mean': float(np.mean(summary['api_http_latency_s'])),
             'episode_wall_time_s': seed_record['wall_episode_s'],
             'api_errors': summary['errors'],
             'artifact_directory': str(td), 'api_artifact_directory': str(d)}, summary['session_id'], ids)


def audit_preserved_attempt(path):
    """Preservation and reliability evidence; no task outcome assigned."""
    metadata = read(path / 'interruption.json')
    inventory = read(path / 'archive_inventory.json')
    files = {str(p.relative_to(path)) for p in path.rglob('*')
             if p.is_file() and p.name != 'archive_inventory.json'}
    checks = {'archive_file_coverage': files == set(inventory),
              'archive_hashes_match': all((path/name).is_file()
                  and (path/name).stat().st_size == entry['bytes']
                  and digest(path/name) == entry['sha256'] for name, entry in inventory.items()),
              'excluded_from_policy_denominator': metadata['included_in_policy_success_denominator'] is False}
    summary_path = path / f"run_{metadata['seed']}_api/summary.json"
    summary = read(summary_path) if summary_path.exists() else None
    error_path = path / f"run_{metadata['seed']}_error.json"
    error = read(error_path) if error_path.exists() else None
    responses, identity, http_errors = [], None, []
    if summary is not None:
        identity = audit_model_identity(summary_path.parent, expected_model=MODEL, summary=summary)
        checks['model_identity'] = identity['passed']
        checks['execution_counts_preserved'] = all(metadata[key] == summary[key]
            for key in ('requests', 'responses', 'executed_steps'))
        for request_path in sorted(summary_path.parent.glob('http_*_request.json')):
            request = read(request_path)
            response_path = request_path.with_name(request_path.name.replace('_request', '_response'))
            if response_path.exists() and request['path'] == '/v1/chat/completions':
                response = read(response_path)
                responses.append(response)
                if response['http_status'] >= 400:
                    http_errors.append({'http_status': response['http_status'], 'body': response['body']})
        checks['no_invented_accepted_responses'] = summary['responses'] <= len(responses) <= summary['requests']
    if error is not None:
        checks['no_invented_policy_result'] = error['policy_success'] is None
    query_path = path / 'session_status_after_interruption.json'
    query = read(query_path) if query_path.exists() else None
    server = (query or {}).get('response')
    server = server if isinstance(server, dict) else {}
    closed_by_get = bool(summary and query and query.get('http_status') == 200
        and server.get('id') == summary.get('session_id') and server.get('model') == summary.get('resolved_model', MODEL)
        and server.get('status') == 'closed')
    closed_by_client = bool(summary and summary.get('session_close_status') == 'closed')
    return {'directory': str(path), 'metadata': metadata, 'checks': checks,
            'preservation_passed': all(checks.values()), 'summary': summary,
            'model_identity': identity, 'error_record': error, 'http_error_responses': http_errors,
            'received_http_inference_responses': len(responses),
            'session_closed_acknowledged': closed_by_client or closed_by_get,
            'session_identity_sha256': identity_digest(summary['session_id']) if summary and summary.get('session_id') else None,
            'partial_trajectory_excluded_from_valid_episode_tracking': True}


def main():
    from tools.roboapi_eval.full120_supervisor import GROUP
    global TASKS
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--partial', action='store_true')
    mode.add_argument('--final', action='store_true')
    args = parser.parse_args()
    base = args.run
    manifest, metadata = read(base/'manifest.json'), read(base/'run_metadata.json')
    protocol = read(GROUP/'protocol.json')
    baseline = read(GROUP/'catalog/official_task_configs.json')
    catalog = {r['task_name']:r for r in read(GROUP/'task_catalog.json')}
    TASKS = {name: {'seconds':120, 'source':Path(catalog[name]['filename']).name,
                   'terminations':baseline[name]['terminations']} for name in manifest['tasks']}
    failures, episodes, sessions, request_ids = [], [], [], []
    required(digest(GROUP/'protocol.json') == metadata['protocol_sha256'], 'protocol_hash', failures)
    required(digest(GROUP/'catalog/official_task_configs.json') == metadata['official_task_configs_sha256'],
             'official_task_config_baseline_hash', failures)
    required(manifest['tasks'] == protocol['worker_tasks'][metadata['worker_index']], 'worker_task_assignment', failures)
    frozen = audit_protocol(base, REPO, manifest, failures, expected_model=MODEL)
    if args.partial and not (base/'episode_results.jsonl').exists():
        rows, ledger_hash, issues = [], None, [{'status': 'ledger_not_created_no_scored_episodes'}]
    else:
        rows, ledger_hash, issues = jsonl_snapshot(base/'episode_results.jsonl', partial=args.partial)
    required(not any(issue['status']=='invalid_json_record' for issue in issues), 'ledger_parse_error', failures)
    keys = [(row['env_name'], row['seed']) for row in rows]
    expected = {(task, seed) for task in TASKS for seed in range(10)}
    required(len(keys) == len(set(keys)) and set(keys) <= expected, 'unique_expected_slots', failures)
    if args.final:
        required(set(keys) == expected and len(rows) == len(expected), 'worker_coverage_incomplete', failures)
    configs = {}
    for task in TASKS:
        if any(row['env_name'] == task for row in rows):
            configs[task] = audit_task_config(base, task, failures)
    for row in rows:
        try:
            episode, session, ids = audit_episode(base, row, expected_model=MODEL)
            episodes.append(episode)
            sessions.append(session)
            request_ids.extend(ids)
            failures.extend(f"{row['env_name']}/{row['seed']}:{failure}" for failure in episode['failures'])
        except Exception as exc:
            failures.append(f"{row['env_name']}/{row['seed']}:{type(exc).__name__}:{exc}")
    interrupted = [audit_preserved_attempt(p.parent)
                   for p in sorted((base/'interrupted_attempts').glob('*/interruption.json'))]
    required(all(x['preservation_passed'] for x in interrupted), 'interrupted_archive_integrity', failures)
    required(all((x['metadata']['task'],x['metadata']['seed']) in expected for x in interrupted),
             'interrupted_unknown_slot', failures)
    valid_hashes = [identity_digest(session) for session in sessions]
    interrupted_hashes = [x['session_identity_sha256'] for x in interrupted if x['session_identity_sha256']]
    required(len(set(valid_hashes + interrupted_hashes)) == len(valid_hashes + interrupted_hashes),
             'session_reused_between_episodes_or_attempts', failures)
    required(len(request_ids) == len(set(request_ids)), 'reused_request_id', failures)
    # Final audit also requires no unscored live/stale API directory outside archives.
    pending = []
    for task in TASKS:
        for path in (base/task).glob('run_*_api/summary.json'):
            seed = int(path.parent.name.split('_')[1])
            if (task,seed) not in set(keys):
                pending.append({'task':task,'seed':seed,'path':str(path)})
    if args.final:
        required(not pending, 'unscored_unarchived_attempt', failures)
    result = {'checked_at_utc':datetime.now(timezone.utc).isoformat(), 'mode':'partial' if args.partial else 'final',
              'audit_script_sha256':digest(Path(__file__)), 'worker_index':metadata['worker_index'],
              'model':MODEL, 'expected_episodes':len(expected), 'audited_episodes':len(episodes),
              'official_successes':sum(row['success'] for row in rows), 'ledger_snapshot_sha256':ledger_hash,
              'protocol':frozen, 'task_runtime_configs':configs, 'episodes':episodes,
              'interrupted_attempts':interrupted, 'pending_unscored':pending,
              'missing_slots':[list(key) for key in sorted(expected-set(keys))], 'failures':failures,
              'audit_passed':not failures, 'final_acceptance':bool(args.final and not failures and len(episodes)==len(expected))}
    output = base/'validation'/('partial_acceptance_audit.json' if args.partial else 'final_acceptance_audit.json')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('mode','worker_index','audited_episodes','official_successes','audit_passed','final_acceptance','failures')},ensure_ascii=False,indent=2))
    return 0 if result['audit_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
