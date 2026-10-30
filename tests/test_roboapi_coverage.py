"""Synthetic result coverage and explicit model identity checks."""
import json

import pytest

from policies.roboapi.config import MODEL
from tools.roboapi_eval.full120_collect_results import validate_coverage
from tools.roboapi_eval.audit_full120_worker import audit_model_identity


def records():
    tasks = [f'TestTask{i}' for i in range(120)]
    rows = [dict(env_name=t, seed=s, run=s, model=MODEL, success=False,
                 episode_length_limit_s=120, episode_step=3600, dt=1/30, duration=120.)
            for t in tasks for s in range(10)]
    return tasks, rows


def test_complete_unique_coverage():
    tasks, rows = records()
    validate_coverage(rows, tasks)


@pytest.mark.parametrize('case', ['missing', 'duplicate', 'model', 'short_failure'])
def test_incomplete_or_invalid_outcomes_are_rejected(case):
    tasks, rows = records()
    if case == 'missing': rows.pop()
    elif case == 'duplicate': rows[-1] = rows[0]
    elif case == 'model': rows[0]['model'] = 'other-model'
    elif case == 'short_failure': rows[0]['episode_step'] = 30
    with pytest.raises(ValueError): validate_coverage(rows, tasks)


def test_error_response_does_not_invent_model_identity(tmp_path):
    request = {'method': 'POST', 'path': '/v1/chat/completions', 'body': {'model': MODEL}}
    (tmp_path/'http_0001_request.json').write_text(json.dumps(request))
    path = tmp_path/'http_0001_response.json'
    path.write_text(json.dumps({'http_status': 500, 'body': {'error': 'test'}}))
    report = audit_model_identity(tmp_path, expected_model=MODEL, summary={'model': MODEL})
    assert report['passed']
    assert report['checks'][-1]['model_field_present'] is False
    path.write_text(json.dumps({'http_status': 200, 'body': {'choices': []}}))
    assert not audit_model_identity(tmp_path, expected_model=MODEL, summary={'model': MODEL})['passed']


def test_requested_alias_and_resolved_version_have_separate_identity_checks(tmp_path):
    requested, resolved = 'test-alias', 'test-version'
    (tmp_path/'http_0001_request.json').write_text(json.dumps({
        'path': '/v1/sessions', 'body': {'model': requested}}))
    (tmp_path/'http_0001_response.json').write_text(json.dumps({
        'http_status': 200, 'body': {'model': resolved}}))
    summary = {'model': requested, 'resolved_model': resolved}
    assert audit_model_identity(tmp_path, expected_model=requested, summary=summary)['passed']
    summary['resolved_model'] = 'unexpected-version'
    assert not audit_model_identity(tmp_path, expected_model=requested, summary=summary)['passed']
