"""Wire contract tests with an in-process HTTP server substitute."""
import base64
import io
import json

import httpx
import numpy as np
from PIL import Image
import pytest

from policies.roboapi.transport import SessionPolicyClient, PolicyUnavailable

RID = 'a' * 32


def response(turn=1):
    return {'model': 'test-model', 'choices': [{'finish_reason': 'tool_calls', 'message': {
        'role': 'assistant', 'content': None, 'tool_calls': [{'id': 'call_' + RID,
        'type': 'function', 'function': {'name': 'execute_actions', 'arguments':
        json.dumps({'output': [[.1, .2, .3, 0, 0, 0, 2, .25], [1, 2, 3, 0, 1, 0, 0, .75]]})}}]}}],
        'metadata': {'session_id': 'test-session', 'turn': str(turn), 'next_turn': str(turn+1)}}


def client(tmp_path, mutate=None):
    requests = []
    def handler(request):
        body = json.loads(request.content) if request.content else None
        requests.append((request.url.path, body))
        assert request.headers['Authorization'] == 'Bearer test-credential'
        if request.url.path.endswith('/models'):
            data = {'data': [{'id': 'test-model'}]}
        elif request.url.path.endswith('/sessions'):
            data = {'id': 'test-session', 'model': 'test-model', 'next_turn': 1, 'messages': []}
        elif request.url.path.endswith('/close'):
            data = {'id': 'test-session', 'messages': [], 'status': 'closed'}
        else:
            turn = int(body['metadata']['turn'])
            data = response(turn)
            if mutate:
                data = mutate(data)
        return httpx.Response(200, json=data, headers={'X-Request-ID': RID})
    key = tmp_path / 'key'
    key.write_text('test-credential\n')
    c = SessionPolicyClient('https://example.invalid/v1', key, 'test-model',
                            http_transport=httpx.MockTransport(handler))
    c.connect()
    c.begin_session()
    return c, requests


def infer(c):
    return c.infer(np.array([.1, .2, .3, 1, 0, 0, 0]), .75,
                   np.full((360, 640, 3), 41, dtype=np.uint8), 'place object')


def test_wire_pixels_pose_full_chunk_and_execution_receipts(tmp_path):
    c, requests = client(tmp_path)
    chunk = infer(c)
    assert len(chunk) == 2 and chunk.action_dt == 1/30
    np.testing.assert_array_equal(chunk.arm[:, 3:], [[1, 0, 0, 0], [0, 0, 1, 0]])
    np.testing.assert_array_equal(chunk.gripper, [.25, .75])
    first = requests[-1][1]
    text, image = first['messages'][0]['content']
    state = json.loads(text['text'])
    assert state['instruction'] == 'place object'
    np.testing.assert_allclose(state['state'], [.1, .2, .3, 0, 0, 0, 1, .75])
    pixels = np.asarray(Image.open(io.BytesIO(base64.b64decode(image['image_url']['url'].split(',')[1]))))
    np.testing.assert_array_equal(pixels, np.full((360, 640, 3), 41, dtype=np.uint8))
    with pytest.raises(PolicyUnavailable):
        infer(c)  # No request before execution has been acknowledged.
    c.record_execution(chunk, 2, completed=True)
    second = infer(c)
    sent = requests[-1][1]
    assert [m['role'] for m in sent['messages']] == ['assistant', 'tool', 'user']
    assert json.loads(sent['messages'][1]['content']) == {'executed_steps': 2, 'status': 'completed'}
    c.record_execution(second, 1, completed=False)
    c.end_session()
    assert requests[-1][0].endswith('/close')
    assert json.loads(requests[-1][1]['messages'][1]['content']) == {'executed_steps': 1, 'status': 'interrupted'}
    assert c.session_status['status'] == 'closed'
    c.close()


def corrupt(data, case):
    if case == 'model': data['model'] = 'other-model'
    elif case == 'turn': data['metadata']['turn'] = '9'
    elif case == 'tool': data['choices'][0]['message']['tool_calls'][0]['function']['name'] = 'other'
    else:
        values = {'empty': [], 'zero_quaternion': [[0]*8], 'nan': [[float('nan')]*8],
                  'gripper': [[0, 0, 0, 0, 0, 0, 1, 1.2]]}
        data['choices'][0]['message']['tool_calls'][0]['function']['arguments'] = json.dumps({'output': values[case]})
    return data


@pytest.mark.parametrize('case', ['model', 'turn', 'tool', 'empty', 'zero_quaternion', 'nan', 'gripper'])
def test_invalid_response_cannot_be_retried(tmp_path, case):
    c, requests = client(tmp_path, lambda data: corrupt(data, case))
    with pytest.raises(PolicyUnavailable): infer(c)
    count = len(requests)
    with pytest.raises(PolicyUnavailable): infer(c)
    assert len(requests) == count and c.session_status['status'] == 'unknown'
    c.end_session()
    c.close()


def test_timed_out_request_is_not_retried_or_exposed(tmp_path):
    def fail(data):
        raise httpx.ReadTimeout('private response test-credential')
    c, requests = client(tmp_path, fail)
    with pytest.raises(PolicyUnavailable) as exc:
        infer(c)
    assert 'test-credential' not in str(exc.value)
    with pytest.raises(PolicyUnavailable): infer(c)
    assert len(requests) == 3
    c.end_session()
    c.close()


def test_session_records_alias_resolution_and_requires_that_version(tmp_path):
    c, requests = client(tmp_path)
    # Keep the requested alias for requests; the session pins the server version.
    c.model = 'test-alias'
    chunk = infer(c)
    assert requests[-1][1]['model'] == 'test-alias'
    assert c.session_status['model'] == 'test-model'
    c.record_execution(chunk, len(chunk), completed=True)
    c.end_session()
    c.close()
