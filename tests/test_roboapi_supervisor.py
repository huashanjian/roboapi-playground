import json
import os
from pathlib import Path
import tempfile
import unittest

from tools.roboapi_eval.full120_supervisor import archive_interrupted, process_token, rows


class SupervisorSafetyTests(unittest.TestCase):
    def test_process_identity_uses_live_proc_not_state_file(self):
        self.assertIsNotNone(process_token(os.getpid()))
        self.assertIsNone(process_token(999999999))

    def test_duplicate_slots_are_rejected_and_partial_line_is_not_scored(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            path = root / 'episode_results.jsonl'
            row = {'env_name': 'Task', 'seed': 0}
            path.write_text(json.dumps(row) + '\n' + '{"env_name":')
            self.assertEqual(rows(root, partial=True), [row])
            path.write_text((json.dumps(row) + '\n') * 2)
            with self.assertRaises(RuntimeError):
                rows(root)

    def test_archive_never_moves_scored_seed_or_overwrites_prior_attempt(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            task = root / 'Task'
            task.mkdir()
            (root / 'episode_results.jsonl').write_text(json.dumps({'env_name': 'Task', 'seed': 1}) + '\n')
            (task / 'run_1.hdf5').write_bytes(b'scored immutable evidence')
            (task / 'env_cfg.json').write_text('{}')
            log = root / 'runtime.log'
            log.write_text('HTTP failure')
            for index in range(2):
                api = task / 'run_0_api'
                api.mkdir()
                (api / 'summary.json').write_text(json.dumps({'requests': index + 1, 'responses': 0,
                    'executed_steps': 0, 'session_id': None, 'errors': [{'phase': 'begin_episode'}]}))
                (task / 'run_0_error.json').write_text('{"policy_success": null}')
                (task / 'run_0.hdf5').write_bytes(b'partial')
                (task / 'instruction_0.mp4').write_bytes(b'partial video')
                result = archive_interrupted(root, 'Task', log)
                self.assertEqual(result[0]['attempt_index'], index)
                self.assertFalse(result[0]['included_in_policy_success_denominator'])
                archived = root / 'interrupted_attempts' / f'Task_seed0_attempt{index}'
                self.assertTrue((archived / 'run_0.hdf5').exists())
                self.assertTrue((archived / 'archive_inventory.json').exists())
                self.assertFalse((task / 'run_0_api').exists())
                self.assertEqual((task / 'run_1.hdf5').read_bytes(), b'scored immutable evidence')
            self.assertEqual(len(list((root / 'interrupted_attempts').iterdir())), 2)


if __name__ == '__main__':
    unittest.main()
