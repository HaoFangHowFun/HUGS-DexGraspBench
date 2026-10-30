"""Learning's two-slot qpos contract with single-hand and dual metadata."""
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from task import convert_format as cf


class LearningMetadataTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.env = patch.dict(os.environ, HUGS_DATASET_ROOT=str(self.root))
        self.env.start()
        self.addCleanup(self.env.stop)
        cf.LEARNING_METADATA_CACHE.clear()
        self.scene_path = self.root / 'object/DGN_2k/scene_cfg/obj/pose.npy'
        self.scene_path.parent.mkdir(parents=True)
        self.scene = {'task': {'obj_name': 'obj'}, 'scene': {'obj': {
            'file_path': '../../../processed_data/obj/mesh/obj.obj',
            'pose': np.array([1, 2, 3, 1, 0, 0, 0]), 'scale': [.03] * 3}}}
        np.save(self.scene_path, self.scene)

    def metadata(self, group, right=22, left=0, arms=False):
        names = ([f'rh_j{i}' for i in range(right)] + [f'lh_j{i}' for i in range(left)])
        if arms:
            names = ['la_x'] + names[::-1] + ['ra_x']
        p = self.root / 'BimanBODex_fixture/shadow' / group / 'metadata.json'
        p.parent.mkdir(parents=True)
        p.write_text(json.dumps({'joint_names': names, 'wrist_body_names': ['rh_palm', 'lh_palm']}))
        return cf._load_learning_metadata(str(self.scene_path), 'shadow', group)

    def test_right_only_all_stages_and_type_filter(self):
        m = self.metadata('right_full')
        source = self.root / 'samples'
        source.mkdir()
        raw = {'scene_path': 'object/DGN_2k/scene_cfg/obj/pose.npy', 'pred_grasp_type_id': 3}
        for i, stage in enumerate(cf.LEARNING_STAGE_NAMES):
            q = np.arange(58, dtype=np.float32) + i * 100
            q[3:7] = [0, 1, 0, 0]  # nonidentity wxyz, kept unchanged
            raw[stage + '_qpos'] = q
        np.save(source / 'sample.npy', raw)
        c = NS(task=NS(data_path=str(source), learning_metadata_group='right_full'),
               hand=NS(mocap=True), hand_name='shadow', exp_name='test_right_full',
               grasp_dir=str(self.root / 'converted'))
        cf.Learning((str(source / 'sample.npy'), c))
        out = np.load(self.root / 'converted/sample.npy', allow_pickle=True).item()
        self.assertEqual(out['joint_names'], m['right_hand_joint_names'])
        self.assertEqual(out['pred_grasp_type'], 'right_full')
        for stage in cf.LEARNING_STAGE_NAMES:
            q = out[stage + '_qpos']
            self.assertEqual(q.shape, (29,))
            self.assertTrue(np.isfinite(q).all())
            np.testing.assert_array_equal(q, raw[stage + '_qpos'][:29])
        np.testing.assert_array_equal(out['obj_pose'], self.scene['scene']['obj']['pose'])
        self.assertEqual(out['obj_scale'], .03)
        raw['pred_grasp_type_id'] = 5
        np.save(source / 'both.npy', raw)
        cf.Learning((str(source / 'both.npy'), c))
        self.assertFalse((self.root / 'converted/both.npy').exists())

    def test_both_hand_split_and_joint_assembly(self):
        m = self.metadata('both_full', right=2, left=3, arms=True)
        q = np.arange(19, dtype=np.float32)
        rp, rj, lp, lj = cf._split_learning_robot_qpos(q, m)
        np.testing.assert_array_equal(rp, q[:7])
        np.testing.assert_array_equal(rj, q[7:9])
        np.testing.assert_array_equal(lp, q[9:16])
        np.testing.assert_array_equal(lj, q[16:])
        with patch.object(cf, '_solve_dummy_arm_qpos', side_effect=[np.array([101]), np.array([102])]):
            out = cf._convert_learning_stage_dual(q, m, {})
        values = dict(zip(m['right_hand_joint_names'], rj))
        values.update(zip(m['left_hand_joint_names'], lj))
        values.update(ra_x=101, la_x=102)
        np.testing.assert_array_equal(out, [values[n] for n in m['joint_names']])
        np.testing.assert_array_equal(cf._convert_learning_stage_single(q, m), q[:9])

    def test_left_metadata_is_not_interpreted_as_right(self):
        m = self.metadata('left_full', right=0, left=2)
        rp, rj, lp, lj = cf._split_learning_robot_qpos(np.arange(18), m)
        self.assertIsNone(rp)
        self.assertIsNone(rj)
        np.testing.assert_array_equal(lp, np.arange(9, 16))
        np.testing.assert_array_equal(lj, [16, 17])
        with self.assertRaisesRegex(ValueError, 'requires right hand'):
            cf._convert_learning_stage_single(np.arange(18), m)

    def test_missing_dual_metadata_and_invalid_payload_fail(self):
        m = self.metadata('right_full')
        with self.assertRaisesRegex(ValueError, 'both-hand conversion requires'):
            cf._get_learning_ik_solver(NS(), m, 'shadow')
        for size in (29, 36, 57, 59):
            with self.assertRaisesRegex(ValueError, 'Unexpected qpos size'):
                cf._convert_learning_stage_single(np.zeros(size), m)
        q = np.zeros(58); q[0] = np.nan
        with self.assertRaisesRegex(ValueError, 'NaN or Inf'):
            cf._convert_learning_stage_single(q, m)


if __name__ == '__main__':
    unittest.main()
