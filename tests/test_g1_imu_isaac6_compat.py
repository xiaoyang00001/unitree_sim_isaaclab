from __future__ import annotations

import importlib.util
import os
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import torch

from isaaclab_compat import isaac_quat_to_wxyz


os.environ.setdefault("UNITREE_DDS_DOMAIN", "92")
os.environ.setdefault("UNITREE_DDS_INTERFACE", "lo")

module_spec = importlib.util.spec_from_file_location(
    "g1_29dof_state_imu_test_module",
    Path(__file__).parents[1] / "tasks/common_observations/g1_29dof_state.py",
)
g1_state = importlib.util.module_from_spec(module_spec)
module_spec.loader.exec_module(g1_state)


class G1ImuIsaac6CompatTests(unittest.TestCase):
    def test_xyzw_identity_is_published_as_wxyz_and_rotated_consistently(self):
        data = SimpleNamespace(
            body_names=["pelvis"],
            body_link_pose_w=torch.tensor(
                [[[1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]]]
            ),
            body_link_vel_w=torch.zeros(1, 1, 6),
            body_com_acc_w=torch.zeros(1, 1, 6),
        )
        env = SimpleNamespace(
            scene={"robot": SimpleNamespace(data=data)},
            cfg=SimpleNamespace(sim=SimpleNamespace(gravity=(0.0, 0.0, -9.81))),
        )

        def convert_xyzw(quaternion):
            return isaac_quat_to_wxyz(quaternion, input_is_xyzw=True)

        with mock.patch.object(g1_state, "isaac_quat_to_wxyz", side_effect=convert_xyzw):
            actual = g1_state.get_robot_imu_data(env, body_candidates=("pelvis",))

        expected = torch.tensor(
            [[1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0]]
        )
        torch.testing.assert_close(actual, expected)


if __name__ == "__main__":
    unittest.main()
