from __future__ import annotations

import unittest
from unittest import mock

import torch

from isaaclab_compat import (
    isaac_quat_to_wxyz,
    isaac_runtime_quaternions_are_xyzw,
)


class IsaacLabQuaternionCompatTests(unittest.TestCase):
    def test_isaac6_xyzw_is_reordered_for_unitree(self):
        quaternion_xyzw = torch.tensor(
            [
                [0.0, 0.0, 0.0, 1.0],
                [0.0, 0.0, -0.70710678, 0.70710678],
            ]
        )

        actual = isaac_quat_to_wxyz(quaternion_xyzw, input_is_xyzw=True)

        expected = torch.tensor(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.70710678, 0.0, 0.0, -0.70710678],
            ]
        )
        torch.testing.assert_close(actual, expected)

    def test_isaac5_wxyz_is_preserved(self):
        quaternion_wxyz = torch.tensor([0.70710678, 0.0, 0.0, -0.70710678])

        actual = isaac_quat_to_wxyz(quaternion_wxyz, input_is_xyzw=False)

        self.assertIs(actual, quaternion_wxyz)

    def test_invalid_shape_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "last dimension to be 4"):
            isaac_quat_to_wxyz(torch.zeros(3), input_is_xyzw=True)

    def test_runtime_order_uses_package_metadata_without_importing_kit(self):
        isaac_runtime_quaternions_are_xyzw.cache_clear()
        with mock.patch("isaaclab_compat.importlib_metadata.version", return_value="6.0.0.0"):
            self.assertTrue(isaac_runtime_quaternions_are_xyzw())

        isaac_runtime_quaternions_are_xyzw.cache_clear()
        with mock.patch("isaaclab_compat.importlib_metadata.version", return_value="5.1.0"):
            self.assertFalse(isaac_runtime_quaternions_are_xyzw())

        isaac_runtime_quaternions_are_xyzw.cache_clear()


if __name__ == "__main__":
    unittest.main()
