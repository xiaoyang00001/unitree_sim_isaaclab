from __future__ import annotations

import ast
import math
import unittest
from pathlib import Path


SOURCE_PATH = (
    Path(__file__).parents[1]
    / "tasks/g1_tasks/g1_29dof_sonic_conveyor/conveyor_env_cfg.py"
)
SOURCE_TREE = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))


def _module_assignment(name: str) -> ast.expr:
    for node in SOURCE_TREE.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == name:
                    return node.value
    raise AssertionError(f"Missing module assignment: {name}")


def _scene_assignment(name: str) -> ast.expr:
    for node in SOURCE_TREE.body:
        if isinstance(node, ast.ClassDef) and node.name == "G129SonicConveyorSceneCfg":
            for statement in node.body:
                if not isinstance(statement, (ast.Assign, ast.AnnAssign)):
                    continue
                targets = statement.targets if isinstance(statement, ast.Assign) else [statement.target]
                if any(isinstance(target, ast.Name) and target.id == name for target in targets):
                    return statement.value
    raise AssertionError(f"Missing conveyor scene assignment: {name}")


def _first_keyword(expression: ast.expr, keyword_name: str) -> ast.expr:
    for node in ast.walk(expression):
        if isinstance(node, ast.keyword) and node.arg == keyword_name:
            return node.value
    raise AssertionError(f"Missing keyword {keyword_name!r}")


def _evaluate(expression: ast.expr, values: dict[str, object]) -> object:
    if isinstance(expression, ast.Constant):
        return expression.value
    if isinstance(expression, (ast.Tuple, ast.List)):
        return tuple(_evaluate(item, values) for item in expression.elts)
    if isinstance(expression, ast.Name):
        return values[expression.id]
    if isinstance(expression, ast.IfExp):
        condition = bool(_evaluate(expression.test, values))
        return _evaluate(expression.body if condition else expression.orelse, values)
    raise AssertionError(f"Unsupported static expression: {ast.dump(expression)}")


def _rotate_x_axis_xyzw(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float]:
    x, y, z, w = quaternion
    return (
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y + w * z),
        2.0 * (x * z - w * y),
    )


def _assert_vector_close(
    test_case: unittest.TestCase,
    actual: tuple[float, float, float],
    expected: tuple[float, float, float],
) -> None:
    for actual_value, expected_value in zip(actual, expected, strict=True):
        test_case.assertTrue(math.isclose(actual_value, expected_value, abs_tol=1.0e-6))


class ConveyorIsaac6OrientationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.values = {
            name: ast.literal_eval(_module_assignment(name))
            for name in (
                "_ISAAC6_QUAT_IDENTITY_XYZW",
                "_ISAAC6_QUAT_YAW_180_XYZW",
                "_ISAAC6_QUAT_YAW_POS_90_XYZW",
                "_ISAAC6_QUAT_ROLL_POS_45_XYZW",
            )
        }

    def test_robot_1_faces_robot_2_and_robot_2_faces_positive_x(self):
        robot_1_expression = _module_assignment("_ROBOT_1_ROT")
        robot_2_expression = _module_assignment("_ROBOT_2_ROT")

        normal_layout = dict(self.values, _ROBOT_YAW_IDENTITY=False)
        robot_1_quat = _evaluate(robot_1_expression, normal_layout)
        robot_2_quat = _evaluate(robot_2_expression, normal_layout)

        _assert_vector_close(self, _rotate_x_axis_xyzw(robot_1_quat), (-1.0, 0.0, 0.0))
        _assert_vector_close(self, _rotate_x_axis_xyzw(robot_2_quat), (1.0, 0.0, 0.0))

    def test_identity_override_keeps_both_robots_facing_positive_x(self):
        identity_layout = dict(self.values, _ROBOT_YAW_IDENTITY=True)
        robot_1_quat = _evaluate(_module_assignment("_ROBOT_1_ROT"), identity_layout)

        _assert_vector_close(self, _rotate_x_axis_xyzw(robot_1_quat), (1.0, 0.0, 0.0))

    def test_warehouse_rotates_about_z_without_tipping_up_axis(self):
        background_rot = _evaluate(_first_keyword(_scene_assignment("background"), "rot"), self.values)
        _assert_vector_close(self, _rotate_x_axis_xyzw(background_rot), (0.0, 1.0, 0.0))

    def test_conveyor_collider_uses_xyzw_identity(self):
        collider_rot = _evaluate(
            _first_keyword(_scene_assignment("conveyor_collider"), "rot"),
            self.values,
        )
        self.assertEqual(collider_rot, (0.0, 0.0, 0.0, 1.0))


if __name__ == "__main__":
    unittest.main()
