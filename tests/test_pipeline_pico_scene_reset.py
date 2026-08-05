from __future__ import annotations

from pathlib import Path
import unittest


BRINGUP = Path(__file__).resolve().parents[1] / "tools" / "pipeline_pico_bringup.sh"


class PipelinePicoSceneResetTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.source = BRINGUP.read_text(encoding="utf-8")

    def test_only_manager_one_has_global_scene_reset_authority(self) -> None:
        manager_one = self.source.split(
            'echo "== start pico manager#1', maxsplit=1
        )[1].split('echo "== start pico manager#2', maxsplit=1)[0]
        manager_two = self.source.split(
            'echo "== start pico manager#2', maxsplit=1
        )[1].split("else\n", maxsplit=1)[0]

        self.assertIn("--enable_isaac_scene_reset", manager_one)
        self.assertIn("UNITREE_DDS_DOMAIN=1 UNITREE_DDS_INTERFACE=lo", manager_one)
        self.assertNotIn("--enable_isaac_scene_reset", manager_two)

    def test_single_pico_manager_keeps_scene_reset_authority(self) -> None:
        single_manager = self.source.split(
            'echo "== start pico manager (udp transport', maxsplit=1
        )[1].split("fi\n", maxsplit=1)[0]

        self.assertIn("--enable_isaac_scene_reset", single_manager)
        self.assertIn("UNITREE_DDS_DOMAIN=1 UNITREE_DDS_INTERFACE=lo", single_manager)


if __name__ == "__main__":
    unittest.main()
