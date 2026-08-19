from __future__ import annotations

from pathlib import Path
import subprocess
import tempfile
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

    def test_five_robot_host_and_staged_count_are_forwarded(self) -> None:
        self.assertIn('SONIC_ROBOT_COUNT="${PIPELINE_SONIC_ROBOT_COUNT:-5}"', self.source)
        self.assertIn('ISAACLAB_SONIC_ROBOT_COUNT="$SONIC_ROBOT_COUNT"', self.source)
        self.assertIn("2|3|4|5", self.source)

    def test_default_paths_follow_checkout_and_current_user(self) -> None:
        self.assertIn('DEFAULT_SIM_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)', self.source)
        self.assertIn('SIM_DIR="${PIPELINE_SIM_DIR:-$DEFAULT_SIM_DIR}"', self.source)
        self.assertIn('GR00T_ROOT="${GR00T_WBC_ROOT:-$HOME/GR00T-WholeBodyControl}"', self.source)
        self.assertNotIn("/home/nolo/", self.source)
        self.assertIn('if [ ! -d "$DEPLOY_DIR" ]', self.source)
        self.assertIn('if [ ! -x "$PY" ]', self.source)

    def test_extra_deploys_start_after_first_two_and_use_isolated_topics(self) -> None:
        extra_marker = "# robot_3..N："
        self.assertLess(self.source.index("== start deploy#1"), self.source.index(extra_marker))
        self.assertLess(self.source.index("== start deploy#2"), self.source.index(extra_marker))
        extra_block = self.source.split(extra_marker, maxsplit=1)[1].split(
            "# manager 拿到第一帧", maxsplit=1
        )[0]
        self.assertIn('for ROBOT_ID in $(seq 3 "$SONIC_ROBOT_COUNT")', extra_block)
        self.assertIn('SONIC_DDS_TOPIC_PREFIX="rt/r$ROBOT_ID"', extra_block)
        self.assertIn('--zmq-out-port "$ROBOT_DEBUG_PORT"', extra_block)
        self.assertIn('--udp-out-port "$ROBOT_DEBUG_PORT"', extra_block)

    def test_all_extra_channels_are_waited_and_started(self) -> None:
        self.assertIn(
            'wait_for "sonic_dds:r2.*CONTROL marker received"', self.source
        )
        self.assertIn(
            'wait_for "Init Done" "$LOG_DIR/deploy_r$ROBOT_ID.log"', self.source
        )
        self.assertIn(
            'wait_for "sonic_dds:r$ROBOT_ID.*CONTROL marker received"', self.source
        )

    def test_all_deploy_pids_are_recorded_from_exec_wrappers(self) -> None:
        self.assertIn("declare -a DEPLOY_PIDS=()", self.source)
        self.assertIn("DEPLOY_PIDS[1]=$!", self.source)
        self.assertIn("DEPLOY_PIDS[2]=$!", self.source)
        self.assertIn("DEPLOY_PIDS[$ROBOT_ID]=$!", self.source)
        self.assertIn('exec bash deploy.sh --disable-crc-check', self.source)
        self.assertIn('exec env G1_LOCAL_ROBOT_ID=2 bash deploy.sh', self.source)
        self.assertIn('check_process_alive "$deploy_pid" "deploy#$robot_id"', self.source)

    def test_single_and_dual_pico_manager_pids_are_tracked(self) -> None:
        self.assertIn("declare -a MANAGER_PIDS=()", self.source)
        self.assertEqual(self.source.count("MANAGER_PIDS[1]=$!"), 2)
        self.assertEqual(self.source.count("MANAGER_PIDS[2]=$!"), 1)
        self.assertIn('exec env XROBO_TRANSPORT=udp', self.source)
        self.assertIn(
            'check_process_alive "$manager_pid" "pico manager#$manager_id"',
            self.source,
        )
        self.assertIn('if [ "$DUAL_PICO" = "1" ]; then\n    manager_count=2', self.source)

    def test_done_is_gated_by_liveness_and_two_physics_progress_probes(self) -> None:
        gate = self.source.index("check_tracked_processes_alive || exit 1")
        probe_one = self.source.index(
            'wait_for_physics_progress "$PHYSICS_BASELINE" 60 "physics progress probe#1"'
        )
        probe_two = self.source.index(
            'wait_for_physics_progress "$PHYSICS_PROBE_1" 60 "physics progress probe#2"'
        )
        first_done = self.source.index('echo "BRINGUP_DONE')

        self.assertLess(gate, probe_one)
        self.assertLess(probe_one, probe_two)
        self.assertLess(probe_two, first_done)
        self.assertIn('[ "$current" -gt "$baseline" ]', self.source)
        self.assertIn('GR00T_WBC_ROOT="$GR00T_ROOT" PYTHONUNBUFFERED=1', self.source)
        self.assertIn("--profile_interval 25", self.source)
        self.assertIn("--sim-state-export-hz 0", self.source)

    def test_physics_steps_parser_returns_last_complete_counter(self) -> None:
        start = self.source.index("latest_physics_steps() {")
        end = self.source.index("\n}\n\ncheck_process_alive()", start) + 3
        helper = self.source[start:end]
        with tempfile.NamedTemporaryFile("w", encoding="utf-8") as log_file:
            log_file.write(
                "noise physics_steps=7 sync_waits=1\n"
                "truncated physics_steps=\n"
                "two fields physics_steps=18 x physics_steps=23, sync_waits=2\n"
            )
            log_file.flush()
            completed = subprocess.run(
                ["bash", "-c", f'{helper}\nlatest_physics_steps "$1"', "_", log_file.name],
                check=True,
                capture_output=True,
                text=True,
            )
        self.assertEqual(completed.stdout.strip(), "23")


if __name__ == "__main__":
    unittest.main()
