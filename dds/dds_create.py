# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
from dds.dds_master import dds_manager
from robots.sonic_multi_robot import sonic_robot_channel_specs


def _cleanup_unregistered_dds_object(obj):
    """Close shared-memory handles owned by an object rejected by the manager."""

    for attribute_name in ("input_shm", "output_shm"):
        shared_memory = getattr(obj, attribute_name, None)
        if shared_memory is None:
            continue
        try:
            shared_memory.cleanup()
        except Exception as exc:
            print(
                "[dds_create] failed to clean unregistered DDS object "
                f"{attribute_name}: {exc}"
            )
        finally:
            setattr(obj, attribute_name, None)


def _register_required_dds_object(name, obj):
    """Register a required multi-robot channel or fail before DDS startup."""

    try:
        registered = dds_manager.register_object(name, obj)
    except Exception:
        _cleanup_unregistered_dds_object(obj)
        raise
    if not registered:
        _cleanup_unregistered_dds_object(obj)
        raise RuntimeError(f"required DDS object {name!r} could not be registered")


def create_dds_objects(args_cli,env):
    publish_names = []
    subscribe_names = []
    try:
        if args_cli.robot_type=="g129" or args_cli.robot_type=="h1_2":
            from dds.g1_robot_dds import G1RobotDDS
            g1_robot = G1RobotDDS()
            dds_manager.register_object("g129", g1_robot)
            publish_names.append("g129")
            subscribe_names.append("g129")
        if args_cli.enable_dex3_dds:
            from dds.dex3_dds import Dex3DDS
            dex3 = Dex3DDS()
            dds_manager.register_object("dex3", dex3)
            publish_names.append("dex3")
            subscribe_names.append("dex3")
        elif args_cli.enable_dex1_dds:
            from dds.gripper_dds import GripperDDS
            gripper = GripperDDS()
            dds_manager.register_object("dex1", gripper)
            publish_names.append("dex1")
            subscribe_names.append("dex1")
        elif args_cli.enable_inspire_dds:
            from dds.inspire_dds import InspireDDS
            inspire = InspireDDS()
            dds_manager.register_object("inspire", inspire)
            publish_names.append("inspire")
            subscribe_names.append("inspire")
        # host 多机器人模式（工作包 B）：robot_2..5 各有一套隔离的 DDS 通道。
        # 话题前缀 rt/rN、shm 后缀 _rN 缺一不可——shm 同名会被静默 attach 共享
        # （见 G1RobotDDS docstring）。必须在 start_* 之前完成全部注册。
        sonic_robot_count = getattr(args_cli, "sonic_robot_count", None)
        if sonic_robot_count is None:
            sonic_robot_count = 2 if getattr(args_cli, "enable_second_robot_dds", False) else 1
        extra_specs = sonic_robot_channel_specs(int(sonic_robot_count))[1:]
        if extra_specs:
            from dds.g1_robot_dds import G1RobotDDS
            from dds.dex3_dds import Dex3DDS
            for spec in extra_specs:
                g1_robot_extra = G1RobotDDS(
                    node_name=f"g1_robot_r{spec.robot_id}",
                    topic_prefix=spec.topic_prefix,
                    shm_suffix=spec.shm_suffix,
                )
                _register_required_dds_object(spec.robot_dds_name, g1_robot_extra)
                publish_names.append(spec.robot_dds_name)
                subscribe_names.append(spec.robot_dds_name)
                dex3_extra = Dex3DDS(
                    node_name=f"dex3_r{spec.robot_id}",
                    topic_prefix=spec.topic_prefix,
                    shm_suffix=spec.shm_suffix,
                )
                _register_required_dds_object(spec.dex3_dds_name, dex3_extra)
                publish_names.append(spec.dex3_dds_name)
                subscribe_names.append(spec.dex3_dds_name)
        if "Wholebody" in args_cli.task or args_cli.enable_wholebody_dds:
            from dds.commands_dds import RunCommandDDS
            run_command_dds = RunCommandDDS()
            dds_manager.register_object("run_command", run_command_dds)
            publish_names.append("run_command")
            subscribe_names.append("run_command")
        from dds.reset_pose_dds import ResetPoseCmdDDS
        reset_pose_dds = ResetPoseCmdDDS()
        dds_manager.register_object("reset_pose", reset_pose_dds)
        subscribe_names.append("reset_pose")
        from dds.sim_state_dds import SimStateDDS
        sim_state_dds = SimStateDDS(env,args_cli.task)
        dds_manager.register_object("sim_state", sim_state_dds)
        publish_names.append("sim_state")
        from dds.rewards_dds import RewardsDDS
        rewards_dds = RewardsDDS(env,args_cli.task)
        dds_manager.register_object("rewards", rewards_dds)
        publish_names.append("rewards")

        dds_manager.start_publishing(publish_names)
        dds_manager.start_subscribing(subscribe_names)
        return reset_pose_dds,sim_state_dds,dds_manager
    except Exception:
        try:
            dds_manager.cleanup()
        except Exception as cleanup_exc:
            print(f"[dds_create] rollback cleanup failed: {cleanup_exc}")
        raise


def create_dds_objects_replay(args_cli,env):
    publish_names = []
    subscribe_names = []
    if args_cli.robot_type=="g129" or args_cli.robot_type=="h1_2":
        from dds.g1_robot_dds import G1RobotDDS
        g1_robot = G1RobotDDS()
        dds_manager.register_object("g129", g1_robot)
        publish_names.append("g129")
        subscribe_names.append("g129")
    if args_cli.enable_dex3_dds:
        from dds.dex3_dds import Dex3DDS
        dex3 = Dex3DDS()
        dds_manager.register_object("dex3", dex3)
        publish_names.append("dex3")
        subscribe_names.append("dex3")
    elif args_cli.enable_dex1_dds:
        from dds.gripper_dds import GripperDDS
        gripper = GripperDDS()
        dds_manager.register_object("dex1", gripper)
        publish_names.append("dex1")
        subscribe_names.append("dex1")
    elif args_cli.enable_inspire_dds:
        from dds.inspire_dds import InspireDDS
        inspire = InspireDDS()
        dds_manager.register_object("inspire", inspire)
        publish_names.append("inspire")
        subscribe_names.append("inspire")

    dds_manager.start_publishing(publish_names)
    dds_manager.start_subscribing(subscribe_names)
