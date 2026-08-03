#!/bin/bash
# Pico VR 控制 bring-up（pipeline 双机器人，工作包 Pico-①）：
#   sim(HOST_MODE 双机器人) + deploy#1(--input-type zmq_manager ← Pico manager)
#   + deploy#2(keyboard 调试通道) + pico_manager_thread_server(--manager)
#
# 与 keyboard 版编排的差异：
#   - deploy#1 的输入从 stdin keyboard 换成 ZMQ(localhost:5556)，发车不再靠 ']'，
#     而是 manager 在操作者按 A+B+X+Y 进 PLANNER 模式时发 command(start=True)。
#     manager 起点用 --no_auto_pose：头显数据一到就自动进 POSE 全身跟随太危险，
#     改为操作者显式按键发车。
#   - manager 走 XROBO_TRANSPORT=udp：头显端 GameLink/定制 Unity 发送端直发本机
#     63901/udp（JSON 里含全身追踪+手柄按键/摇杆），不需要 XRoboToolkit PC Service。
#   - 锁步注意：双 ack AND 门下 deploy#1/#2 仍必须并行启动（串行会自锁）；
#     ack 在 Init 后即持续回，(操作者未发车时机器人保持默认站姿，物理照常推进)。
#
# 操作者手册（头显侧）：
#   1. GameLink 目标 IP 指向本机（192.168.50.68），端口 63901 —— 改
#      /sdcard/Android/data/<GameLink包名>/files/ 下 JSON 副本后重启 app 即可。
#   2. 戴上头显、手柄唤醒，确认 pico_manager.log 出现 body 数据（不再刷 waiting）。
#   3. 按 A+B+X+Y（四键同按，瞬按）→ 进 PLANNER：左摇杆=行走方向、右摇杆=转向；
#      A+B 升档(SLOW_WALK→WALK→RUN...)、X+Y 降档。
#   4. A+X 切 POSE 全身跟随（校准帧=进入时刻的身体姿态）；左摇杆按下切 VR_3PT。
#   5. 再按 A+B+X+Y = 急停（manager 退出，deploy 停控）。
set -u
LOG_DIR="${PIPELINE_LOG_DIR:-/tmp/pipeline_pico}"
SIM_DIR="${PIPELINE_SIM_DIR:-/home/nolo/unitree_sim_isaaclab-pipeline}"
GR00T_ROOT="${GR00T_WBC_ROOT:-/home/nolo/GR00T-WholeBodyControl}"
DEPLOY_DIR="$GR00T_ROOT/gear_sonic_deploy"
PY="${PIPELINE_SIM_PY:-/home/nolo/miniconda3/envs/env_isaaclab/bin/python}"
MGR_PY="$GR00T_ROOT/.venv_teleop/bin/python"
PEER_IP="${ISAACLAB_SCENE_SYNC_PEER_IP:-192.168.50.127}"
mkdir -p "$LOG_DIR"

# 渲染形态：默认 GUI（--hide_ui）——需要一个能用的 X 会话，DISPLAY 优先取调用方
# 环境、否则 :0（PIPELINE_DISPLAY 可强制指定）。无桌面/纯 ssh 的机器用
# PIPELINE_HEADLESS=1 切 --no_render（headless experience，不碰窗口子系统；host
# 本地无画面，物理/锁步/viewer 均不受影响）。不预检硬跑的下场：kit 起到 ~5s 在
# RTX 插件初始化段错误（日志指纹 = "GLFW initialization failed" ×3 → SIGSEGV）。
# ⚠️ --device cpu 只切 PhysX 后端、不关渲染，与这个崩溃无关，别为此去掉它。
RENDER_ARG="--hide_ui"
DISPLAY_TARGET="${PIPELINE_DISPLAY:-${DISPLAY:-:0}}"
if [ "${PIPELINE_HEADLESS:-0}" = "1" ]; then
  RENDER_ARG="--no_render"
elif command -v xset >/dev/null 2>&1 && ! timeout 3 xset -display "$DISPLAY_TARGET" q >/dev/null 2>&1; then
  echo "ERROR: X display $DISPLAY_TARGET 不可用（ssh 无桌面会话 / 显示号不对）。"
  echo "  修法A: 在机器本地桌面终端跑，或 PIPELINE_DISPLAY=<实际显示> 指过去"
  echo "  修法B: PIPELINE_HEADLESS=1 bash $0   # 无本地画面，其余功能不受影响"
  exit 1
fi

wait_for() {  # wait_for <pattern> <file> <timeout_s> <label>
  local waited=0
  until grep -q "$1" "$2" 2>/dev/null; do
    sleep 3
    waited=$((waited + 3))
    if [ "$waited" -ge "$3" ]; then
      echo "TIMEOUT waiting for $4 ($1)"
      return 1
    fi
  done
  echo "OK: $4"
}

echo "== stop old processes (hard) =="
pkill -INT -f "sim_mai[n].py" 2>/dev/null
sleep 5
pkill -9 -f "g1_deploy_onnx_re[f]" 2>/dev/null
pkill -9 -f "deploy.s[h]" 2>/dev/null
pkill -9 -f "pico_manager_thread_serve[r]" 2>/dev/null
pkill -f "tail -f.*dk_r[12]" 2>/dev/null
sleep 4
pgrep -f "sim_mai[n]|g1_deploy_onnx_re[f]" >/dev/null && { pkill -9 -f "sim_mai[n]|g1_deploy_onnx_re[f]"; sleep 3; }
echo "residual deploys: $(pgrep -c -f 'g1_deploy_onnx_re[f]' 2>/dev/null || echo 0)"

echo "== start host sim (dual robot) =="
cd "$SIM_DIR" || exit 1
env DISPLAY="$DISPLAY_TARGET" GR00T_WBC_ROOT="$GR00T_ROOT" \
    UNITREE_DDS_DOMAIN=1 UNITREE_DDS_INTERFACE=lo \
    ISAACLAB_LOCAL_ROBOT_ID=1 ISAACLAB_HOST_BOTH_ROBOTS=1 \
    ISAACLAB_SCENE_SYNC_PEER_IP="$PEER_IP" \
    UNITREE_SKIP_LOWSTATE_CRC=1 UNITREE_LOWCMD_CRC_SAMPLE_INTERVAL=50 \
    "$PY" sim_main.py --task Isaac-G1-29DoF-Sonic-Conveyor --robot_type g129 \
    --action_source sonic_dds --device cpu $RENDER_ARG --stats_interval 10 \
    > "$LOG_DIR/host_dual.log" 2>&1 &

# 双 ack AND 门：两个 deploy 必须并行启动（.trt 缓存只读共享，build 用 20s 错峰）。
echo "== start deploy#1 (rt/*, input=zmq_manager) =="
echo "" > "$LOG_DIR/dk_r1"
( cd "$DEPLOY_DIR" && tail -f "$LOG_DIR/dk_r1" | bash deploy.sh --disable-crc-check \
    --input-type zmq_manager isaac > "$LOG_DIR/deploy_r1.log" 2>&1 ) &
sleep 20

echo "== start deploy#2 (rt/r2/*, input=keyboard) =="
echo "" > "$LOG_DIR/dk_r2"
( cd "$DEPLOY_DIR" && tail -f "$LOG_DIR/dk_r2" | env G1_LOCAL_ROBOT_ID=2 bash deploy.sh \
    --disable-crc-check --input-type keyboard isaac > "$LOG_DIR/deploy_r2.log" 2>&1 ) &

echo "== start pico manager (udp transport, PUB :5556) =="
# manager 先等 63901/udp 的头显数据、拿到第一帧才 bind 5556——deploy#1 的 SUB 是
# connect 语义，先起后起都能接上；command 有 1Hz keepalive 兜 slow-joiner。
( cd "$GR00T_ROOT" && env XROBO_TRANSPORT=udp PYTHONUNBUFFERED=1 "$MGR_PY" \
    gear_sonic/scripts/pico_manager_thread_server.py --manager --no_auto_pose \
    --port 5556 > "$LOG_DIR/pico_manager.log" 2>&1 ) &

wait_for "Init Done" "$LOG_DIR/deploy_r1.log" 300 "deploy#1 Init Done" || exit 1
wait_for "Init Done" "$LOG_DIR/deploy_r2.log" 300 "deploy#2 Init Done" || exit 1
grep -m1 "DDS topics" "$LOG_DIR/deploy_r1.log" || true
grep -m1 "DDS topics" "$LOG_DIR/deploy_r2.log" || true

echo "== wait sim STARTUP HOLD, start channel #2 (keyboard) =="
wait_for "STARTUP HOLD" "$LOG_DIR/host_dual.log" 300 "sim STARTUP HOLD" || exit 1
sleep 2
printf ']' >> "$LOG_DIR/dk_r2"
wait_for "CONTROL marker received" "$LOG_DIR/host_dual.log" 60 "channel#2 CONTROL" || exit 1
# 帧率三件套之三：提优先级（sim + 双 deploy）
sudo -n renice -n -10 -p $(pgrep -f "sim_mai[n].py" | head -1) \
    $(pgrep -f "g1_deploy_onnx_re[f]" | tr '\n' ' ') 2>/dev/null || true
# 通道 #2 planner 预激活（行走靠往 dk_r2 里发 w/s/a/d）
sleep 3; printf '\n' >> "$LOG_DIR/dk_r2"
sleep 3; printf '2' >> "$LOG_DIR/dk_r2"
echo "BRINGUP_DONE (robot#1 等操作者头显发车: A+B+X+Y)"
echo "logs: $LOG_DIR/{host_dual,deploy_r1,deploy_r2,pico_manager}.log"
