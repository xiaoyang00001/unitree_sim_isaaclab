#!/bin/bash
# Pico VR 控制 bring-up（pipeline 双机器人，工作包 Pico-①/②）：
#   sim(HOST_MODE 双机器人) + deploy#1(--input-type zmq_manager ← Pico manager)
#   + 默认 deploy#2(keyboard 调试通道)，或 PIPELINE_DUAL_PICO=1 时：
#     deploy#2(--input-type zmq_manager ← Pico manager#2)
#
# 与 keyboard 版编排的差异：
#   - deploy#1 的输入从 stdin keyboard 换成 ZMQ(localhost:5556)，发车不再靠 ']'，
#     而是 manager 在操作者按 A+B+X+Y 进 PLANNER 模式时发 command(start=True)。
#     manager 起点用 --no_auto_pose：头显数据一到就自动进 POSE 全身跟随太危险，
#     改为操作者显式按键发车。
#   - manager 走 XROBO_TRANSPORT=udp：头显端 GameLink/定制 Unity 发送端直发本机
#     63901/udp（双 Pico 时 #2 用 63902/udp），不需要 XRoboToolkit PC Service。
#   - 双 Pico 的 manager ZMQ PUB 分别为 5556/5566，deploy#2 必须显式
#     --zmq-port 5566；G1_LOCAL_ROBOT_ID=2 只分 DDS/输出侧，不分 ZMQ 输入。
#   - 锁步注意：双 ack AND 门下 deploy#1/#2 仍必须并行启动（串行会自锁）；
#     ack 在 Init 后即持续回，(操作者未发车时机器人保持默认站姿，物理照常推进)。
#
# 操作者手册（头显侧）：
#   1. GameLink 目标 IP 指向本机（192.168.50.68），#1 端口 63901、#2 端口 63902。
#      机器人专用强刷版以 APK 内置 JSON 为真源并在每次启动时刷新 files 副本；旧版才
#      手工修改 files/XrLinkConfig.json。安装/覆盖规则及强制回读步骤见部署文档 §1.1。
#   2. 戴上头显、手柄唤醒，确认对应 pico_manager*.log 出现 body 数据（不再刷 waiting）。
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
DUAL_PICO="${PIPELINE_DUAL_PICO:-0}"
PICO_R1_UDP_PORT=63901
PICO_R2_UDP_PORT=63902
PICO_R1_ZMQ_PORT=5556
PICO_R2_ZMQ_PORT=5566

case "$DUAL_PICO" in
  0|1) ;;
  *)
    echo "ERROR: PIPELINE_DUAL_PICO 只接受 0 或 1（当前值: $DUAL_PICO）。"
    exit 2
    ;;
esac

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
elif ! command -v xset >/dev/null 2>&1; then
  echo "WARN: 没有 xset（x11-xserver-utils），无法预检 X 可用性。"
  echo "      若 sim 10s 内段错误且日志有 'GLFW initialization failed'，"
  echo "      用 PIPELINE_HEADLESS=1 重跑或装 xset 后再试。"
elif ! timeout 3 xset -display "$DISPLAY_TARGET" q >/dev/null 2>&1; then
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
SIM_PID=$!

# 早崩检测：kit 渲染/驱动层的崩溃集中在启动头几秒——与其让 bash 甩一行段错误,
# 不如当场把日志指纹和判读打出来。
sleep 10
if ! kill -0 "$SIM_PID" 2>/dev/null; then
  echo "ERROR: sim 启动 10s 内就退出了（多为渲染/驱动层崩溃）。host_dual.log 尾部："
  echo "----------------------------------------------------------------------"
  tail -20 "$LOG_DIR/host_dual.log"
  echo "----------------------------------------------------------------------"
  echo "判读："
  echo "  · 有 'GLFW initialization failed' → 拿不到 X：本地桌面跑 / PIPELINE_HEADLESS=1"
  echo "  · 无 GLFW 报错、栈在 librtx/carb → 驱动/Vulkan 层："
  echo "      vulkaninfo --summary | grep -i device   # 掉成 llvmpipe = 用户态驱动错配"
  echo "      nvidia-smi --query-gpu=name,driver_version --format=csv"
  exit 1
fi

# 双 ack AND 门：两个 deploy 必须并行启动（.trt 缓存只读共享，build 用 20s 错峰）。
echo "== start deploy#1 (rt/*, input=zmq_manager) =="
echo "" > "$LOG_DIR/dk_r1"
( cd "$DEPLOY_DIR" && tail -f "$LOG_DIR/dk_r1" | bash deploy.sh --disable-crc-check \
    --input-type zmq_manager --zmq-port "$PICO_R1_ZMQ_PORT" \
    isaac > "$LOG_DIR/deploy_r1.log" 2>&1 ) &
sleep 20

if [ "$DUAL_PICO" = "1" ]; then
  echo "== start deploy#2 (rt/r2/*, input=zmq_manager :$PICO_R2_ZMQ_PORT) =="
  # deploy.sh 启动前会 read 一次确认：只送一个空行接受默认 Y；该换行被确认提示
  # 消费后，运行期 stdin 只剩 EOF，没有任何键控字节。不能保留 dk_r2 键管道，
  # 否则脚本仍能替操作者发车。
  ( cd "$DEPLOY_DIR" && printf '\n' | env G1_LOCAL_ROBOT_ID=2 bash deploy.sh \
      --disable-crc-check --input-type zmq_manager --zmq-port "$PICO_R2_ZMQ_PORT" \
      isaac > "$LOG_DIR/deploy_r2.log" 2>&1 ) &
else
  echo "== start deploy#2 (rt/r2/*, input=keyboard) =="
  echo "" > "$LOG_DIR/dk_r2"
  ( cd "$DEPLOY_DIR" && tail -f "$LOG_DIR/dk_r2" | env G1_LOCAL_ROBOT_ID=2 bash deploy.sh \
      --disable-crc-check --input-type keyboard isaac > "$LOG_DIR/deploy_r2.log" 2>&1 ) &
fi

# manager 拿到第一帧头显数据后才 bind ZMQ PUB；deploy 的 SUB 是 connect 语义，
# command 有 1Hz keepalive 兜 slow-joiner，所以 manager/deploy 谁先启动都能接上。
: > "$LOG_DIR/pico_manager.log"
if [ "$DUAL_PICO" = "1" ]; then
  : > "$LOG_DIR/pico_manager_r2.log"
fi
if [ "$DUAL_PICO" = "1" ]; then
  echo "== start pico manager#1 (udp :$PICO_R1_UDP_PORT, PUB :$PICO_R1_ZMQ_PORT) =="
  # 双 Pico 模式显式钉死两个 UDP 端口，防调用环境残留 XROBO_UDP_PORT 污染隔离。
  ( cd "$GR00T_ROOT" && env XROBO_TRANSPORT=udp XROBO_UDP_PORT="$PICO_R1_UDP_PORT" \
      PYTHONUNBUFFERED=1 "$MGR_PY" gear_sonic/scripts/pico_manager_thread_server.py \
      --manager --no_auto_pose --port "$PICO_R1_ZMQ_PORT" \
      > "$LOG_DIR/pico_manager.log" 2>&1 ) &

  echo "== start pico manager#2 (udp :$PICO_R2_UDP_PORT, PUB :$PICO_R2_ZMQ_PORT) =="
  ( cd "$GR00T_ROOT" && env XROBO_TRANSPORT=udp XROBO_UDP_PORT="$PICO_R2_UDP_PORT" \
      PYTHONUNBUFFERED=1 "$MGR_PY" gear_sonic/scripts/pico_manager_thread_server.py \
      --manager --no_auto_pose --port "$PICO_R2_ZMQ_PORT" \
      > "$LOG_DIR/pico_manager_r2.log" 2>&1 ) &
else
  echo "== start pico manager (udp transport, PUB :$PICO_R1_ZMQ_PORT) =="
  # 单 Pico 保留原行为：UDP 默认 63901，也允许调用方沿用 XROBO_UDP_PORT 覆盖。
  ( cd "$GR00T_ROOT" && env XROBO_TRANSPORT=udp PYTHONUNBUFFERED=1 "$MGR_PY" \
      gear_sonic/scripts/pico_manager_thread_server.py --manager --no_auto_pose \
      --port "$PICO_R1_ZMQ_PORT" > "$LOG_DIR/pico_manager.log" 2>&1 ) &
fi

# 不等头显首帧/ZMQ bind，但至少确认两个 UDP receiver 已成功启动；否则依赖缺失、
# 端口占用等早退不能伪装成 BRINGUP_DONE。单 Pico允许外部覆盖端口，所以只认通用标记。
if [ "$DUAL_PICO" = "1" ]; then
  wait_for "UDP receiver listening on .*:$PICO_R1_UDP_PORT" "$LOG_DIR/pico_manager.log" \
    60 "pico manager#1 UDP :$PICO_R1_UDP_PORT" || exit 1
  wait_for "UDP receiver listening on .*:$PICO_R2_UDP_PORT" "$LOG_DIR/pico_manager_r2.log" \
    60 "pico manager#2 UDP :$PICO_R2_UDP_PORT" || exit 1
else
  wait_for "UDP receiver listening on" "$LOG_DIR/pico_manager.log" \
    60 "pico manager UDP receiver" || exit 1
fi

wait_for "Init Done" "$LOG_DIR/deploy_r1.log" 300 "deploy#1 Init Done" || exit 1
wait_for "Init Done" "$LOG_DIR/deploy_r2.log" 300 "deploy#2 Init Done" || exit 1
grep -m1 "DDS topics" "$LOG_DIR/deploy_r1.log" || true
grep -m1 "DDS topics" "$LOG_DIR/deploy_r2.log" || true

if [ "$DUAL_PICO" = "1" ]; then
  echo "== wait sim STARTUP HOLD (dual Pico: wait for operators) =="
else
  echo "== wait sim STARTUP HOLD, start channel #2 (keyboard) =="
fi
wait_for "STARTUP HOLD" "$LOG_DIR/host_dual.log" 300 "sim STARTUP HOLD" || exit 1
if [ "$DUAL_PICO" = "0" ]; then
  sleep 2
  printf ']' >> "$LOG_DIR/dk_r2"
  wait_for "CONTROL marker received" "$LOG_DIR/host_dual.log" 60 "channel#2 CONTROL" || exit 1
fi
# 帧率三件套之三：提优先级（sim + 双 deploy）
sudo -n renice -n -10 -p $(pgrep -f "sim_mai[n].py" | head -1) \
    $(pgrep -f "g1_deploy_onnx_re[f]" | tr '\n' ' ') 2>/dev/null || true
if [ "$DUAL_PICO" = "1" ]; then
  echo "BRINGUP_DONE (dual Pico: robot#1/#2 均等各自操作者 A+B+X+Y 发车)"
  echo "logs: $LOG_DIR/{host_dual,deploy_r1,deploy_r2,pico_manager,pico_manager_r2}.log"
else
  # 通道 #2 planner 预激活（行走靠往 dk_r2 里发 w/s/a/d）
  sleep 3; printf '\n' >> "$LOG_DIR/dk_r2"
  sleep 3; printf '2' >> "$LOG_DIR/dk_r2"
  echo "BRINGUP_DONE (robot#1 等操作者头显发车: A+B+X+Y)"
  echo "logs: $LOG_DIR/{host_dual,deploy_r1,deploy_r2,pico_manager}.log"
fi
