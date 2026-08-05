当前机器信息我已核对：

- Ubuntu host：`192.168.1.131`
- 仿真工程：`/home/nolovr/Documents/unitree_sim_isaaclab`
- GR00T：`/home/nolovr/GR00T-WholeBodyControl`
- Pico#1：`192.168.1.190:5555`，已正确配置到 `192.168.1.131:63901`
- Pico#2：当前还未出现在 ADB 中，需要先连接并配置为 `63902`
- 当前旧链仍在运行，不能直接重复启动

端口关系必须保持：

| 控制链 | Pico UDP | manager ZMQ PUB | deploy 输入 | DDS |
|---|---:|---:|---|---|
| robot_1 | 63901 | 5556 | 5556 | `rt/*` |
| robot_2 | 63902 | 5566 | 5566 | `rt/r2/*` |

## 一、先配置 Pico#2

先让第二台 Pico 开启无线 ADB，然后执行：

```bash
PICO2_ADB="<Pico#2的IP>:5555"
PICO2_CONFIG="/sdcard/Android/data/com.Nolo.CloudVR/files/XrLinkConfig.json"
PICO2_WORK="$(mktemp -d)"

adb connect "$PICO2_ADB"
adb -s "$PICO2_ADB" pull "$PICO2_CONFIG" "$PICO2_WORK/original.json"

jq '
  .wholeBodyTracking.serverHost = "192.168.1.131"
  | .wholeBodyTracking.serverPort = 63902
  | .wholeBodyTracking.sendControllers = true
' "$PICO2_WORK/original.json" > "$PICO2_WORK/updated.json"

adb -s "$PICO2_ADB" push "$PICO2_WORK/updated.json" "$PICO2_CONFIG"
adb -s "$PICO2_ADB" shell am force-stop com.Nolo.CloudVR
adb -s "$PICO2_ADB" shell monkey -p com.Nolo.CloudVR \
  -c android.intent.category.LAUNCHER 1

adb -s "$PICO2_ADB" shell cat "$PICO2_CONFIG" | jq '.wholeBodyTracking'
```

最后必须确认输出包含：

```json
{
  "serverHost": "192.168.1.131",
  "serverPort": 63902,
  "sendControllers": true
}
```

如果没装 GameLink：

```bash
adb -s "$PICO2_ADB" install -r /path/to/GameLink.apk
```

若启用了 UFW：

```bash
sudo ufw allow 63901/udp
sudo ufw allow 63902/udp
sudo ufw status
```

## 二、停止当前旧链

优先在原终端逐个 `Ctrl+C`。如果原终端找不到，执行：

```bash
pkill -INT -f 'sim_mai[n].py'
pkill -INT -f 'pico_manager_thread_serve[r].py'
pkill -INT -f 'g1_deploy_onnx_re[f]'
pkill -f 'tail -f.*dk_r[12]'

sleep 5
pgrep -fa 'sim_main.py|g1_deploy_onnx_ref|pico_manager_thread_server.py'
```

最后一条应无输出。若仍有残留，再针对残留执行：

```bash
pkill -9 -f 'g1_deploy_onnx_re[f]|pico_manager_thread_serve[r].py'
```

## 三、启动五个终端

### 终端 1：双机器人 host sim

```bash
cd /home/nolovr/Documents/unitree_sim_isaaclab

env DISPLAY=:1 \
  GR00T_WBC_ROOT=/home/nolovr/GR00T-WholeBodyControl \
  UNITREE_DDS_DOMAIN=1 \
  UNITREE_DDS_INTERFACE=lo \
  ISAACLAB_LOCAL_ROBOT_ID=1 \
  ISAACLAB_HOST_BOTH_ROBOTS=1 \
  ISAACLAB_SCENE_SYNC_PEER_IP=127.0.0.1 \
  UNITREE_SKIP_LOWSTATE_CRC=1 \
  UNITREE_LOWCMD_CRC_SAMPLE_INTERVAL=50 \
  /home/nolovr/miniconda3/envs/env_isaaclab/bin/python \
  sim_main.py \
  --task Isaac-G1-29DoF-Sonic-Conveyor \
  --robot_type g129 \
  --action_source sonic_dds \
  --device cpu \
  --hide_ui \
  --stats_interval 10
```

如果需要把场景同步给远端 viewer，把 `127.0.0.1` 换成 viewer IP。纯 SSH 无桌面时，把 `--hide_ui` 换成 `--no_render`。

### 终端 2：deploy#1

```bash
cd /home/nolovr/GR00T-WholeBodyControl/gear_sonic_deploy

G1_LOCAL_ROBOT_ID=1 bash deploy.sh \
  --disable-crc-check \
  --input-type zmq_manager \
  --zmq-port 5556 \
  isaac
```

出现：

```text
Proceed with deployment? [Y/n]:
```

直接按一次回车。

启动后等待约 20 秒，但不要等它完整 `Init Done` 才启动 deploy#2，否则双 ack 门可能自锁。

### 终端 3：deploy#2

```bash
cd /home/nolovr/GR00T-WholeBodyControl/gear_sonic_deploy

G1_LOCAL_ROBOT_ID=2 bash deploy.sh \
  --disable-crc-check \
  --input-type zmq_manager \
  --zmq-port 5566 \
  isaac
```

确认提示同样直接按回车。

检查启动输出应明确包含：

```text
--zmq-port 5566
```

并且后续 DDS 日志属于 `rt/r2/*`。

两个 deploy 终端确认以后，不要再输入 `]`、回车、`2` 或 WASD；双 Pico 发车全部由头显完成。

### 终端 4：manager#1

```bash
cd /home/nolovr/GR00T-WholeBodyControl

XROBO_TRANSPORT=udp \
XROBO_UDP_PORT=63901 \
PYTHONUNBUFFERED=1 \
.venv_teleop/bin/python \
gear_sonic/scripts/pico_manager_thread_server.py \
--manager \
--no_auto_pose \
--port 5556
```

首先应看到：

```text
XRoboToolkit UDP receiver listening on 0.0.0.0:63901
```

Pico#1 数据到达后应看到：

```text
[Manager] ZMQ socket bound to port 5556
```

### 终端 5：manager#2

```bash
cd /home/nolovr/GR00T-WholeBodyControl

XROBO_TRANSPORT=udp \
XROBO_UDP_PORT=63902 \
PYTHONUNBUFFERED=1 \
.venv_teleop/bin/python \
gear_sonic/scripts/pico_manager_thread_server.py \
--manager \
--no_auto_pose \
--port 5566
```

首先应看到：

```text
XRoboToolkit UDP receiver listening on 0.0.0.0:63902
```

Pico#2 数据到达后应看到：

```text
[Manager] ZMQ socket bound to port 5566
```

## 四、按隔离顺序发车

1. 先关闭 Pico#1 的 GameLink，只启动并戴上 Pico#2。
2. manager#2 应停止打印 waiting，并 bind 5566；manager#1 必须仍在 waiting。
3. 操作者#2 站直，按 `A+B+X+Y`。
4. sim 终端应出现：

```text
[sonic_dds:r2] CONTROL marker received
```

此时只能 robot_2 动。

5. 再启动 Pico#1 GameLink，等待 manager#1 bind 5556。
6. 操作者#1 站直后按 `A+B+X+Y`，robot_1 才应进入控制。
7. 双人同时操作，确认 `physics_steps` 持续增长、`sync_waits` 不持续增长。

可辅助检查端口：

```bash
ss -lunp | rg ':(63901|63902)\b'
ss -ltnp | rg ':(5556|5566)\b'
```

## 五、急停与退出

任一操作者再次按 `A+B+X+Y`，只应停止自己的机器人并退出自己的 manager，另一路继续工作。

完整结束时：

1. 两位操作者分别急停。
2. 两个 deploy 终端 `Ctrl+C`。
3. sim 终端 `Ctrl+C`。

急停后如果要重新发车，需要重启对应 manager；若该路进入 planner 原地踉跄的退化态，则同时重启对应 deploy。

完整手册在 [pipeline_pico_vr_deployment_zh.md](/home/nolovr/Documents/unitree_sim_isaaclab/doc/pipeline_pico_vr_deployment_zh.md:137)。
