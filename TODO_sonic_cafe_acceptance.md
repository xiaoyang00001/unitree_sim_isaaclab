# 双 SONIC 咖啡场景后续事项

当前版本已通过 Linux 双 deploy、Win130 普通 articulation Viewer、60 秒 50 Hz 锁步和
F12 reset。正式 Win130 OpenXR/AR Viewer 仍被外部头显链路阻塞，因此总验收为“部分通过”。
第 1 项阻塞最终通过；其余项目为非阻塞后续事项。

## 1. 恢复 Pico 链路并完成 Win130 OpenXR/AR 实跑

- [ ] 让 Pico `192.168.1.190` 重新接入局域网，确认 Win130 ping 可达且 ARP 有对应记录。
- [ ] 在 NOLO XRLink 或 ALVR Dashboard 中明确看到 Pico client connected，不以虚拟 HMD 进程存在代替连接状态。
- [ ] 确认 SteamVR 头显与控制器均激活，不再处于仅有 `NOLO-HMD-1` standby 的状态。
- [ ] 从 Win130 交互桌面双击 `run_cafe_viewer_ar_131.local.bat`，不得通过 SSH 非交互会话启动。
- [ ] 确认 OpenXR 不再出现 `xrCreateInstance failed`，并到达 `XR anchor -> PeerRobot2`。
- [ ] 确认两台 articulation G1 完成 `white=26, dark=22, logo=1` 材质绑定。
- [ ] 保存 OpenXR mirror/headset 画面，确认只有两台 G1 和杯子，robot 2 anchor 正确。
- [ ] 在 AR 运行期间复查 ZMQ 约 50 Hz、F12 reset 和 reset 后场景恢复。

当前阻塞证据：Pico ping/ARP 不可达，ALVR 未安装，SteamVR/NOLO 仅提供 standby 虚拟 HMD；
Isaac OpenXR 在 `xrCreateInstance` 阶段退出。失败日志为 `/tmp/cafe-v61-ar-viewer-20260821.log`。

## 2. 补齐 KitchenRoom 法线贴图

- [ ] 从合法的 Lightwheel KitchenRoom 资产包补齐
  `Locomotion/KitchenRoom/Materials/Textures/3d66Model-7443619-files-8.jpg`。
- [ ] 在 Linux 和 Windows 启动日志中确认不再出现 `normalmap_texture` asset-not-found。
- [ ] 使用固定 Cafe 相机复拍，确认厨房表面法线细节恢复且机器人材质不受影响。

当前影响：KitchenRoom 个别表面的法线细节缺失；场景创建、关键碰撞、机器人和杯子均正常。

## 3. 重建当前 GPU 对应的 TensorRT engine

- [ ] 删除或归档从其他 GPU 型号生成的旧 `.trt` engine。
- [ ] 在实际部署 GPU 上分别重新生成 policy 和 encoder engine。
- [ ] 两套 deploy 并行启动后确认不再出现 `Using an engine plan file across different models of devices`。
- [ ] 重跑不少于 60 秒锁步，确认 `timeouts=0`、`sync_waits` 不增长和 50 Hz 不回退。

当前影响：TensorRT 给出可移植性警告；本次两套 deploy 实跑未出现错误、死锁或降频。

## 4. 为未来交互扩充 Cafe 碰撞代理

- [ ] 只有任务明确需要机器人接触墙、柜体、冰箱或咖啡机时，才增加对应简化碰撞代理。
- [ ] 每个新增代理使用实测 KitchenRoom 几何尺寸，并增加机器人站位、杯子承托和 F12 回归。
- [ ] 保持 KitchenRoom visual USD 不变，不恢复 298 个复杂背景碰撞和动态刚体。

当前影响：当前端咖啡/交接任务只需要地面和岛台，现有两个代理已经覆盖；机器人可与未建代理的
背景物体发生视觉穿透，因此不能直接把本配置扩展成厨房自由导航任务。

## 5. 增加自动化外观运行门禁

- [ ] 在 CI 或 Viewer smoke test 中校验参考外观 USD 的 SHA256：
  `01677e6ab1d321e72533dc42393500c17655584d7f272b328857ab5080ef7c98`。
- [ ] 运行时断言两台 PeerRobot 均完成 `white=26, dark=22, logo=1` 绑定。
- [ ] 保存固定相机的 Conveyor/Cafe 机器人 mask，并增加轮廓和分色回归比较。
- [ ] 明确禁止 Cafe Viewer launcher 设置 `ISAACLAB_PEER_ROBOT_MODE=visual_lod`。

当前影响：已有静态 factory/launcher 测试和 Win130 实跑截图，但运行时材质计数及图像比较尚未进入
自动化流水线。
