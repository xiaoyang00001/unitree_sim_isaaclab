# 双 SONIC 咖啡场景后续事项

当前 `f12abe2674c2f09a48653ff0b1528980952542d3` 已通过 Linux 双 deploy、Win130 Viewer、
60 秒 50 Hz 锁步和 F12 reset 验收。以下事项不阻塞本次通过结论，后续按条处理。

## 1. 补齐 KitchenRoom 法线贴图

- [ ] 从合法的 Lightwheel KitchenRoom 资产包补齐
  `Locomotion/KitchenRoom/Materials/Textures/3d66Model-7443619-files-8.jpg`。
- [ ] 在 Linux 和 Windows 启动日志中确认不再出现 `normalmap_texture` asset-not-found。
- [ ] 使用固定 Cafe 相机复拍，确认厨房表面法线细节恢复且机器人材质不受影响。

当前影响：KitchenRoom 个别表面的法线细节缺失；场景创建、关键碰撞、机器人和杯子均正常。

## 2. 重建当前 GPU 对应的 TensorRT engine

- [ ] 删除或归档从其他 GPU 型号生成的旧 `.trt` engine。
- [ ] 在实际部署 GPU 上分别重新生成 policy 和 encoder engine。
- [ ] 两套 deploy 并行启动后确认不再出现 `Using an engine plan file across different models of devices`。
- [ ] 重跑不少于 60 秒锁步，确认 `timeouts=0`、`sync_waits` 不增长和 50 Hz 不回退。

当前影响：TensorRT 给出可移植性警告；本次两套 deploy 实跑未出现错误、死锁或降频。

## 3. 为未来交互扩充 Cafe 碰撞代理

- [ ] 只有任务明确需要机器人接触墙、柜体、冰箱或咖啡机时，才增加对应简化碰撞代理。
- [ ] 每个新增代理使用实测 KitchenRoom 几何尺寸，并增加机器人站位、杯子承托和 F12 回归。
- [ ] 保持 KitchenRoom visual USD 不变，不恢复 298 个复杂背景碰撞和动态刚体。

当前影响：当前端咖啡/交接任务只需要地面和岛台，现有两个代理已经覆盖；机器人可与未建代理的
背景物体发生视觉穿透，因此不能直接把本配置扩展成厨房自由导航任务。

## 4. 增加自动化外观运行门禁

- [ ] 在 CI 或 Viewer smoke test 中校验参考外观 USD 的 SHA256：
  `01677e6ab1d321e72533dc42393500c17655584d7f272b328857ab5080ef7c98`。
- [ ] 运行时断言两台 PeerRobot 均完成 `white=26, dark=22, logo=1` 绑定。
- [ ] 保存固定相机的 Conveyor/Cafe 机器人 mask，并增加轮廓和分色回归比较。
- [ ] 明确禁止 Cafe Viewer launcher 设置 `ISAACLAB_PEER_ROBOT_MODE=visual_lod`。

当前影响：已有静态 factory/launcher 测试和 Win130 实跑截图，但运行时材质计数及图像比较尚未进入
自动化流水线。
