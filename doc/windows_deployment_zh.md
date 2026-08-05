# 在一台全新 Windows 机器上部署 unitree_sim_isaaclab + 跨机 DDS 闭环

> 本文是**从零部署到新机器**的完整任务书，写成可以直接交给执行者（人或 AI）照做的形式。
> 已经部署好的机器（如 win2）的日常速查、帧率账本与已否决的优化方向见 `CLAUDE.md` 的
> 「Windows 部署」章节，两者互补、不重复。
>
> 内容来自 2026-07-29/30 的实战，每个坑都写了现象与根因——**目的是让下一台机器不用重新踩**。
> 所有 Windows 专属的代码修复都已进 git 分支，执行者主要是「按顺序拉取正确的分支/环境」，
> 而不是重新写代码。

## 背景（你需要知道的上下文）

`unitree_sim_isaaclab` 是基于 Isaac Lab 的 Unitree G1 仿真工程。它发送/接收和真机完全相同的
DDS 话题（`rt/lowcmd`、`rt/lowstate` 等），所以可以和一个独立的控制器进程（`GR00T-WholeBodyControl`
的 `gear_sonic_deploy`）跨机联调，组成一个"仿真端 + 控制器端"的锁步闭环：

```
[这台新 Windows 机器]                    [已有的 GR00T controller 机器]
  Isaac Sim 仿真 (SONIC 43DoF 机器人)  <--DDS(跨机,同网段)-->  gear_sonic_deploy (C++ 控制器)
  发布 rt/lowstate, 订阅 rt/lowcmd                             订阅 rt/lowstate, 发布 rt/lowcmd
```

这台新机器只需要跑仿真端。控制器端已经在另一台机器上跑通过，不需要你重新部署，只需要确认
它那边的 `GR00T-WholeBodyControl` 仓库也在含 Windows 兼容修复的分支上（见阶段 7）。

**这是一个全新的 Windows 机器，之前没有装过这套东西。** 已经有另外两台机器成功部署过（一台
Linux、一台 Windows），下面每一步都是从那两次实战里提炼出来的，包括踩过的坑和现成的解决方案——
**目的是让你不用重新踩坑一遍**。所有 Windows 专属的修复都已经提交进 git 仓库的特定分支，
你要做的主要是"按顺序拉取正确的分支/环境"，而不是重新写代码。

## 开始前必须向用户确认的信息（不要自己瞎猜，问清楚再动手）

1. **这台机器与 GR00T controller 机器是否在同一局域网？能否接有线网线到同一网段？**
   —— 强烈建议接有线。已有实测：同一网段下 WiFi 长跑会有明显尾部延迟抖动（p90 远高于中位数），
   而有线连接下本地测试能跑到 200Hz+ 量级的锁步握手。如果只能用 WiFi，先跑通功能，性能优化
   单独再谈。
2. **controller 机器的 IP 和网卡名是什么？**（要传给 `deploy.sh isaac:<iface>` 和
   `run_win.bat --dds-interface <本机IP>`）
3. **是否需要 AR/XR（头显）支持？** 如果只是要非 AR 的仿真闭环，跳过阶段 9；如果需要，阶段 9
   的 extscache 三件套要装。
4. **`assets/`（场景资产，约 1.65GB/423 文件）和 GR00T 侧的机器人网格/URDF 资产从哪里获取？**
   —— 如果这台机器和某台已部署好的机器在同一局域网，最快是用 `tar` + `scp` 直接传（比走代理/
   HuggingFace 快且稳，之前实测本机下载只有 1MB/s，局域网传输快得多）；否则要走
   `fetch_assets.sh`（HuggingFace + git-lfs），需要确认网络能否访问 HuggingFace。
5. **Isaac Sim / IsaacLab 用哪个版本组合？** 本文默认走 **Isaac Sim 5.1.0.0 +
   `xiaoyang00001/IsaacLab`**，这是 win2 上**已经跑通验证过**的组合，新机器建议照抄。
   ⚠️ 两个已知环境分别用过该 fork 的 `0703` 与 `07241` 分支，**不要不问就套用某一个**，
   向用户确认当前该用哪个（或是否已有更新分支）。
   ⚠️ 另外注意：主开发机上还有一套 **Isaac Sim 6.0 + IsaacLab 2.3.2** 的第二环境
   （见 `doc/isaacsim6_migration_zh.md`），那是**本机的迁移实验环境，不是新机器该装的**——
   6.0 系列在 Windows 上从未验证过，且版本必须钉死 6.0.0.0（6.0.0.1 起砍掉了 IsaacLab 2.3.2
   依赖的 PhysX API）。除非用户明确要求，否则新机器一律走 5.1.0.0。

## 阶段 0：系统前置设置

在装任何东西之前，先做这两件事，否则后面会踩到看起来毫不相关的坑：

1. **打开 Windows 长路径支持**（否则后面装 XR 扩展缓存会在 260 字符路径处失败，且失败后
   会留下**半装残渣**，导致完全不相关的仿真任务在 `import h5py` 时报
   `DLL load failed while importing _errors`——如果遇到这个报错，先用裸 `python -c "import h5py"`
   确诊是不是这个问题，如果能正常 import 说明环境本身没坏，是 kit 运行时被污染了，
   把对应的 extscache 目录清空重装即可）：
   ```powershell
   # 管理员 PowerShell
   New-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem" `
     -Name "LongPathsEnabled" -Value 1 -PropertyType DWORD -Force
   ```
   改完建议重启一次。

2. **把电源计划切到高性能**（不是必须，但省得后面测帧率时怀疑到降频头上）：
   ```powershell
   powercfg /setactive SCHEME_MIN
   ```

## 阶段 1：conda 环境

```powershell
# 装 miniconda（如果还没有）
conda create -n env_isaaclab python=3.11 -y
conda activate env_isaaclab
```

⚠️ Isaac Sim 5.1 系列锁死 Python 3.11，不要用别的版本（后面 cyclonedds 只有 0.10.x 系列
没有 Windows cp311 wheel 这件事和这个版本锁死是分开的两个约束，都要满足）。

## 阶段 2：Isaac Sim + Isaac Lab

```powershell
pip install isaacsim==5.1.0.0 --extra-index-url https://pypi.nvidia.com
# torch 2.7.0+cu128 等依赖会随 isaacsim 一起装好

# 必须钉死：h5py 3.16 自带 HDF5 2.0，与 Isaac Sim 5.1 的 HDF5 1.14.6 DLL 冲突
pip install --no-deps --force-reinstall h5py==3.15.1
```

若启动时报“无法定位程序输入点 `H5Tdecode` 于 `hdf5_cpp.dll`”，说明 h5py 被升级到了
3.16；重新执行上面的降级命令即可。

装完先验证能不能起独立的 Isaac Sim（不涉及本工程）：
```powershell
python -c "import isaacsim, h5py; print('Isaac Sim ok'); print(h5py.__version__, h5py.version.hdf5_version)"
```

IsaacLab fork：
```powershell
git clone https://github.com/xiaoyang00001/IsaacLab.git D:\Isaac\xiaoyangIsaacLab
cd D:\Isaac\xiaoyangIsaacLab
git checkout <向用户确认的分支名>   # 已知参考值 0703 / 07241，务必先向用户核实
```
装成可编辑安装（具体安装脚本参照该仓库自己的 `install` 脚本，不同分支可能不同，如果不确定
就问用户或读该仓库的 README）。

## 阶段 3：cyclonedds —— 全流程里最容易踩坑的一步

**核心问题**：cyclonedds 的 PyPI 包只有 `11.0.1` 才有 Windows cp311 wheel（0.10.x 系列完全
没有预编译的 Windows wheel）。但**绝对不能用 11.0.1**——它对应 CycloneDDS C 库 0.11，
握手时会广播 XTypes TypeObject；而 GR00T controller 那边的 C++ 链接的是 CycloneDDS 0.10.2，
解析不了这个类型对象会直接 **segfault**（报 `ddsi_xt_type_init_impl with invalid type object`，
崩溃发生在 C++ 侧，看起来完全和 Windows/Python 无关，很容易查错方向）。两侧都没有运行时开关
可以关闭 type discovery。

正确做法是从源码编译 0.10.5：

1. **装 MSVC Build Tools**（cyclonedds 的 Python 绑定要编译 C 扩展）：
   ```powershell
   vs_BuildTools.exe --quiet --wait --norestart --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended
   ```
   ⚠️ **千万不要加 `--nocache` 参数**——它会阻止 VS Installer 的自更新组件落地，
   引导器会报 `Unexpected Installer Version` / `EXITCODE=5008`。如果看到日志里有
   `Certificate is invalid` 字样，那是假线索（BITS 下载失败后 WinInet 重试会自己成功），
   不用去折腾证书或代理。这一步比较慢，建议用 `schtasks` 后台跑，避免 ssh/远程会话断开
   把安装打断。

2. **源码编译装 cyclonedds C 库 + Python 绑定**：
   ```powershell
   pip install cyclonedds==0.10.5
   ```
   这一步会现场用刚装好的 MSVC 编译，比预编译 wheel 慢很多，属于正常现象。

3. **装完必须手改一个文件**（每次重装 cyclonedds 都要重新改一次，这是已知的、无法绕过的
   小烦人问题）：
   ```
   <conda env>\Lib\site-packages\cyclonedds\__library__.py
   ```
   里面生成的路径类似 `library_path = 'C:\Users\...\ddsc.dll'`——这是一个普通 Python
   字符串，其中的 `\U` 会被解释成 Unicode 转义序列，导致 `import cyclonedds` 直接
   `SyntaxError`。把反斜杠全部换成正斜杠即可（Windows 下正斜杠路径同样能用）。

4. 装 `unitree_sdk2py` 时要用 `--no-deps` 绕开它钉死的 `cyclonedds==0.10.2`（我们已经装了
   兼容的 0.10.5，不需要它再去解析/覆盖依赖）：
   ```powershell
   pip install --no-deps -e <unitree_sdk2py 目录路径>
   ```

**验证方法（关键！不要只测 Python↔Python）**：cyclonedds 版本兼容性必须测到真实的 C++
对端才有意义——Python 和 Python 之间即使版本不对也能顺利通信，测不出和 C++ 的兼容性问题。
最终验证要等到阶段 8 和真实的 GR00T controller 联调时才算数。

## 阶段 4：unitree_sdk2py（已经修好 Windows 兼容问题，直接拉取，不要重新改代码）

之前已经在这上面做过两处 Windows 专属修复：
- 按 IPv4 地址选网卡（Windows 上原来的按名字 `name=` 选网卡完全选不中）
- 新增 `UNITREE_DDS_PEERS` 单播模式（可选，缓解 WiFi 对组播的限速；默认不需要开启）

这些修复已经提交进 `GR00T-WholeBodyControl` 仓库（`https://gitlab.nolovr.com:22043/xiaoyang/GR00T-WholeBodyControl.git`）
的 `sonic-windows-cross-machine-dds` 分支（基于 `master`）。获取方式：

```powershell
git clone https://gitlab.nolovr.com:22043/xiaoyang/GR00T-WholeBodyControl.git D:\Isaac\GR00T-WholeBodyControl
cd D:\Isaac\GR00T-WholeBodyControl
git checkout sonic-windows-cross-machine-dds
```

`unitree_sdk2py` 实际路径是这个仓库里的 `external_dependencies/unitree_sdk2_python`。
可以整个仓库都 clone 下来（体积较大但最省心，也不用担心以后又漏了什么改动），也可以只把
`external_dependencies/unitree_sdk2_python` 这一个子目录单独拷贝到别处用——如果单独拷贝，
**注意它会变成一份普通文件快照，不是 git 仓库**，以后有新修复需要重新拷贝一遍来同步，
不能直接在那个目录里 `git pull`。

**不要凭空重新写按 IP 选网卡的逻辑或者重新踩这两个坑**——这些已经在上面这个分支里改好了，
拉下来就能用。

## 阶段 5：unitree_sim_isaaclab 工程本体

```powershell
git clone https://github.com/xiaoyang00001/unitree_sim_isaaclab.git D:\Isaac\unitree_sim_isaaclab
cd D:\Isaac\unitree_sim_isaaclab
git checkout perf/win-fps-native
```

这个分支已经包含全部 Windows 部署所需的修复，具体是这 6 个提交（从旧到新）：
- `os.getuid()` 换成跨平台实现（POSIX 用 uid，Windows 用登录名）
- URDF 合成失败且旧产物缺失时打印真因（不再哑死）
- 精简 experience 下恢复 viewport 的 FPS HUD
- **`sim_main` 的四处 Windows 适配**（含 `timeBeginPeriod(1)` 定时器分辨率，这是后面帧率能
  到 50Hz 的关键一环）
- **DDS 两处 CRC 优化**（纯 Python CRC 在 Windows 上很慢，会吃掉大量 GIL 时间；已经改成
  可以跳过校验，需要 controller 端配合传 `--disable-crc-check`）
- **`run_win.bat` / `run_win_ar.bat` 两个启动脚本**——**日常启动一律用这两个脚本，
  不要自己拼 `python sim_main.py ...` 命令**，脚本里封装了下面这些必需但容易漏掉的环境变量：
  - `chcp 65001` + `PYTHONUTF8=1`：中文 Windows 控制台默认 GBK 编码，工程日志里的中文/emoji
    会让 `print()` 直接抛 `UnicodeEncodeError` 崩掉整个进程（不是乱码，是进程死掉）。
  - `GR00T_WBC_ROOT`：SONIC 任务在**模块级**就要合成 URDF，这个环境变量不设的话
    `import tasks` 这一步就会死掉，跟你后面具体跑哪个任务无关。必须指向阶段 4 里
    clone 下来的 `GR00T-WholeBodyControl` 仓库根目录。

`assets/` 目录（场景资产）：本工程用符号链接指向外部盘，需要单独准备，参照开头"需要确认的
信息"第 4 条获取，这部分内容和代码仓库分开维护，不在 git 里。

## 阶段 6：GR00T 侧数据资产（仿真端也需要一部分）

SONIC 43DoF 的 URDF 是仿真启动时**运行时动态合成**的，只需要 `GR00T-WholeBodyControl` 仓库里
这三个路径（不需要整个 `gear_sonic/data`）：
```
gear_sonic/data/robots/g1/
gear_sonic/data/assets/robot_description/urdf/g1/
gear_sonic/data/robot_model/model_data/g1/meshes/
```
如果阶段 4 已经把整个仓库 clone 下来，这些路径本来就在里面，不需要额外操作。
⚠️ 如果是从别的机器单独打包这三个路径传过来：最后一个 `meshes` 目录在源头机器上可能是
符号链接，打包时要用 `tar -h`（跟随符号链接）而不是普通 `tar`，否则传过去的是一个空链接。

## 阶段 7：确认 controller 端（另一台机器，你可能不需要动，但要确认状态）

这台新机器起来之后要跟一个**已经在别处跑通过**的 `gear_sonic_deploy` 控制器联调。
需要向操作 controller 那台机器的人确认：

1. 它的 `GR00T-WholeBodyControl` 仓库是否已经切到 `sonic-windows-cross-machine-dds` 分支
   （或者已经合并进它们用的主线分支）——这个分支里除了阶段 4 提到的 SDK 修复，还有两处
   `deploy.sh` 的改动：
   - 新增 `isaac:<iface>` 模式（跨机场景专用）：`./deploy.sh --disable-crc-check isaac:<网卡名>`。
     **不要传裸网卡名或者只传 `isaac`**——裸网卡名会被脚本判断成 `real`（真机模式）而
     静默丢掉 Isaac 相关的参数，`isaac:<iface>` 这个前缀形式才是跨机模式该用的写法。
   - `init-duration` 从默认 3.0 秒降到 0.1 秒（否则 `Init Done` 要等 6 分钟）。
2. 启动时要带 `--disable-crc-check`，因为仿真端默认会跳过 LowState 的 CRC 计算
   （Windows 上纯 Python 计算 CRC 太慢），如果对端不配合会导致每一帧都被当成校验失败丢弃。

## 阶段 8：冒烟测试

先起仿真端（这台新机器）：
```powershell
D:\Isaac\unitree_sim_isaaclab\run_win.bat --task Isaac-G1-29DoF-Sonic --robot_type g129 ^
    --action_source sonic_dds --device cpu --dds-interface <这台机器自己的IP>
```
等日志打出：
```
[sonic_dds] Body mapping 29/29, Dex3 mapping 14/14
```
再在 controller 机器上起：
```bash
./deploy.sh --disable-crc-check isaac:<controller机器的网卡名>
```
等 `Init Done` 出现（应该是几秒钟量级，不是几分钟——如果等了很久没出现，去确认 controller
端是否已经切到阶段 7 提到的分支）。

**验证锁步是否正常**：仿真端日志会周期性打印 `[Performance] A:...ms, E:...ms, S:...ms, T:...ms,
stepped=..., physics_steps=..., sync_waits=...` 这样的行（打印频率可以用 `--profile_interval N`
调节，数值越小打印越密）。正常工作时 `physics_steps` 应该持续增长、`stepped=1` 居多；
如果 `A` 长期卡在接近 250ms、`physics_steps` 不再增长，说明两端锁步的 tick/epoch 对不上了
（常见原因是只重启了一侧），把仿真端和 controller 端**同时**重启一遍即可。

非 AR、headless 模式下，`--device cpu` 是推荐配置（SONIC 场景下把 GPU 让给外部推理进程）。
目标是 `T` 中位数在 20ms 左右（对应 50Hz）。如果启动后长期卡在明显更低的帧率，先确认：
- controller 端和仿真端是否**同时**重启过（见上）
- 有没有残留的旧 `deploy` 进程仍在跑，同时发 ack 会污染整个测量
- 有没有 VNC/远程桌面客户端连着（历史上发现过：VNC **客户端**连接哪怕只占 5% CPU，
  也会因为阻塞画面呈现（而不是抢 CPU）把帧率从正常水平拖到很低——这个不能靠看 CPU 占用
  排查出来，得靠"断开 VNC 试试看"来验证）

## 阶段 9（可选）：AR/XR 支持

如果这台机器需要接头显做 AR：

1. 装三个 extscache 包（阶段 0 的长路径设置必须先做好，否则这里会解包失败留下半装残渣）：
   ```powershell
   pip install isaacsim-extscache-kit isaacsim-extscache-kit-sdk isaacsim-extscache-physics
   ```
   下载可能很慢/不稳定（这几个包体积较大，NVIDIA 源本身也不允许 pip 缓存，失败了会整个重下），
   如果这台机器和其他已装好的机器在同一局域网，更快的办法是在另一台机器上先下载好，
   `scp` 过来后本地 `pip install` 本地包。
2. AR 必须在**物理桌面会话**里启动（双击 `run_win_ar.bat`），不能通过 SSH 远程启动——
   OpenXR runtime（SteamVR）只在交互式桌面会话里可用，SSH 起的进程连不上它。
3. 启动前需要先把头显链路（NOLO Link 或对应厂商客户端）连上，这会自动拉起 SteamVR。

## 收尾自查清单

- [ ] `python -c "import cyclonedds; print(cyclonedds.__version__)"` 输出 `0.10.5`
- [ ] `python -c "import isaacsim"` 不报错
- [ ] `GR00T_WBC_ROOT` 环境变量指向正确路径（或已经在 `run_win.bat` 里设好默认值）
- [ ] 非 AR headless 冒烟：仿真端 `Body mapping 29/29, Dex3 mapping 14/14`，controller 端
      `Init Done`，`[Performance]` 行里 `physics_steps` 持续增长
- [ ] 确认电源计划已是高性能、长路径已开启
- [ ] 如果需要 AR：SteamVR/NOLO Link 已连接，`run_win_ar.bat` 是双击启动的
