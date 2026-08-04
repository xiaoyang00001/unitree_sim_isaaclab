# Pipeline Windows Viewer 同机复刻部署手册

本文用于把一台已经验证可用的 Windows Pipeline Viewer（参考机）复刻到另一台
Windows 电脑（目标机），并让两台 Viewer 同时连接同一台 Ubuntu Pipeline Host。

本文记录的实战拓扑是：

| 角色 | 地址 | 用途 |
|---|---|---|
| Ubuntu Host | `192.168.1.131` | 运行物理仿真、双机器人控制和场景同步 PUB，监听 TCP `15555` |
| Windows 参考 Viewer | `192.168.1.130` | 已验证环境，作为软件、工程和资源基线 |
| Windows 新 Viewer | `192.168.1.129` | 本次部署目标，只镜像 Host 场景，可运行普通 Viewer 和 AR Viewer |

如果用于别的现场，必须替换上表中的 IP、Windows 用户名和磁盘路径，不要直接照抄。

相关文档：

- [Pipeline 新机器部署说明](pipeline_new_machine_deploy_zh.md)
- [双机器人 Host/Viewer 架构说明](pipeline_dual_robot_host_viewer_zh.md)
- [Windows 基础环境部署](windows_deployment_zh.md)
- [Pico VR 部署说明](pipeline_pico_vr_deployment_zh.md)

## 1. 部署原则

新机需要与参考机保持以下内容一致：

- Windows 版本、NVIDIA 驱动大版本和 GPU 可用性；
- Python、Isaac Sim、PyTorch、CycloneDDS 和工程依赖版本；
- 两个 Git 工程的提交、分支和工作树状态；
- 场景资源、GR00T 数据、SteamVR/OpenXR 和 XRLink；
- Viewer 所连接的 Host IP，以及新机自己的 DDS 网卡 IP。

Viewer 不运行 GR00T 推理，也不参与 Host 的 DDS 锁步控制。它只通过出站 TCP
连接 Host 的 `15555` 端口并镜像场景，因此通常不需要开放 Pipeline 入站端口。
但是，任务导入和场景构建仍会访问 GR00T/机器人资源，相关数据不能省略。

复刻过程中不要复制以下内容：

- SSH 私钥、Windows 登录密码、Steam 登录态；
- Isaac Sim 的机器相关日志和缓存；
- 参考机的固定 IP 配置；
- 未经核对的 `known_hosts` 记录。

## 2. 本次验证过的软件基线

129 最终验证通过的基线如下：

| 项目 | 版本或状态 |
|---|---|
| Windows | Windows 10 Pro，`10.0.19045` |
| CPU / GPU | Intel i7-14790F / NVIDIA GeForce RTX 5080 16 GB |
| NVIDIA 驱动 | `581.80` |
| Python | `3.11.15` |
| Isaac Sim | `5.1.0.0` |
| PyTorch | `2.7.0+cu128`，CUDA 可用 |
| CycloneDDS Python | `0.10.5` |
| h5py | `3.16.0` |
| Git for Windows | `2.33.0.windows.2` |
| Visual Studio Code | `1.113.0`，x64 User Setup |
| SteamVR | App `250820`，build ID `14523237` |
| XRLink | `3.0.1` |

工程基线：

| 工程 | 分支 | 提交 |
|---|---|---|
| `unitree_sim_isaaclab` | `feat/pipeline-pico-vr-control` | `1968e88519dc5c69fe4fd8981ad412979f7ae8ed` |
| `xiaoyangIsaacLab` | `feat/sonic-pickplace-ref-20260724` | `239c68c48f8cf5acba4eb7b72bcc067cb1a7e9db` |

> 注意：参考机 `xiaoyangIsaacLab` 中有一个名为 `2.7.0+cu128` 的未跟踪文件。
> 精确复刻时它也会被传到新机。除非确认其来源和用途，否则不要仅为了让
> `git status` 变干净而删除。

## 3. 在目标 Windows 上准备 SSH

以下命令都在目标机的“管理员 PowerShell”中执行。

### 3.1 安装并启用 OpenSSH Server

先检查组件：

```powershell
Get-WindowsCapability -Online |
    Where-Object Name -Like 'OpenSSH.Server*'
```

如果状态不是 `Installed`：

```powershell
Add-WindowsCapability -Online -Name OpenSSH.Server~~~~0.0.1.0
```

启动服务并设置为开机启动：

```powershell
Start-Service sshd
Set-Service -Name sshd -StartupType Automatic
Get-Service sshd
```

服务名是 **`sshd`**，不是 `ssh`。如果执行 `Start-Service ssh`，会出现“找不到
任何服务名称为 ssh 的服务”，这不代表 OpenSSH Server 安装失败。

检查系统创建的防火墙规则：

```powershell
Get-NetFirewallRule -Name OpenSSH-Server-In-TCP -ErrorAction SilentlyContinue |
    Format-Table Name, Enabled, Profile, Direction, Action
```

如果规则不存在，再创建仅允许当前可信局域网访问的 TCP 22 入站规则。现场有明确
网段时应设置 `-RemoteAddress`，不要把 SSH 暴露到不可信网络。

### 3.2 配置 SSH 公钥认证

如果使用的是管理员组账号，Windows OpenSSH 默认读取：

```text
C:\ProgramData\ssh\administrators_authorized_keys
```

把控制端的**公钥**追加到这个文件，每把密钥占一行；不要把私钥复制到 Windows。
然后收紧 ACL：

```powershell
icacls C:\ProgramData\ssh\administrators_authorized_keys /inheritance:r
icacls C:\ProgramData\ssh\administrators_authorized_keys /grant 'SYSTEM:F'
icacls C:\ProgramData\ssh\administrators_authorized_keys /grant '*S-1-5-32-544:F'
Restart-Service sshd
```

`S-1-5-32-544` 是本机 Administrators 组的 SID，能避免中文和英文系统组名不同导致
ACL 命令失效。

### 3.3 核对目标机身份

在目标机本地控制台查看 Ed25519 Host Key 指纹：

```powershell
ssh-keygen -lf C:\ProgramData\ssh\ssh_host_ed25519_key.pub
```

再从控制端扫描，并人工比对两边的 SHA256 指纹：

```bash
ssh-keyscan -t ed25519 192.168.1.129 2>/dev/null | ssh-keygen -lf -
```

只有指纹一致后才能接受新 Host Key。若目标机重装过系统，SSH 报 Host Key 变化是
正常现象，但仍须在本机控制台重新核对后再更新 `known_hosts`，不能直接跳过校验。

本次 129 经本机控制台确认的指纹为：

```text
SHA256:5+LHcnoUICNG7qRScitNXork680BhODX9ghTotw+z8A
```

## 4. 部署前盘点

先分别登录参考机和目标机，记录基线，不要一上来就覆盖文件。

### 4.1 Windows、硬件和磁盘

```powershell
$env:COMPUTERNAME
Get-ComputerInfo | Select-Object WindowsProductName, WindowsVersion, OsBuildNumber
Get-CimInstance Win32_Processor | Select-Object Name
Get-CimInstance Win32_VideoController |
    Select-Object Name, DriverVersion, AdapterRAM
Get-PSDrive -PSProvider FileSystem |
    Select-Object Name, Used, Free
nvidia-smi
```

### 4.2 工程提交和工作树

```powershell
git -C D:\Isaac\unitree_sim_isaaclab branch --show-current
git -C D:\Isaac\unitree_sim_isaaclab rev-parse HEAD
git -C D:\Isaac\unitree_sim_isaaclab status --short

git -C D:\Isaac\xiaoyangIsaacLab branch --show-current
git -C D:\Isaac\xiaoyangIsaacLab rev-parse HEAD
git -C D:\Isaac\xiaoyangIsaacLab status --short
```

`git status` 必须逐项记录。精确复刻包含参考机已有的受控修改和未跟踪文件；不能只
比较提交号，也不能误把参考机的现场修改清理掉。

### 4.3 目录文件数和字节数

大目录不适合逐文件目测。参考机和目标机均可用下面的函数统计：

```powershell
function Measure-Tree([string]$Path) {
    $m = Get-ChildItem -LiteralPath $Path -File -Recurse -Force |
        Measure-Object -Property Length -Sum
    [pscustomobject]@{
        Path  = $Path
        Files = $m.Count
        Bytes = $m.Sum
    }
}

Measure-Tree D:\reboot\assets_unzip\assets
Measure-Tree D:\reboot\GR00T-WholeBodyControl\gear_sonic\data
```

本次参考机统计值：

| 目录 | 文件数 | 字节数 |
|---|---:|---:|
| `D:\reboot\assets_unzip\assets` | 423 | 1,773,909,785 |
| `D:\reboot\GR00T-WholeBodyControl\gear_sonic\data` | 733 | 1,264,561,387 |

## 5. 目标机基础设置

### 5.1 开启 Windows 长路径

Isaac Sim 和 Python 包目录很深，必须开启长路径：

```powershell
New-ItemProperty `
    -Path 'HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem' `
    -Name LongPathsEnabled -PropertyType DWord -Value 1 -Force

Get-ItemProperty `
    -Path 'HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem' `
    -Name LongPathsEnabled
```

结果应为 `LongPathsEnabled : 1`。修改后建议重启一次。

### 5.2 电源计划和目录

```powershell
powercfg /S SCHEME_MIN

New-Item -ItemType Directory -Force D:\Isaac
New-Item -ItemType Directory -Force D:\reboot\assets_unzip
New-Item -ItemType Directory -Force D:\reboot\GR00T-WholeBodyControl\gear_sonic
New-Item -ItemType Directory -Force D:\Isaac\installers
New-Item -ItemType Directory -Force D:\Isaac\drivers
```

## 6. 通过 SSH 在局域网中直传

下面以 Linux 控制端为例。建议先在 `~/.ssh/config` 中为两台 Windows 配置别名，
并为目标机使用单独密钥。示例中的用户名和密钥路径需要按现场替换：

```sshconfig
Host pipeline-win-source
    HostName 192.168.1.130
    User <参考机用户名>
    IdentityFile <参考机私钥路径>

Host pipeline-win-target
    HostName 192.168.1.129
    User Administrator
    IdentityFile <目标机私钥路径>
```

先验证两端：

```bash
ssh pipeline-win-source 'hostname'
ssh pipeline-win-target 'hostname'
```

传输使用“参考机 `tar` 到标准输出 → SSH 管道 → 目标机 `tar` 解包”的方式，不在
控制端留下几十 GB 的中间压缩包。传输期间保持两个 SSH 服务稳定，不要让 Windows
休眠。

### 6.1 工程和源码

可按目录分批传输，方便失败后重试：

```bash
ssh pipeline-win-source \
  'tar -C D:\Isaac -cf - unitree_sim_isaaclab xiaoyangIsaacLab unitree_sdk2_python' \
| ssh pipeline-win-target 'tar -C D:\Isaac -xf -'
```

CycloneDDS 源码和本地构建目录也按参考机的实际目录名传入 `tar`。完成后立即核对
两个仓库的分支、提交和 `git status`。

### 6.2 场景资源和 GR00T 数据

```bash
ssh pipeline-win-source \
  'tar -C D:\reboot\assets_unzip -cf - assets' \
| ssh pipeline-win-target 'tar -C D:\reboot\assets_unzip -xf -'

ssh pipeline-win-source \
  'tar -C D:\reboot\GR00T-WholeBodyControl\gear_sonic -cf - data' \
| ssh pipeline-win-target \
  'tar -C D:\reboot\GR00T-WholeBodyControl\gear_sonic -xf -'
```

本工程的 `assets` 使用目录联接指向统一资源目录。在目标机管理员 PowerShell 中，
确认工程内没有需要保留的真实 `assets` 目录后再执行：

```powershell
cmd /c mklink /J `
    D:\Isaac\unitree_sim_isaaclab\assets `
    D:\reboot\assets_unzip\assets
```

检查联接和目标：

```powershell
Get-Item D:\Isaac\unitree_sim_isaaclab\assets |
    Format-List FullName, LinkType, Target, Attributes
```

不要长期保留同一份资源的第二份直接拷贝。应先完成冒烟测试，确认目录联接工作正常，
再清理临时副本。

### 6.3 复制 Conda/Isaac 环境

本次环境被复刻到与参考机**完全相同的前缀**：

```text
C:\Users\nolovr\miniconda3\envs\env_isaaclab
```

即使目标机登录用户是 `Administrator`，也保留该路径。原因是环境内的启动脚本、
`.pth`、editable install 和部分二进制入口可能包含绝对前缀；改到
`C:\Users\Administrator\...` 后不一定能直接运行。

先在目标机创建目录：

```powershell
New-Item -ItemType Directory -Force C:\Users\nolovr\miniconda3
```

传输时必须排除 Isaac Kit 的运行日志和缓存：

```bash
ssh pipeline-win-source \
  'tar -C C:\Users\nolovr\miniconda3 \
    --exclude=envs/env_isaaclab/Lib/site-packages/isaacsim/kit/logs \
    --exclude=envs/env_isaaclab/Lib/site-packages/isaacsim/kit/cache \
    -cf - .' \
| ssh pipeline-win-target \
  'tar -C C:\Users\nolovr\miniconda3 -xf -'
```

这两个目录包含参考机正在使用或由 GPU/驱动生成的文件，直接打包时可能出现共享冲突；
它们也不应跨机器复用，Isaac Sim 会在目标机首次运行时重新生成。

本次目标机排除缓存后的有效环境统计是 167,392 个文件、17,915,218,763 字节。
它不应与包含缓存的参考机总字节数机械相等，应以核心包导入和实跑结果为准。

### 6.4 Git、VS Code、SteamVR、XRLink 和驱动

Git 和 VS Code 必须使用官方安装包正式安装，不能把 `Program Files` 或
`AppData\Local\Programs` 下的程序目录直接复制过去。仅复制程序目录虽然可能让绝对
路径下的 EXE 暂时可运行，但不会正确写入卸载注册信息、安装任务和 PATH，后续终端会
出现“文件存在但找不到命令”的状态。

- Git：安装与参考机一致的 `Git-2.33.0.2-64-bit.exe`；
- VS Code：安装与参考机一致的 `VSCodeUserSetup-x64-1.113.0.exe`，目标用户为
  `Administrator`；
- SteamVR：复制参考机 `C:\Program Files\Steam`。复制程序目录不会复制或绕过账号
  登录，目标机仍需由操作者正常登录；
- XRLink：优先使用原始 MSI。只有原始安装包已丢失时，才从参考机 Installer 缓存
  取出已确认属于 XRLink 的 MSI，并先验证数字签名；
- NVIDIA 驱动：优先使用 NVIDIA 官方离线安装包。需要锁定参考机 DriverStore 中的
  同版驱动时，复制完整驱动包目录后使用 `pnputil` 安装。

Git 2.33.0(2) 官方发布页给出的 x64 安装包 SHA-256 是：

```text
https://github.com/git-for-windows/git/releases/download/v2.33.0.windows.2/Git-2.33.0.2-64-bit.exe
A5704733C219E9A0C96BFEB0FEBEF62BC2518BDD4E358BC9519DBC5E63A3B5FE
```

VS Code x64 User Setup 的官方版本下载入口是：

```text
https://update.code.visualstudio.com/1.113.0/win32-x64-user/stable
```

如果目标机访问 GitHub 很慢，可在可信控制端下载官方安装包，校验后再通过局域网传到
`D:\Isaac\installers`。目标机安装前必须再次检查 Git 哈希和两份 Authenticode
签名：

```powershell
$gitInstaller = 'D:\Isaac\installers\Git-2.33.0.2-64-bit.exe'
$codeInstaller = 'D:\Isaac\installers\VSCodeUserSetup-x64-1.113.0.exe'

Get-FileHash $gitInstaller -Algorithm SHA256
Get-AuthenticodeSignature $gitInstaller |
    Format-List Status, SignerCertificate
Get-AuthenticodeSignature $codeInstaller |
    Format-List Status, SignerCertificate
```

Git 哈希必须与上面的官方值完全一致，两份签名状态都必须为 `Valid`，VS Code 的签名
主体必须是 Microsoft。目标机管理员 PowerShell 静默安装：

```powershell
$gitArgs = '/VERYSILENT /NORESTART /NOCANCEL /SP- ' +
    '/CLOSEAPPLICATIONS /RESTARTAPPLICATIONS ' +
    '/DIR="C:\Program Files\Git" ' +
    '/LOG="D:\Isaac\installers\git-install.log"'
Start-Process $gitInstaller -ArgumentList $gitArgs -Wait

$codeArgs = '/VERYSILENT /NORESTART /CURRENTUSER ' +
    '/MERGETASKS="!runcode,addcontextmenufiles,addcontextmenufolders,' +
    'associatewithfiles,addtopath" ' +
    '/LOG="D:\Isaac\installers\vscode-install.log"'
Start-Process $codeInstaller -ArgumentList $codeArgs -Wait
```

正式安装后验证版本、PATH 和卸载注册信息：

```powershell
& 'C:\Program Files\Git\cmd\git.exe' --version
& "$env:LOCALAPPDATA\Programs\Microsoft VS Code\bin\code.cmd" --version

Get-ItemProperty `
    'HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\*', `
    'HKCU:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\*' `
    -ErrorAction SilentlyContinue |
    Where-Object DisplayName -Match '^(Git|Microsoft Visual Studio Code)'
```

Git 应注册为 `2.33.0.2`，VS Code 应注册为 `1.113.0`。安装器会分别把
`C:\Program Files\Git\cmd` 写入 Machine PATH，把 VS Code 的 `bin` 写入 User
PATH；旧终端仍可能保留安装前环境，需要重新登录桌面或重启终端父进程。

复制 Steam 前先在参考机退出 Steam、SteamVR 和 XRLink，避免运行中的日志、数据库或
运行时文件被锁定。

复制 Steam 的管道示例：

```bash
ssh pipeline-win-source \
  'tar -C "C:\Program Files" -cf - Steam' \
| ssh pipeline-win-target \
  'tar -C "C:\Program Files" -xf -'
```

本次 581.80 驱动包来自参考机：

```text
C:\Windows\System32\DriverStore\FileRepository\nvmdi.inf_amd64_c2d1126d336032b3
```

目标机保存到：

```text
D:\Isaac\drivers\nvmdi.inf_amd64_c2d1126d336032b3
```

目标机管理员 PowerShell 安装命令：

```powershell
pnputil /add-driver `
  D:\Isaac\drivers\nvmdi.inf_amd64_c2d1126d336032b3\nvmdi.inf `
  /install
```

`pnputil` 可能花数分钟验证签名和复制大量文件，期间 CPU 占用很低也可能是正常现象。
不要因为短时间没有新输出就强制结束。可在另一个管理员 PowerShell 中查看：

```powershell
Get-Content C:\Windows\INF\setupapi.dev.log -Tail 100 -Wait
```

安装成功后重启，并用 `nvidia-smi` 确认驱动版本和 GPU。

## 7. 配置目标 Viewer

### 7.1 系统和用户环境变量

在目标机管理员 PowerShell 中执行。下面是本次 129 的实际值：

```powershell
$values = @{
    PIPELINE_HOST_IP = '192.168.1.131'
    SIM_DDS_IFACE    = '192.168.1.129'
    GR00T_WBC_ROOT   = 'D:\reboot\GR00T-WholeBodyControl'
    SIM_PYTHON       = 'C:\Users\nolovr\miniconda3\envs\env_isaaclab\python.exe'
}

foreach ($item in $values.GetEnumerator()) {
    [Environment]::SetEnvironmentVariable($item.Key, $item.Value, 'Machine')
    [Environment]::SetEnvironmentVariable($item.Key, $item.Value, 'User')
}
```

设置后需要重新登录桌面会话，或重启 Explorer/Windows，已有进程不会自动获得新变量。

变量含义：

- `PIPELINE_HOST_IP`：Ubuntu Host 的地址，不是参考 Viewer 的地址；
- `SIM_DDS_IFACE`：目标 Viewer 自己的有线网卡地址；
- `GR00T_WBC_ROOT`：目标机 GR00T 根目录；
- `SIM_PYTHON`：保持原前缀的 Isaac Python 解释器。

### 7.2 Viewer 启动脚本的目标机默认值

环境变量可以覆盖默认值。为了桌面双击也不依赖旧会话，在目标机的以下两个文件中
把默认 Host IP 改为 `192.168.1.131`，并把 `SIM_DDS_IFACE` 默认值设为
`192.168.1.129`：

```text
D:\Isaac\unitree_sim_isaaclab\run_pipeline_viewer.bat
D:\Isaac\unitree_sim_isaaclab\run_pipeline_viewer_ar.bat
```

关键逻辑应等价于：

```bat
if not defined PIPELINE_HOST_IP set "PIPELINE_HOST_IP=192.168.1.131"
if not defined SIM_DDS_IFACE set "SIM_DDS_IFACE=192.168.1.129"
```

这是目标机的预期现场修改。部署完成后的
`D:\Isaac\unitree_sim_isaaclab` 工作树应只出现这两个已知脚本修改；如出现其他差异，
应逐项解释后再验收。

### 7.3 Windows 防火墙

Viewer 的 Pipeline 数据连接是出站 TCP，不需要为 `15555` 创建入站规则。本次为目标
Python 解释器创建了 Private Profile 的出站和入站允许规则，以免 Isaac/DDS 被本机
防火墙静默拦截。规则的程序路径必须是：

```text
C:\Users\nolovr\miniconda3\envs\env_isaaclab\python.exe
```

可按现场安全策略创建规则；不要对 Public Profile 无条件开放所有程序和端口。

本次使用的规则可按下面的方式创建，重复执行前先按 DisplayName 检查是否已存在：

```powershell
$python = 'C:\Users\nolovr\miniconda3\envs\env_isaaclab\python.exe'

New-NetFirewallRule `
    -DisplayName 'Unitree Pipeline Viewer Python (Private Out)' `
    -Direction Outbound -Action Allow -Profile Private -Program $python

New-NetFirewallRule `
    -DisplayName 'Unitree Pipeline Viewer Python (Private In)' `
    -Direction Inbound -Action Allow -Profile Private -Program $python
```

## 8. 安装和注册 XR 组件

### 8.1 SteamVR OpenXR Runtime

确认运行时文件存在：

```powershell
Test-Path 'C:\Program Files\Steam\steamapps\common\SteamVR\steamxr_win64.json'
```

将它注册为系统 OpenXR Runtime：

```powershell
New-Item -Path 'HKLM:\SOFTWARE\Khronos\OpenXR\1' -Force
Set-ItemProperty `
    -Path 'HKLM:\SOFTWARE\Khronos\OpenXR\1' `
    -Name ActiveRuntime `
    -Value 'C:\Program Files\Steam\steamapps\common\SteamVR\steamxr_win64.json'

Get-ItemProperty 'HKLM:\SOFTWARE\Khronos\OpenXR\1' -Name ActiveRuntime
```

### 8.2 Steam 登录与 Clash 直连

复制 Steam 程序目录不会迁移可复用的账号登录态，新机仍需在物理桌面重新登录。若
目标机运行 Clash Verge，Steam 登录认证和 CM WebSocket 不能走不稳定的代理节点，
否则登录窗口会反复出现：

```text
Failed to poll auth session. Result 2. Transport Error: 2
```

本次对照发现，130 的 Clash 配置中没有额外 Steam 规则；它登录时没有运行 Clash，
因此实际是直接连接。129 需要在 Clash 开启时工作，所以为登录和登录态连接增加显式
`DIRECT` 规则：

```yaml
prepend-rules:
  - DOMAIN,login.steampowered.com,DIRECT
  - DOMAIN,api.steampowered.com,DIRECT
  - DOMAIN-SUFFIX,steamserver.net,DIRECT
```

规则应写到当前订阅绑定的“配置增强 Merge”文件，而不是只修改自动生成的
`clash-verge.yaml`。配置目录通常是：

```text
%APPDATA%\io.github.clash-verge-rev.clash-verge-rev\profiles
```

可在同目录的 `profiles.yaml` 中查看当前远程订阅 `option.merge` 指向的 UID，再编辑
对应的 `<UID>.yaml`。修改前先备份文件；不要输出或复制订阅 URL、控制器 secret 和
代理节点凭据。

这三条规则必须位于订阅通用规则和最终 `MATCH` 之前。保存后重载 Clash 内核，并
完全退出再启动 Steam。检查：

```powershell
Get-Content 'C:\Program Files\Steam\logs\connection_log.txt' -Tail 80 |
    Select-String 'ConnectionCompleted|LogOnResponse|Logged On'
```

正确结果应同时满足：

- Steam CM 连接的 local address 是目标机自身 IP，而不是 `127.0.0.1:7897`；
- 出现 `RecvMsgClientLogOnResponse() ... 'OK'`；
- 后续进入 `Logged On`，不再周期性报告认证轮询错误。

只把 `login.steampowered.com` 直连还不够：认证 API 实际还会访问
`api.steampowered.com`，认证完成后的会话由 `steamserver.net` CM WebSocket 承载。

### 8.3 XRLink

本次使用已确认属于 XRLink 3.0.1 的 MSI。安装前检查签名：

```powershell
Get-AuthenticodeSignature D:\Isaac\installers\XRLink-3.0.1.msi |
    Format-List Status, StatusMessage, SignerCertificate
```

静默安装：

```powershell
Start-Process msiexec.exe -Wait -ArgumentList @(
    '/i',
    'D:\Isaac\installers\XRLink-3.0.1.msi',
    '/qn',
    '/norestart'
)
```

验证：

```powershell
Test-Path 'C:\Program Files\NOLO.lnc\XRLink\XRLink.exe'
```

### 8.4 桌面入口

本次目标机创建了以下快捷方式：

- `Unitree Pipeline Viewer.lnk`
- `Unitree Pipeline AR Robot1.lnk`
- `Unitree Pipeline AR Robot2.lnk`
- `Steam.lnk`
- `XRLink.lnk`

Robot1/Robot2 AR 快捷方式的区别是设置
`ISAACLAB_XR_ANCHOR_ROBOT_ID=1` 或 `2`。快捷方式应放到实际登录并佩戴头显的操作者
桌面，而不是参考机用户目录。

AR 必须从 Windows 物理桌面会话启动，不能通过 SSH 启动。SSH 会话没有可用的交互式
OpenXR/SteamVR 图形会话，远程命令能启动进程也不代表头显链路正常。

## 9. 环境验证

目标机 PowerShell：

```powershell
$python = 'C:\Users\nolovr\miniconda3\envs\env_isaaclab\python.exe'

Get-NetIPAddress -AddressFamily IPv4 |
    Select-Object InterfaceAlias, IPAddress
Test-NetConnection 192.168.1.131 -Port 15555

& $python --version
& $python -c "import torch; print(torch.__version__); print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0))"
& $python -c "from importlib.metadata import version; import cyclonedds; print(version('cyclonedds'))"
& $python -c "from importlib.metadata import version; import h5py; print(version('h5py'))"
& $python -c "from importlib.metadata import version; import isaacsim; print(version('isaacsim'))"
& $python -c "from importlib.metadata import version; import zmq; print(version('pyzmq'))"
& $python -m pip show isaaclab isaaclab_tasks isaaclab_mimic unitree-sdk2py
```

验收重点：

- Python 为 `3.11.15`；
- PyTorch 为 `2.7.0+cu128`；
- `torch.cuda.is_available()` 为 `True`；
- GPU 名称为 RTX 5080；
- CycloneDDS 可导入且版本为 `0.10.5`，不能误升级到不兼容的大版本；
- IsaacLab editable 包指向 `D:\Isaac\xiaoyangIsaacLab`；
- `unitree-sdk2py` editable 包指向 `D:\Isaac\unitree_sdk2_python`。

## 10. Viewer 冒烟测试

先测试普通 Viewer，再测试 AR。普通 Viewer 可以通过 SSH 做无头测试；AR 必须在物理
桌面进行。

### 10.1 无 Host 的基础冒烟

用 SSH 进入目标机 PowerShell，然后直接运行脚本，不要用会迅速退出的
`Start-Process` 外壳来判断结果：

```powershell
$env:PYTHONUNBUFFERED = '1'
$env:SIM_LOG = 'D:\Isaac\pipeline_viewer_smoke.log'
Set-Location D:\Isaac\unitree_sim_isaaclab
cmd /c .\run_pipeline_viewer.bat --headless
```

`PYTHONUNBUFFERED=1` 很重要，否则日志重定向后可能长时间看似为空。

另开一个 SSH 会话观察：

```powershell
Get-Content D:\Isaac\pipeline_viewer_smoke.log -Tail 100 -Wait
```

无 Host 时，程序仍应完成 Isaac、任务和场景初始化，并进入纯 Viewer 的 hold action
模式。不能出现 Python `Traceback`、`FATAL`、DLL 导入失败或资源找不到。

### 10.2 连接 Host 的完整冒烟

先确认 Ubuntu Host 已监听：

```bash
ss -ltnp | rg ':15555'
```

目标机执行：

```powershell
$env:PYTHONUNBUFFERED = '1'
$env:SIM_LOG = 'D:\Isaac\pipeline_viewer_full_smoke.log'
Set-Location D:\Isaac\unitree_sim_isaaclab
cmd /c .\run_pipeline_viewer.bat --headless
```

Host 上检查连接：

```bash
ss -tnp | rg ':15555'
```

当 130 和 129 同时在线时，Host 应看到来自两台 Viewer 的 ESTABLISHED 连接。

日志验收判据：

- 连接地址为 `tcp://192.168.1.131:15555`；
- 场景出现 `peer_robot`、`peer_robot_2` 和传送带对象；
- 日志显示 Viewer 选择 hold action source，不部署策略、不参与 DDS lock-step；
- `physics_steps` 持续增长；
- `sync_waits=0` 或基本不增长；
- 无持续 `Stream stale`、无 `Traceback`、无 `FATAL`。

本次完整冒烟中 `physics_steps` 达到 1500，`sync_waits=0`，统计周期约
18.3–20.2 ms。

### 10.3 安全结束测试进程

测试时应在第二个会话中找到准确的 Python 子进程，再结束对应 PID：

```powershell
Get-CimInstance Win32_Process -Filter "Name='python.exe'" |
    Select-Object ProcessId, ParentProcessId, CommandLine

taskkill /PID <准确的PID> /T /F
```

强制结束后外层脚本记录 `SIM_EXIT=1` 是预期结果，不代表此前的运行验证失败。结束后
再次确认没有遗留仿真 Python 进程。

### 10.4 AR 桌面验收

在目标机物理桌面按顺序执行：

1. 启动 XRLink，连接 Pico/NOLO 设备；
2. 启动 Steam，确认 SteamVR 识别头显；
3. 确认 OpenXR ActiveRuntime 指向 SteamVR；
4. 双击 Robot1 或 Robot2 对应的 AR Viewer 快捷方式；
5. 检查头显画面、镜像机器人锚定、B 键 recenter 和 Host 场景更新。

SSH 无头冒烟通过只能证明 Python/Isaac/网络链路正常，不能替代这一步。

## 11. 常见故障和本次踩坑

### 11.1 `Start-Service ssh` 找不到服务

OpenSSH Server 服务名是 `sshd`。使用：

```powershell
Start-Service sshd
```

如果 `sshd` 也不存在，再检查 `OpenSSH.Server` Windows Capability 是否已安装。

### 11.2 SSH 提示 Host Key 改变

先在目标机本地运行 `ssh-keygen -lf`，核对新指纹。确认是重装后的同一台机器后再删除
旧记录并重新连接；未核对时不要绕过检查。

### 11.3 Conda 打包在 Isaac Kit 目录失败

常见原因是 `isaacsim/kit/logs` 或 `isaacsim/kit/cache` 中有锁定文件。停止 Isaac
进程，并在打包时排除这两个机器相关目录。不要为了复制缓存而反复强杀系统进程。

### 11.4 复制后的 Python 入口或 editable 包失效

Conda 环境复制后路径必须与参考机前缀一致。本次目标机虽然登录用户不同，仍保留
`C:\Users\nolovr\miniconda3`。验证 `pip show` 的 `Location` 和 editable project
location，不能只看 `python --version`。

### 11.5 `Start-Process` 已退出，但 Isaac 仍在运行

BAT、Conda 和 Python 会形成父子进程链。外层 PowerShell 或 `cmd.exe` 退出不代表
最终 `python.exe` 已退出，反过来也不能用外壳退出码判断 Isaac 是否启动成功。应同时
检查 Python 子进程、日志和 Host TCP 连接。

### 11.6 日志为空

重定向到文件时设置 `PYTHONUNBUFFERED=1`，并尽量让启动命令直接附着在 SSH exec
会话。使用另一个会话跟踪日志和进程。

### 11.7 `pnputil` 长时间无输出

驱动签名和文件验证可能持续数分钟。检查 `setupapi.dev.log` 的进展，确认进程仍存在，
不要仅因 CPU 很低就结束安装。本次日志最终为成功，重启后 `nvidia-smi` 显示
`581.80`。

### 11.8 普通 Viewer 成功，AR 无画面

先确认 AR 是从物理桌面而不是 SSH 启动；然后依次检查 XRLink、SteamVR 头显状态和
OpenXR ActiveRuntime。普通 Viewer 的无头成功不覆盖图形会话和头显链路。

### 11.9 Steam 登录窗口持续轮询失败

先检查 `connection_log.txt` 和 `cef_log.txt`。如果 Steam 连接的对端是本机 Clash
端口，且日志包含 `Transport Error: 2`、代理断开后又重连的记录，按 §8.2 增加三条
前置 `DIRECT` 规则并重载内核。DNS、网页 HTTPS 和 SteamVR 下载正常不能排除这个
问题，因为内容 CDN 正常不代表认证 API 与 CM WebSocket 的代理链路稳定。

### 11.10 Git 文件存在但终端找不到命令

若 `C:\Program Files\Git\cmd\git.exe` 存在，但“应用和功能”中没有 Git，说明只复制了
程序目录，没有完成正式安装。不要用手工 PATH 或系统目录 shim 掩盖，应重新运行官方
Git 安装包，使卸载注册信息和 PATH 由安装器写入。安装完成后用绝对路径验证版本，再
重新登录 Windows 或刷新终端父进程。

## 12. 最终验收清单

部署交付前逐项确认：

- [ ] 目标机 Host Key 已在本机控制台核对，SSH 公钥登录正常；
- [ ] Windows 长路径为 `1`，电源计划为高性能；
- [ ] `nvidia-smi` 显示 RTX 5080 和驱动 `581.80`；
- [ ] 两个 Git 工程分支和提交与基线一致；
- [ ] Git 和 VS Code 均有正式卸载注册信息，`git` 与 `code` CLI 版本正确；
- [ ] 工程工作树只有已记录的目标机脚本修改和参考机已有文件；
- [ ] assets 和 GR00T data 的文件数、字节数与参考机一致；
- [ ] 工程 `assets` 是指向统一资源目录的 Junction；
- [ ] Python、Isaac Sim、PyTorch、CUDA、CycloneDDS 和 h5py 导入通过；
- [ ] IsaacLab、Unitree SDK 的 editable 路径正确；
- [ ] `PIPELINE_HOST_IP` 指向 Host，`SIM_DDS_IFACE` 是目标机自身 IP；
- [ ] Host 能看到目标 Viewer 到 TCP `15555` 的 ESTABLISHED 连接；
- [ ] `physics_steps` 持续增加，`sync_waits` 不增长，无 Traceback/FATAL；
- [ ] Clash 开启时 Steam 登录认证与 CM 连接命中前置 `DIRECT` 规则；
- [ ] SteamVR Runtime 已注册，XRLink 已安装；
- [ ] AR 已在物理桌面完成头显、锚定和 recenter 验收；
- [ ] 冒烟测试结束后无遗留 Python/Isaac 进程。

## 13. 本次 129 部署记录

本次部署最终状态：

- 目标主机名：`DESKTOP-72SEFB7`；
- 参考主机名：`DESKTOP-1MNMLD2`；
- Host / 参考 Viewer / 目标 Viewer：`192.168.1.131 / .130 / .129`；
- NVIDIA 驱动由 `576.88` 更新并验证为 `581.80`；
- Git for Windows `2.33.0.2` 与 VS Code User `1.113.0` 已使用官方安装包正式安装，
  两者签名、PATH、卸载注册信息和 CLI 均已验证；
- 场景资源与 GR00T data 的文件数、字节数与参考机一致；
- `D:\Isaac\unitree_sim_isaaclab\assets` 已联接到统一资源目录；
- 普通 Viewer 无 Host 初始化测试通过；
- 连接 Host 的完整测试通过，同时确认 129 和 130 均连接到 `131:15555`；
- Clash 已添加 Steam 认证 API 与 CM 的持久化直连规则；Steam CM 的 local address
  已从 `127.0.0.1:7897` 变为 `192.168.1.129`，并返回 `LogOnResponse: OK`；
- 测试进程和临时启动包装已清理；
- 保留日志：
  - `D:\Isaac\pipeline_viewer_smoke.log`
  - `D:\Isaac\pipeline_viewer_full_smoke.log`
- 部署完成时剩余空间约为：C 盘 111 GB，D 盘 275 GB。

至此，129 已达到与 130 相同的软件和 Viewer 能力基线。唯一仍需由操作者在现场完成
的是物理头显条件下的 AR 交互验收。
