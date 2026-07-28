#!/usr/bin/env bash
# perf_mode.sh — 本机 CPU / GPU / 内存 性能模式一键切换
#
# 用法:
#   perf_mode.sh status     查看三项当前状态(不需要 sudo)
#   perf_mode.sh on         切到性能模式(立即生效,重启后 CPU 档保持,GPU/swappiness 失效)
#   perf_mode.sh off        恢复默认(balanced / adaptive / swappiness=60)
#   perf_mode.sh install    写入开机持久化(sysctl.d + systemd + GNOME autostart),并立即执行 on
#   perf_mode.sh uninstall  移除持久化文件(不改变当前运行状态,需要的话再跑一次 off)
#
# 各项对应关系:
#   CPU    powerprofilesctl performance/balanced  → intel_pstate governor + EPP,自动持久
#   GPU    nvidia-smi -pm 1        persistence mode,驱动常驻,CUDA 启动快(重启失效)
#          PowerMizerMode=1        最高性能优先,避免降频抖动(重启失效,需 X 会话)
#   内存   vm.swappiness=10        少往 swap 换页,仿真/推理时避免卡顿(重启失效)
#
# 说明见知识库: ~/文档/engineer-handbook/wiki/Linux/Linux 性能模式（CPU-GPU-内存）.md
set -euo pipefail

GPU="[gpu:0]"
SWAPPINESS_PERF=10
SWAPPINESS_DEFAULT=60
SYSCTL_CONF=/etc/sysctl.d/99-perf-mode.conf
SYSTEMD_UNIT=/etc/systemd/system/perf-mode-nvidia.service
AUTOSTART_DESKTOP="$HOME/.config/autostart/perf-mode-powermizer.desktop"

note() { printf '\033[32m[perf_mode]\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[perf_mode] 警告:\033[0m %s\n' "$*" >&2; }

have() { command -v "$1" >/dev/null 2>&1; }

# nvidia-settings 需要 X 会话;ssh 无 DISPLAY 时跳过并提示。
# 坑:580 驱动 + Ada(40 系)上赋值返回 assigned 但读回仍是 0,属驱动遗留兼容项,
# 实际锁频要用 nvidia-smi -lgc。这里赋值后校验读回,POWERMIZER_OK 供 install 判断。
POWERMIZER_OK=0
powermizer_set() {
    local mode="$1" readback
    if ! have nvidia-settings; then
        warn "nvidia-settings 不存在,跳过 PowerMizer"
        return 0
    fi
    if [ -z "${DISPLAY:-}" ]; then
        warn "无 DISPLAY(ssh 会话?),跳过 PowerMizer,请在图形会话里执行: nvidia-settings -a '$GPU/GPUPowerMizerMode=$mode'"
        return 0
    fi
    if ! nvidia-settings -a "$GPU/GPUPowerMizerMode=$mode" >/dev/null 2>&1; then
        warn "PowerMizer 设置失败(Wayland 会话下不可用)"
        return 0
    fi
    readback=$(nvidia-settings -t -q "$GPU/GPUPowerMizerMode" 2>/dev/null || echo "?")
    if [ "$readback" = "$mode" ]; then
        POWERMIZER_OK=1
        note "GPU PowerMizer → $mode ($([ "$mode" = 1 ] && echo 最高性能优先 || echo 自适应))"
    else
        warn "PowerMizer 赋值未生效(本驱动上是遗留项,GPU 由驱动自动 boost;确需锁频用: sudo nvidia-smi -lgc 210,2610 / 解锁 -rgc)"
    fi
}

status() {
    echo "===== CPU ====="
    if have powerprofilesctl; then
        echo "电源档位(powerprofilesctl): $(powerprofilesctl get)"
    fi
    echo -n "governor: "
    sort /sys/devices/system/cpu/cpufreq/policy*/scaling_governor | uniq -c | awk '{printf "%s ×%s  ", $2, $1}'; echo
    echo "EPP: $(cat /sys/devices/system/cpu/cpufreq/policy0/energy_performance_preference)"
    echo "  → 性能模式期望: governor=performance, EPP=performance"
    echo
    echo "===== GPU ====="
    if have nvidia-smi; then
        nvidia-smi --query-gpu=name,persistence_mode,pstate,clocks.sm,clocks.max.sm,power.draw,power.limit \
            --format=csv,noheader | awk -F', ' '{printf "%s | persistence=%s | pstate=%s | SM %s / %s | 功耗 %s / %s\n", $1,$2,$3,$4,$5,$6,$7}'
    fi
    if have nvidia-settings && [ -n "${DISPLAY:-}" ]; then
        local pm; pm=$(nvidia-settings -t -q "$GPU/GPUPowerMizerMode" 2>/dev/null || echo "?")
        echo "PowerMizer: $pm (0=自适应 1=最高性能优先)"
    fi
    echo "  → 性能模式期望: persistence=Enabled (跑 CUDA 时 pstate=P2 属正常;PowerMizer 在 580 驱动+Ada 上为只读遗留项,读 0 不用管)"
    echo
    echo "===== 内存 ====="
    echo "swappiness: $(cat /proc/sys/vm/swappiness)  (性能模式期望 $SWAPPINESS_PERF)"
    echo "THP: $(cat /sys/kernel/mm/transparent_hugepage/enabled)  (madvise 即为合理默认,不用动)"
    free -h | awk 'NR==1{print "        "$0} /^内存|^Mem/{print "内存:  "$2" 总 / "$3" 用"} /^交换|^Swap/{print "swap:  "$2" 总 / "$3" 用"}'
    echo
    echo "===== 持久化 ====="
    [ -f "$SYSCTL_CONF" ]        && echo "✓ $SYSCTL_CONF"        || echo "✗ sysctl 持久化未安装"
    [ -f "$SYSTEMD_UNIT" ]       && echo "✓ $SYSTEMD_UNIT ($(systemctl is-enabled perf-mode-nvidia.service 2>/dev/null || echo '?'))" \
                                 || echo "✗ nvidia persistence 开机服务未安装"
    [ -f "$AUTOSTART_DESKTOP" ]  && echo "✓ $AUTOSTART_DESKTOP"  || echo "- PowerMizer 登录自启未装(本驱动上该项不生效,无需安装)"
}

perf_on() {
    if have powerprofilesctl; then
        powerprofilesctl set performance
        note "CPU 电源档位 → performance (governor+EPP,自动持久)"
    else
        for f in /sys/devices/system/cpu/cpufreq/policy*/scaling_governor; do
            echo performance | sudo tee "$f" >/dev/null
        done
        note "CPU governor → performance (无 powerprofilesctl,重启失效)"
    fi
    sudo nvidia-smi -pm 1 >/dev/null && note "GPU persistence mode → 开"
    powermizer_set 1
    sudo sysctl -q vm.swappiness=$SWAPPINESS_PERF
    note "vm.swappiness → $SWAPPINESS_PERF"
    note "完成。跑 'perf_mode.sh status' 核对;要开机保持请跑 'perf_mode.sh install'"
}

perf_off() {
    if have powerprofilesctl; then
        powerprofilesctl set balanced
        note "CPU 电源档位 → balanced"
    fi
    sudo nvidia-smi -pm 0 >/dev/null && note "GPU persistence mode → 关"
    powermizer_set 0
    sudo sysctl -q vm.swappiness=$SWAPPINESS_DEFAULT
    note "vm.swappiness → $SWAPPINESS_DEFAULT"
}

install_persist() {
    perf_on

    printf '# perf_mode.sh 安装,仿真/推理机减少换页\nvm.swappiness = %s\n' "$SWAPPINESS_PERF" \
        | sudo tee "$SYSCTL_CONF" >/dev/null
    note "已写 $SYSCTL_CONF"

    sudo tee "$SYSTEMD_UNIT" >/dev/null <<'EOF'
[Unit]
Description=NVIDIA persistence mode on (perf_mode.sh)
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/usr/bin/nvidia-smi -pm 1

[Install]
WantedBy=multi-user.target
EOF
    sudo systemctl daemon-reload
    sudo systemctl enable perf-mode-nvidia.service >/dev/null 2>&1
    note "已装并 enable perf-mode-nvidia.service (开机 nvidia-smi -pm 1)"

    # PowerMizer 是 X 会话级设置,systemd 阶段没有 X,只能挂登录自启;
    # 本驱动上赋值不生效(POWERMIZER_OK=0)就不装,免得留一个无用自启项
    if [ "$POWERMIZER_OK" = 1 ]; then
        mkdir -p "$(dirname "$AUTOSTART_DESKTOP")"
        cat > "$AUTOSTART_DESKTOP" <<EOF
[Desktop Entry]
Type=Application
Name=perf-mode PowerMizer
Comment=GPU PowerMizer 最高性能优先 (perf_mode.sh)
Exec=nvidia-settings -a $GPU/GPUPowerMizerMode=1
NoDisplay=true
X-GNOME-Autostart-enabled=true
EOF
        note "已写 $AUTOSTART_DESKTOP (登录时设 PowerMizer=1)"
    else
        note "PowerMizer 在本驱动上不生效,跳过登录自启"
    fi
    note "持久化完成:CPU 档位 powerprofilesctl 自动记忆,swap/GPU 由上述文件负责"
}

uninstall_persist() {
    [ -f "$SYSCTL_CONF" ] && sudo rm -f "$SYSCTL_CONF" && note "已删 $SYSCTL_CONF"
    if [ -f "$SYSTEMD_UNIT" ]; then
        sudo systemctl disable perf-mode-nvidia.service >/dev/null 2>&1 || true
        sudo rm -f "$SYSTEMD_UNIT"
        sudo systemctl daemon-reload
        note "已删 perf-mode-nvidia.service"
    fi
    [ -f "$AUTOSTART_DESKTOP" ] && rm -f "$AUTOSTART_DESKTOP" && note "已删 $AUTOSTART_DESKTOP"
    note "持久化已移除;当前运行状态未动,需要立即恢复默认请跑 'perf_mode.sh off'"
}

case "${1:-status}" in
    status)    status ;;
    on)        perf_on ;;
    off)       perf_off ;;
    install)   install_persist ;;
    uninstall) uninstall_persist ;;
    *) echo "用法: $0 {status|on|off|install|uninstall}" >&2; exit 1 ;;
esac
