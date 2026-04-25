#!/usr/bin/env python3
"""
PID 自动调参软件 — 基于系统辨识的数学方法

支持两种测试模式：
  1. 阶跃响应测试 → 辨识一阶+滞后模型 → IMC 整定
  2. 继电反馈测试 → 提取 Ku/Tu → Ziegler-Nichols 整定

用法：
  python pid_auto_tuner.py --port COM3 --baud 115200
"""

import serial
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
import argparse
import sys
import time


# ============================================================
#  1. 数据采集：从串口读取 STM32 发送的测试数据
# ============================================================

class SerialCollector:
    """从 STM32 串口接收测试数据"""

    def __init__(self, port, baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None

    def connect(self):
        print(f"[连接] {self.port} @ {self.baud}")
        self.ser = serial.Serial(self.port, self.baud, timeout=1)
        time.sleep(0.5)
        print("[连接] OK")

    def collect(self, timeout=30):
        """
        等待 STM32 发送 #DATA_BEGIN ... #DATA_END 数据块
        返回: (mode, time_ms[], speed_rpm[], crossings[])
        """
        print(f"[采集] 等待数据... (超时 {timeout}s)")
        lines = []
        mode = None
        capturing = False
        crossings = []
        in_crossings = False
        start = time.time()

        while time.time() - start < timeout:
            if self.ser.in_waiting == 0:
                time.sleep(0.01)
                continue

            raw = self.ser.readline()
            try:
                line = raw.decode('utf-8', errors='ignore').strip()
            except:
                continue

            if not line:
                continue

            # 调试输出
            if line.startswith('#'):
                print(f"  [STM32] {line}")

            if line.startswith('#DATA_BEGIN'):
                parts = line.split()
                mode = parts[1] if len(parts) > 1 else 'UNKNOWN'
                capturing = True
                in_crossings = False
                lines = []
                continue

            if line == '#CROSSINGS':
                in_crossings = True
                continue

            if line == '#DATA_END':
                capturing = False
                break

            if capturing:
                if in_crossings:
                    parts = line.split(',')
                    if len(parts) == 2:
                        crossings.append((int(parts[0]), int(parts[1])))
                else:
                    parts = line.split(',')
                    if len(parts) == 2:
                        lines.append((int(parts[0]), int(parts[1])))

        if not lines:
            print("[采集] 超时，未收到数据")
            return None, None, None, None

        time_ms = np.array([l[0] for l in lines], dtype=float)
        speed = np.array([l[1] for l in lines], dtype=float)
        print(f"[采集] 完成: {len(time_ms)} 个点, 模式={mode}")
        return mode, time_ms, speed, crossings


# ============================================================
#  2. 系统辨识：从阶跃响应辨识一阶+滞后模型
# ============================================================

def first_order_delay(t, K, tau, theta, y0):
    """一阶惯性 + 纯滞后 模型: y = y0 + K * (1 - exp(-(t-theta)/tau)) * u(t-theta)"""
    y = np.full_like(t, y0, dtype=float)
    mask = t > theta
    y[mask] = y0 + K * (1.0 - np.exp(-(t[mask] - theta) / tau))
    return y


def identify_step_response(time_ms, speed_rpm):
    """
    从阶跃响应数据辨识模型参数
    返回: K, tau, theta, y0 (带单位)
    """
    t = time_ms / 1000.0  # 转为秒
    y = speed_rpm

    # 初始猜测
    y0_guess = y[0]
    K_guess = y[-1] - y[0]
    tau_guess = 0.1
    theta_guess = 0.02

    try:
        popt, pcov = curve_fit(
            first_order_delay, t, y,
            p0=[K_guess, tau_guess, theta_guess, y0_guess],
            bounds=([-np.inf, 0.001, 0, -np.inf],
                    [np.inf, 5.0, 1.0, np.inf]),
            maxfev=10000
        )
        K, tau, theta, y0 = popt
        perr = np.sqrt(np.diag(pcov))

        print(f"\n{'='*50}")
        print(f"  系统辨识结果 (阶跃响应)")
        print(f"{'='*50}")
        print(f"  增益      K   = {K:.4f} rpm/current")
        print(f"  时间常数  τ   = {tau*1000:.2f} ms")
        print(f"  滞后时间  θ   = {theta*1000:.2f} ms")
        print(f"  初始值    y0  = {y0:.2f} rpm")
        print(f"  τ/θ 比值      = {tau/theta:.2f}  (>3 表示系统好控)")
        print(f"{'='*50}")
        return K, tau, theta, y0, popt

    except Exception as e:
        print(f"[辨识] 拟合失败: {e}")
        return None, None, None, None, None


# ============================================================
#  3. 继电测试分析：提取 Ku 和 Tu
# ============================================================

def analyze_relay(time_ms, speed_rpm, crossings):
    """
    从继电反馈测试数据提取临界增益 Ku 和振荡周期 Tu
    """
    if len(crossings) < 4:
        print("[继电] 过零点不足，无法分析")
        return None, None

    t = time_ms / 1000.0

    # 从过零点计算振荡周期
    cross_times = [c[0] / 1000.0 for c in crossings]

    # 取后半段稳定振荡的过零点
    n = len(cross_times)
    stable = cross_times[max(0, n-10):]

    if len(stable) < 4:
        print("[继电] 稳定过零点不足")
        return None, None

    # 半周期 = 相邻过零点时间差
    half_periods = [stable[i+1] - stable[i] for i in range(len(stable)-1)]
    Tu_half = np.mean(half_periods)
    Tu = Tu_half * 2  # 全周期

    # 振幅 = 转速的最大值 - 最小值（后半段）
    mid = len(speed_rpm) // 2
    a = (np.max(speed_rpm[mid:]) - np.min(speed_rpm[mid:])) / 2.0

    # 继电幅值 d = RELAY_CURRENT
    d = 2500  # 与 STM32 代码中 RELAY_CURRENT 一致

    # 临界增益 Ku = 4d / (π * a)
    Ku = (4 * d) / (np.pi * a)

    print(f"\n{'='*50}")
    print(f"  继电反馈测试结果")
    print(f"{'='*50}")
    print(f"  振荡周期  Tu = {Tu*1000:.2f} ms")
    print(f"  振荡振幅  a  = {a:.2f} rpm")
    print(f"  继电幅值  d  = {d}")
    print(f"  临界增益  Ku = {Ku:.4f}")
    print(f"{'='*50}")
    return Ku, Tu


# ============================================================
#  4. PID 参数整定
# ============================================================

def imc_tuning(K, tau, theta, lambda_factor=3.0):
    """
    IMC (Internal Model Control) 整定
    基于一阶+滞后模型，计算 PID 参数

    lambda_factor: 闭环时间常数 = lambda_factor * theta
    返回: Kp, Ki, Kd
    """
    lam = lambda_factor * theta

    Kp = tau / (K * (theta + lam))
    Ki = Kp / tau   # 积分时间倒数
    Kd = 0          # 一阶模型不需要微分

    print(f"\n{'='*50}")
    print(f"  IMC 整定结果 (λ = {lambda_factor}θ = {lam*1000:.2f}ms)")
    print(f"{'='*50}")
    print(f"  Kp = {Kp:.4f}")
    print(f"  Ki = {Kp/tau:.4f}  (积分时间 Ti = {tau*1000:.2f}ms)")
    print(f"  Kd = {Kd}")
    print(f"{'='*50}")
    return Kp, Ki, Kd


def ziegler_nichols_tuning(Ku, Tu, controller_type='PID'):
    """
    Ziegler-Nichols 临界振荡法整定
    返回: Kp, Ki, Kd
    """
    if controller_type == 'P':
        Kp = 0.5 * Ku
        Ki = 0
        Kd = 0
    elif controller_type == 'PI':
        Kp = 0.45 * Ku
        Ki = 0.54 * Ku / Tu
        Kd = 0
    else:  # PID
        Kp = 0.6 * Ku
        Ki = 1.2 * Ku / Tu
        Kd = 0.075 * Ku * Tu

    print(f"\n{'='*50}")
    print(f"  Ziegler-Nichols 整定 ({controller_type})")
    print(f"{'='*50}")
    print(f"  Kp = {Kp:.4f}")
    print(f"  Ki = {Ki:.4f}")
    print(f"  Kd = {Kd:.6f}")
    print(f"{'='*50}")
    return Kp, Ki, Kd


def recommend_params(K, tau, theta, Ku=None, Tu=None):
    """
    综合推荐：给出多组参数供选择
    """
    print(f"\n{'='*60}")
    print(f"  综合推荐 PID 参数")
    print(f"{'='*60}")

    results = []

    # IMC 保守 (λ=5θ)
    lam5 = 5 * theta
    kp1 = tau / (K * (theta + lam5))
    ki1 = kp1 / tau
    results.append(("IMC 保守 (λ=5θ)", kp1, ki1, 0))

    # IMC 标准 (λ=3θ)
    lam3 = 3 * theta
    kp2 = tau / (K * (theta + lam3))
    ki2 = kp2 / tau
    results.append(("IMC 标准 (λ=3θ)", kp2, ki2, 0))

    # IMC 激进 (λ=1θ)
    lam1 = 1 * theta
    kp3 = tau / (K * (theta + lam1))
    ki3 = kp3 / tau
    results.append(("IMC 激进 (λ=1θ)", kp3, ki3, 0))

    # ZN (如果做了继电测试)
    if Ku and Tu:
        kp4 = 0.6 * Ku
        ki4 = 1.2 * Ku / Tu
        kd4 = 0.075 * Ku * Tu
        results.append(("Ziegler-Nichols PID", kp4, ki4, kd4))

    print(f"\n  {'方案':<22} {'Kp':>10} {'Ki':>10} {'Kd':>10}")
    print(f"  {'-'*55}")
    for name, kp, ki, kd in results:
        print(f"  {name:<22} {kp:>10.4f} {ki:>10.4f} {kd:>10.6f}")

    print(f"\n  DJI 3508 建议 max_out=10000, max_iout=2000")
    print(f"  建议先用「IMC 保守」，确认能跑再逐步激进")
    print(f"{'='*60}")

    return results


# ============================================================
#  5. 可视化
# ============================================================

def plot_step_response(time_ms, speed_rpm, popt, title="Step Response"):
    """画阶跃响应 + 拟合曲线"""
    fig, ax = plt.subplots(1, 1, figsize=(10, 5))

    t = time_ms / 1000.0
    ax.plot(t * 1000, speed_rpm, 'b-', linewidth=1, label='Actual (rpm)')

    if popt is not None:
        t_fit = np.linspace(t[0], t[-1], 500)
        y_fit = first_order_delay(t_fit, *popt)
        ax.plot(t_fit * 1000, y_fit, 'r--', linewidth=2,
                label=f'Fit: K={popt[0]:.2f}, τ={popt[1]*1000:.1f}ms, θ={popt[2]*1000:.1f}ms')

    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Speed (rpm)')
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('step_response.png', dpi=150)
    print("[图表] 已保存 step_response.png")
    plt.show()


def plot_relay(time_ms, speed_rpm, crossings, title="Relay Test"):
    """画继电测试波形"""
    fig, ax = plt.subplots(1, 1, figsize=(10, 5))

    t = time_ms / 1000.0
    ax.plot(t * 1000, speed_rpm, 'b-', linewidth=1, label='Speed (rpm)')

    for c in crossings:
        ax.axvline(c[0], color='r', alpha=0.5, linewidth=0.8)

    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Speed (rpm)')
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('relay_response.png', dpi=150)
    print("[图表] 已保存 relay_response.png")
    plt.show()


# ============================================================
#  6. 主程序
# ============================================================

def main():
    parser = argparse.ArgumentParser(description='PID Auto Tuner')
    parser.add_argument('--port', type=str, default='COM3', help='串口端口')
    parser.add_argument('--baud', type=int, default=115200, help='波特率')
    parser.add_argument('--lambda-factor', type=float, default=3.0, help='IMC λ 因子 (默认 3.0)')
    parser.add_argument('--step-current', type=int, default=2000, help='阶跃电流值')
    parser.add_argument('--offline', type=str, default=None, help='离线模式：从 CSV 文件加载数据')
    args = parser.parse_args()

    mode = None
    time_ms = None
    speed = None
    crossings = []

    if args.offline:
        # 离线模式：从文件加载
        print(f"[离线] 加载数据: {args.offline}")
        data = np.loadtxt(args.offline, delimiter=',', skiprows=1)
        time_ms = data[:, 0]
        speed = data[:, 1]
        mode = 'STEP'  # 默认当阶跃处理
        print(f"[离线] {len(time_ms)} 个点")
    else:
        # 在线模式：串口采集
        collector = SerialCollector(args.port, args.baud)
        collector.connect()
        mode, time_ms, speed, crossings = collector.collect(timeout=30)
        collector.ser.close()

    if mode is None:
        print("[退出] 无数据")
        return

    # ===== 根据模式处理 =====
    if mode == 'STEP':
        # 阶跃响应辨识
        print("\n" + "="*60)
        print("  阶跃响应分析")
        print("="*60)

        K, tau, theta, y0, popt = identify_step_response(time_ms, speed)

        if K is not None:
            results = recommend_params(K, tau, theta)
            plot_step_response(time_ms, speed, popt)

            # 保存推荐参数到文件
            with open('pid_params_recommended.txt', 'w') as f:
                f.write("# PID 参数推荐 (基于阶跃响应 IMC 整定)\n")
                f.write(f"# 模型: K={K:.4f}, tau={tau:.4f}s, theta={theta:.4f}s\n\n")
                for name, kp, ki, kd in results:
                    f.write(f"# {name}\n")
                    f.write(f"fp32 pid_param[3] = {{{kp:.4f}, {ki:.4f}, {kd:.6f}}};\n\n")
            print("[输出] 已保存 pid_params_recommended.txt")

    elif mode == 'RELAY':
        # 继电测试分析
        print("\n" + "="*60)
        print("  继电反馈分析")
        print("="*60)

        Ku, Tu = analyze_relay(time_ms, speed, crossings)

        if Ku is not None:
            results = recommend_params(0, 0, 0, Ku, Tu)
            plot_relay(time_ms, speed, crossings)

            with open('pid_params_recommended.txt', 'w') as f:
                f.write("# PID 参数推荐 (基于继电反馈 ZN 整定)\n")
                f.write(f"# Ku={Ku:.4f}, Tu={Tu:.4f}s\n\n")
                for name, kp, ki, kd in results:
                    f.write(f"# {name}\n")
                    f.write(f"fp32 pid_param[3] = {{{kp:.4f}, {ki:.4f}, {kd:.6f}}};\n\n")
            print("[输出] 已保存 pid_params_recommended.txt")

    print("\n[完成]")


if __name__ == '__main__':
    main()
