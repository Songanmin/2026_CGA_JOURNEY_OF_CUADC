#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""室内台架虚拟 GPS —— 给任务状态机喂固定 fix 过锁框架门禁（纯 DDS 话题叠加，不碰 FC）

用途：室内无真实 GPS fix，任务 FSM 的 navigation_ready() 恒 false、停死
WAIT_NAV_STABLE（SSOT §5.1 红线① fail-closed，设计如此）。本节点在
/mavros/global_position/global 上叠加 10Hz 的 status=STATUS_FIX NavSatFix 帧，
任务门禁取「最近一帧」即通过——FSM 可走 LOCK_FRAME→PRESTREAM→WAIT_GUIDED→WAIT_ARM，
从而在真机台架上验证室内可启动的全部状态链。

⚠️ 安全边界（三条，违反任一立即 Ctrl-C）：
  1. 只在室内台架、拆桨状态使用；
  2. 绝不在室外/真飞/自主飞行时运行——本帧的经纬度/海拔全是假的，会污染真实 GPS 判读；
  3. FSM 到 WAIT_ARM 即停（onekey 默认不解锁，解锁权在飞手=仓库红线 2）；
     本工具从不下发任何 MAVLink 指令，纯话题叠加。

原理：mavros 的真实 NO_FIX 帧（~1Hz）与本工具的 FIX 帧（默认 10Hz）在同一话题上
交错，任务门禁按「最近一帧」判定 → 恒见 FIX。FC 的 EKF 完全不吃这些数据，零侵入。

用法（Jetson 上，非交互 ssh 直用亦可，缺 ROS 环境自动 source 后重执行自身）:
  python3 fc_virtual_gps.py [--lat 30.0] [--lon 120.0] [--alt 20.0] [--hz 10] [--dur 0] [--status 0]
  --dur 0 = 一直发到 Ctrl-C；--status -1 可反向注入 NO_FIX（负向测试用）。
PC 端（Windows）入口 = ops 自动 ssh 远跑:
  python ops.py vgps
"""
import argparse
import os
import shutil
import sys


def ensure_ros(argv):
    """ros2/rclpy 不在 PATH → bash -lc source 后重执行自身（onekey 同款）。"""
    if os.name == "nt":
        sys.exit("[vgps] 本工具须在 Jetson 上运行（DDS 话题与 mavros 同域）。"
                 "Windows 端请用: python ops.py vgps")
    if shutil.which("ros2"):
        return
    print("[env] ros2 不在 PATH，source ROS 后重执行自身…")
    src = ("source /opt/ros/humble/setup.bash && "
           "source $HOME/cuadc_ws/install/setup.bash")
    cmd = " ".join("'" + a.replace("'", "'\\''") + "'" for a in [sys.executable] + argv)
    os.execvp("bash", ["bash", "-lc", f"{src} && exec {cmd}"])


def main():
    ap = argparse.ArgumentParser(description="室内台架虚拟 GPS（拆桨！只测 FSM 到 WAIT_ARM）")
    ap.add_argument("--lat", type=float, default=30.0, help="虚拟纬度（默认 30.0，假值勿用于真导航）")
    ap.add_argument("--lon", type=float, default=120.0, help="虚拟经度（默认 120.0）")
    ap.add_argument("--alt", type=float, default=20.0, help="虚拟海拔 m（成为任务高度基准，台架任意）")
    ap.add_argument("--hz", type=float, default=10.0, help="发布频率（默认 10Hz，需高于 mavros 真 NO_FIX 帧率）")
    ap.add_argument("--dur", type=float, default=0.0, help="持续秒数（0=直到 Ctrl-C）")
    ap.add_argument("--status", type=int, default=0, help="注入的 status 值（0=STATUS_FIX；-1=NO_FIX 负向测试）")
    ap.add_argument("--hdg", type=float, default=0.0, help="虚拟罗盘航向°（喂 compass_hdg；室内静止时 mavros 对地航向 65535 无效、任务 have_compass_ 永假，故须代喂）")
    args = ap.parse_args()
    ensure_ros(sys.argv)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import NavSatFix, NavSatStatus
    from std_msgs.msg import Float64 as Float64Msg

    rclpy.init()
    node = Node("virtual_gps_bench")
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=5)
    pub = node.create_publisher(NavSatFix, "/mavros/global_position/global", qos)
    hdg_pub = node.create_publisher(Float64Msg, "/mavros/global_position/compass_hdg", qos)

    def make_msg(seq):
        msg = NavSatFix()
        msg.header.frame_id = "virtual_gps_bench"
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.status.status = args.status
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = args.lat
        msg.longitude = args.lon
        msg.altitude = args.alt
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        msg.position_covariance[0] = msg.position_covariance[4] = 0.01
        msg.position_covariance[8] = 0.04
        return msg

    print(f"[vgps] 虚拟 GPS 启动: status={args.status} lat={args.lat} lon={args.lon} "
          f"alt={args.alt}m hdg={args.hdg}° @{args.hz}Hz → global + compass_hdg")
    print("[vgps] ⚠️ 仅限室内台架/拆桨！观察任务 FSM: WAIT_NAV_STABLE→LOCK_FRAME→PRESTREAM→WAIT_GUIDED→WAIT_ARM")
    print("[vgps] Ctrl-C 停止。")

    import time
    period = 1.0 / max(0.1, args.hz)
    n = 0
    t0 = time.time()
    try:
        while True:
            pub.publish(make_msg(n))
            hdg_pub.publish(Float64Msg(data=args.hdg))
            n += 1
            if n % int(max(1, args.hz * 5)) == 0:
                print(f"[vgps] 已发 {n} 帧（status={args.status}）", flush=True)
            if args.dur > 0 and time.time() - t0 >= args.dur:
                print(f"[vgps] 达到 --dur {args.dur}s，退出")
                break
            time.sleep(period)
    except KeyboardInterrupt:
        print(f"[vgps] Ctrl-C，共发 {n} 帧，退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
