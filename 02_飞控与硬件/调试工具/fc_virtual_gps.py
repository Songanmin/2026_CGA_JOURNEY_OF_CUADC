#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""室内台架虚拟 GPS —— 给任务状态机喂固定 fix + 给 FC 注入位置源（纯台架工具，不碰飞行栈代码）

三种模式（按需叠加，--fc-gps-feed 与 --mavlink-tcp 互斥，fc 模式优先）：
  默认（ROS 叠加）: /mavros/global_position/global 叠 10Hz status=FIX 帧 + compass_hdg 代喂
                    → 任务门禁（GPS+罗盘）室内可过，FSM 走到 WAIT_GUIDED；
  --mavlink-tcp:    经 fc_serial_bridge --multi 以 MAVLink 直发 GPS_INPUT 给 FC；
  --fc-gps-feed:    FC 侧 GPS 注入全套——设 FC GPS1_TYPE=14(GPS_TYPE_MAV) + 开 mavros
                    fake_gps(发 GPS_INPUT) + 喂静态 mocap 位姿 → FC EKF 有位置源 →
                    mavros local_position/odom 流出 → 任务 have_odom_ 满足 → FSM 全链
                    （含 GUIDED/自动解锁/TAKEOFF）室内可走。

⚠️ 安全边界（三条，违反任一立即 Ctrl-C）：
  1. 只在室内台架、拆桨状态使用；
  2. 绝不在室外/真飞/自主飞行时运行——经纬度/海拔全是假的，会污染真实 GPS 判读；
  3. --fc-gps-feed 会改 FC 参数 GPS1_TYPE（台架临时值）——bench 结束务必恢复
     GPS1_TYPE=1（恢复命令见运行时打印）。FSM 解锁后 TAKEOFF 电机研磨无升力属预期，
     90s 看护或遥控切走 GUIDED+上锁收尾。

用法（Jetson 上，非交互 ssh 直用亦可，缺 ROS 环境自动 source 后重执行自身）:
  python3 fc_virtual_gps.py [--fc-gps-feed] [--lat 30.0] [--lon 120.0] [--alt 20.0]
                            [--hz 10] [--dur 0] [--hdg 0.0] [--mavlink-tcp H:P]
PC 端（Windows）入口 = ops 自动 ssh 远跑:
  python ops.py vgps ... / python ops.py bench <23|13|12>
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
    os.execvp("bash", ["bash", "-c", f"{src} && exec {cmd}"])  # -c 非 login：setsid 无 tty 下 profile 会卡死


def main():
    ap = argparse.ArgumentParser(description="室内台架虚拟 GPS（拆桨！）")
    ap.add_argument("--lat", type=float, default=30.0, help="虚拟纬度（默认 30.0，假值勿用于真导航）")
    ap.add_argument("--lon", type=float, default=120.0, help="虚拟经度（默认 120.0）")
    ap.add_argument("--alt", type=float, default=20.0, help="虚拟海拔 m（成为任务高度基准，台架任意）")
    ap.add_argument("--hz", type=float, default=10.0, help="发布频率（默认 10Hz）")
    ap.add_argument("--dur", type=float, default=0.0, help="持续秒数（0=直到 Ctrl-C）")
    ap.add_argument("--status", type=int, default=0, help="注入的 status 值（0=STATUS_FIX；-1=NO_FIX 负向测试）")
    ap.add_argument("--hdg", type=float, default=0.0, help="虚拟罗盘航向°（喂 compass_hdg；室内静止时 mavros 对地航向 65535 无效、任务 have_compass_ 永假，故须代喂）")
    ap.add_argument("--mavlink-tcp", dest="mavlink_tcp", default="",
                    help="MAVLink GPS_INPUT 注入（台架：经 fc_serial_bridge --multi，如 127.0.0.1:14550）；"
                         "FC 需 GPS1_TYPE=14（GPS_TYPE_MAV）。注入后 FC EKF 有位置源，mavros odom 才会流")
    ap.add_argument("--fc-gps-feed", dest="fc_gps_feed", action="store_true",
                    help="FC 侧 GPS 注入全套：设 FC GPS1_TYPE=14 + 开 mavros fake_gps + 喂静态 mocap 位姿"
                         "（EKF 位置源→odom 流→FSM 全链；bench 结束恢复 GPS1_TYPE=1）")
    args = ap.parse_args()
    ensure_ros(sys.argv)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import NavSatFix, NavSatStatus
    from std_msgs.msg import Float64 as Float64Msg
    from geometry_msgs.msg import PoseStamped

    rclpy.init()
    node = Node("virtual_gps_bench")
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=5)

    # ---------- ① FC 侧注入准备（--fc-gps-feed）：GPS1_TYPE=14 + fake_gps 开关 ----------
    import time
    if args.fc_gps_feed:
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
        from rcl_interfaces.srv import SetParameters
        from mavros_msgs.srv import ParamSet as ParamSetSrv

        def mavros_fc_param(name, value_int):
            """写 FC 参数（经 mavros param service；GPS1_TYPE=14=GPS_TYPE_MAV）。"""
            cli = node.create_client(ParamSetSrv, "/mavros/param/set")  # mavros 2.x 服务名（无 set_parameter）
            t0 = time.time()
            while not cli.service_is_ready() and time.time() - t0 < 60:
                time.sleep(1.0)
            if not cli.service_is_ready():
                vis = [s for s, _ in node.get_service_names_and_types()
                       if s.startswith("/mavros/param")]
                print("[vgps] ⚠️ mavros param service 60s 不可见，GPS1_TYPE 未改；"
                      f"图内 /mavros/param/*: {vis[:6] if vis else '无'}")
                return False
            req = ParamSetSrv.Request()
            req.param_id = name
            req.value.integer = value_int
            req.value.real = 0.0
            for attempt in range(3):
                fut = cli.call_async(req)
                rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
                if fut.result() is not None:
                    ok = fut.result().success
                    print(f"[vgps] FC 参数 {name}={value_int}: {'✓' if ok else '✗'}")
                    return ok
                print(f"[vgps] FC 参数 {name}: 第 {attempt+1} 次调用无响应，重试")
                time.sleep(2.0)
            print(f"[vgps] FC 参数 {name}={value_int}: ✗ 三次无响应")
            return False

        mavros_fc_param("GPS1_TYPE", 14)

        pcli = node.create_client(SetParameters, "/mavros/set_parameters")
        pcli.wait_for_service(timeout_sec=15.0)

        def set_mavros_param(name, val, ptype):
            pv = ParameterValue(type=ptype)
            if ptype == ParameterType.PARAMETER_BOOL:
                pv.bool_value = val
            elif ptype == ParameterType.PARAMETER_DOUBLE:
                pv.double_value = val
            req = SetParameters.Request()
            req.parameters = [Parameter(name=name, value=pv)]
            fut = pcli.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
            r = fut.result()
            ok = bool(r and r.results) and r.results[0].successful
            reason = r.results[0].reason if (r and r.results) else "无结果"
            print(f"[vgps] mavros 参数 {name}={val}: {'✓' if ok else '✗ ' + reason}")
            return ok

        set_mavros_param("fake_gps.send_fake_gps", True, ParameterType.PARAMETER_BOOL)
        set_mavros_param("fake_gps.use_hil_gps", False, ParameterType.PARAMETER_BOOL)
        set_mavros_param("fake_gps.gps_rate", float(max(1.0, args.hz)), ParameterType.PARAMETER_DOUBLE)
        set_mavros_param("fake_gps.geo_origin/lat", args.lat, ParameterType.PARAMETER_DOUBLE)
        set_mavros_param("fake_gps.geo_origin/lon", args.lon, ParameterType.PARAMETER_DOUBLE)
        set_mavros_param("fake_gps.geo_origin/alt", args.alt, ParameterType.PARAMETER_DOUBLE)
        print("[vgps] ⚠️ 台架结束务必恢复 FC: GPS1_TYPE=1（--fc-gps-feed 只写临时值）")

    # ---------- ② 话题发布器 ----------
    pub = node.create_publisher(NavSatFix, "/mavros/global_position/global", qos)
    hdg_pub = node.create_publisher(Float64Msg, "/mavros/global_position/compass_hdg", qos)
    pose_pub = None
    if args.fc_gps_feed:
        pose_pub = node.create_publisher(PoseStamped, "/mavros/fake_gps/mocap/pose", qos)

    # ---------- ③ MAVLink 直注入通道（可选，经桥；与 fc-gps-feed 互斥） ----------
    mav_sock = None
    if args.mavlink_tcp and not args.fc_gps_feed:
        import socket as sock_mod
        from pymavlink import mavutil as mu
        host, port = args.mavlink_tcp.rsplit(":", 1)
        mav_sock = sock_mod.create_connection((host, int(port)), timeout=5)
        print(f"[vgps] MAVLink 注入通道: {host}:{port}（GPS_INPUT fix3 @{args.hz}Hz；确认 FC GPS1_TYPE=14）")

    print(f"[vgps] 虚拟 GPS 启动: status={args.status} lat={args.lat} lon={args.lon} "
          f"alt={args.alt}m hdg={args.hdg}° @{args.hz}Hz fc_feed={args.fc_gps_feed}")
    print("[vgps] ⚠️ 仅限室内台架/拆桨！Ctrl-C 停止。")

    import io as io_mod
    import time
    period = 1.0 / max(0.1, args.hz)
    n = 0
    t0 = time.time()

    def send_gps_input_mavlink(frame_ms):
        buf = io_mod.BytesIO()
        mav = mu.mavlink.MAVLink(buf)
        mav.gps_input_send(
            0, 0, 0, int(frame_ms) % 600000, 1234, 3,
            int(args.lat * 1e7), int(args.lon * 1e7), args.alt,
            0.8, 1.2, 0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 10)
        mav_sock.sendall(buf.getvalue())

    try:
        while True:
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
            pub.publish(msg)
            hdg_pub.publish(Float64Msg(data=args.hdg))
            if pose_pub is not None:
                ps = PoseStamped()
                ps.header.stamp = msg.header.stamp
                ps.header.frame_id = "map"
                pose_pub.publish(ps)  # 静态原点位姿：fake_gps 换算 geo_origin 处 fix
            if mav_sock is not None:
                send_gps_input_mavlink(n * (1000 / max(0.1, args.hz)))
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
        if mav_sock is not None:
            mav_sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
