# -*- coding: utf-8 -*-
"""罗盘精细校准（机载六面球面拟合）：基线快照 → DO_START_MAG_CAL → 进度监视
→ 自动保存+自动重启 → 回读参数前后对比 → 模长一致性验证 → 报告落参数备份/

与 Mission Planner「校准罗盘」同一条机载链路（ArduPilot 专用命令
MAV_CMD_DO_START_MAG_CAL；PREFLIGHT_CALIBRATION param2=1 在 ArduPilot 上是
UNSUPPORTED，勿用）。脚本化差异：显式订阅 MAG_CAL_PROGRESS(191)/MAG_CAL_REPORT(192)
——两报文挂在 EXTRA3 流下，流率为 0 时校准会「假挂死」，故启动前先设报文间隔。

流程：
  ① 门禁：必须已解除解锁（disarmed），armed 直接退出；口被 MP 占用直接报错
  ② 校前基线：COMPASS_* 参数 + 各路地磁模长中位数（mG）
  ③ 人肉六面翻转（水平/头下/头上/左下/右下/倒扣），机载端保持静止时采样自动拟合
  ④ 成功自动保存 + 自动重启（--no-reboot 只保存不重启，断电重启后生效）
  ⑤ 链路断开（翻转中 USB 抖动/校准收尾自动重启）不裸崩：重连后自动转回读验证
  ⑥ 回读：OFS/DIA/ODL/SCALE 前后对比 + 模长一致性（>150 mG ⛔，80~850 范围外 ⛔）
  ⑦ 报告落 02_飞控与硬件/参数备份/罗盘校准_时间戳.txt（无论成败都落）

用法: python fc_compass_calib.py [COM口] [--mask 位掩码] [--no-reboot] [--yes]
  --mask 1 只校第 1 路 / --mask 2 只校第 2 路 / 缺省 0=全部（某路反复失败时只校它）
红线: 跑前断开 Mission Planner（占 COM 口）；拆桨；翻转半径 1m 内无铁磁物
     （钢筋地面/桌架/工具箱/钥匙/手机/音箱/电源）；USB 线留够翻转余量
注意: 电机干扰补偿（compassmot，带电流程）不在本脚本范围，需要时走 Mission Planner
"""
import math
import os
import sys
import time
from datetime import datetime

import serial
from pymavlink import mavutil
from serial.tools import list_ports

import verify_params as vp

MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_CMD_DO_START_MAG_CAL = 42424     # p1 罗盘位掩码(0=全部) p2 失败重试 p3 自动保存
                                     # p4 起始延迟(秒) p5 完成后自动重启
MAV_CMD_DO_CANCEL_MAG_CAL = 42426    # p1 罗盘位掩码(0=全部)

MSG_INTERVAL_US = {
    "MAG_CAL_PROGRESS": 200000,      # 5 Hz
    "MAG_CAL_REPORT": 500000,        # 2 Hz
}

CAL_STATUS = {0: "未开始", 1: "等待开始", 2: "进行中", 3: "成功",
               4: "失败", 5: "姿态超差", 6: "半径超差"}

# 罗盘实例 i(0 起) -> 参数名基（ArduPilot 命名：首路无序号，2/3 路带序号）
def pbase(inst, first):
    return first if inst == 0 else f"{first}{inst + 1}"

def comp_params(inst):
    b_dev = pbase(inst, "COMPASS_DEV_ID")
    b_use = pbase(inst, "COMPASS_USE")
    b_ofs = pbase(inst, "COMPASS_OFS")
    b_dia = pbase(inst, "COMPASS_DIA")
    b_odl = pbase(inst, "COMPASS_ODL")
    b_scl = pbase(inst, "COMPASS_SCALE")
    names = [b_dev, b_use]
    for ax in "XYZ":
        names += [f"{b_ofs}_{ax}", f"{b_dia}_{ax}", f"{b_odl}_{ax}"]
    names += [b_scl]
    return names

ALL_COMPASS_PARAMS = sorted({n for i in range(3) for n in comp_params(i)})


class LinkLost(Exception):
    """串口层断开（USB 抖动/被拔/飞控重启重枚举）"""


def pick_port():
    """缺省自动认口：按 README 口径认端口描述不认号（飞控 = "ArduPilot MAVLink"）"""
    cands = [p for p in list_ports.comports() if "ardupilot mavlink" in p.description.lower()]
    if len(cands) == 1:
        return cands[0].device
    if not cands:
        sys.exit("错误：没扫到描述为 \"ArduPilot MAVLink\" 的口——飞控没插/驱动没起，"
                 "或用 python fc_compass_calib.py COM17 手动指定。\n"
                 "当前在位: " + " | ".join(f"{p.device}({p.description})" for p in list_ports.comports()))
    return cands[0].device


def fmt(v):
    if v is None:
        return "无"
    if abs(v - round(v)) < 1e-6 and abs(v) < 1e9:
        return str(int(round(v)))
    return f"{v:.4f}"


def get_param(params, name):
    return params.get(name)


def recv_msg_safe(master):
    try:
        return master.recv_match(blocking=False)
    except (serial.SerialException, OSError) as e:
        raise LinkLost(str(e))


def connect_any():
    """pick_port + vp.connect，带重试；返回 (master, port)"""
    last_err = None
    for attempt in range(5):
        try:
            port = pick_port()
            m = vp.connect(port)
            return m, port
        except SystemExit as e:
            last_err = e
        except Exception as e:  # 口在但打开失败（正在重枚举等）
            last_err = e
        print(f"  重连尝试 {attempt + 1}/5 失败，8 秒后重试...（{last_err}）")
        time.sleep(8)
    sys.exit(f"错误：连不回飞控——手动重插 USB 后跑 prearm 复查（{last_err}）")


def resubscribe_cal_msgs(master):
    for t, us in MSG_INTERVAL_US.items():
        mid = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{t}", None)
        if mid is not None:
            master.mav.command_long_send(
                master.target_system, master.target_component,
                MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, us, 0, 0, 0, 0, 0)


def snapshot_fields(master, seconds=3.0):
    """订阅 SCALED_IMU/2/3 并采 seconds 秒，返回各路地磁模长中位数（mG）"""
    types = ["SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3"]
    for t in types:
        mid = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{t}")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, 500000, 0, 0, 0, 0, 0)
    # 清空残留报文
    t0 = time.time()
    while time.time() - t0 < 0.5:
        master.recv_match(blocking=False)
        time.sleep(0.02)
    samples = {t: [] for t in types}
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = master.recv_match(blocking=False)
        if msg is None:
            time.sleep(0.02)
            continue
        t = msg.get_type()
        if t in samples:
            mag = math.sqrt(msg.xmag ** 2 + msg.ymag ** 2 + msg.zmag ** 2)
            if mag > 0:
                samples[t].append(mag)
    out = {}
    for i, t in enumerate(types):
        arr = samples[t]
        out[i] = sorted(arr)[len(arr) // 2] if arr else 0.0
    return out


def print_baseline(params, fields):
    print("\n=== 校前基线 ===")
    for i in range(3):
        dev = get_param(params, pbase(i, "COMPASS_DEV_ID"))
        use = get_param(params, pbase(i, "COMPASS_USE"))
        if dev is None or (isinstance(dev, float) and dev == 0):
            continue
        b_ofs = pbase(i, "COMPASS_OFS")
        b_dia = pbase(i, "COMPASS_DIA")
        b_odl = pbase(i, "COMPASS_ODL")
        b_scl = pbase(i, "COMPASS_SCALE")
        ofs = tuple(get_param(params, f"{b_ofs}_{ax}") for ax in "XYZ")
        dia = tuple(get_param(params, f"{b_dia}_{ax}") for ax in "XYZ")
        odl = tuple(get_param(params, f"{b_odl}_{ax}") for ax in "XYZ")
        scl = get_param(params, b_scl)
        live = fields.get(i, 0)
        print(f"  罗盘{i + 1}: DEV_ID={fmt(dev)} USE={fmt(use)} 实测模长={live:.0f} mG")
        print(f"    OFS=({', '.join(fmt(v) for v in ofs)}) mG")
        print(f"    DIA=({', '.join(fmt(v) for v in dia)})")
        print(f"    ODL=({', '.join(fmt(v) for v in odl)})")
        print(f"    SCALE={fmt(scl)}")


def _covered(reports, mask):
    if mask is None:
        return False
    need = [i for i in range(8) if mask & (1 << i)]
    if not need:
        return False
    return all(i in reports for i in need)


def _all_success(reports):
    return bool(reports) and all(r.cal_status == 3 for r in reports.values())


def _cancel(master):
    try:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            MAV_CMD_DO_CANCEL_MAG_CAL, 0, 0, 0, 0, 0, 0, 0, 0)
    except Exception:
        pass


def main():
    argv = sys.argv[1:]
    no_reboot = "--no-reboot" in argv
    assume_yes = "--yes" in argv
    mask = 0
    if "--mask" in argv:
        i = argv.index("--mask")
        mask = int(argv[i + 1])
    # 位置参数=COM 口；--mask 的取值不算位置参数
    pos = [a for a in argv if not a.startswith("--")]
    if "--mask" in argv:
        mi = argv.index("--mask")
        pos = [a for n, a in enumerate(argv) if not a.startswith("--") and n != mi + 1]
    port = pos[0] if pos else pick_port()

    master = vp.connect(port)
    hb = master.messages.get("HEARTBEAT")  # wait_heartbeat 已在 connect 内完成并缓存

    # ---- 门禁：必须未解锁 ----
    armed = bool(hb.base_mode & 128) if hb else True
    if armed:
        sys.exit("⛔ 飞控当前已解锁（armed）——罗盘校准必须在未解锁状态下进行，退出")
    print("已确认：未解锁 ✓")

    # ---- 校前基线 ----
    print("下载全参数（基线）...")
    before = vp.download_all_params(master)
    fields_before = snapshot_fields(master, 3.0)
    print_baseline(before, fields_before)

    # ---- 现场提醒 + 确认 ----
    print("\n" + "=" * 62)
    print("翻转前检查（逐条确认）：")
    print("  1. 桨已拆（台架纪律）")
    print("  2. 翻转半径 1m 内无铁磁物：钢筋地面/桌架/工具箱/钥匙/")
    print("     手机/音箱/电源/带磁螺丝刀——这是校准质量的第一影响因素")
    print("  3. USB 线留够翻转余量，笔记本推远（USB 口被翻转拉扯易瞬断）")
    print("  4. 六面翻转（顺序不限，覆盖即可）：")
    print("     ①水平正放 ②机头朝下 ③机头朝上")
    print("     ④左侧朝下 ⑤右侧朝下 ⑥倒扣（机腹朝天）")
    print("     缓慢匀速，每面停 3~5 秒，多循环几轮直到 100%")
    print("=" * 62)
    if not assume_yes:
        input("确认后回车开始校准（5 秒后机载端启动采样）...")

    # ---- 订阅校准报文 + 启动 ----
    resubscribe_cal_msgs(master)
    time.sleep(0.3)
    print(f"发送 DO_START_MAG_CAL（位掩码 {mask}，失败重试，自动保存，"
          + ("完成后自动重启）..." if not no_reboot else "不自动重启）..."))
    master.mav.command_long_send(
        master.target_system, master.target_component,
        MAV_CMD_DO_START_MAG_CAL, 0,
        mask,   # p1 位掩码 0=全部
        1,      # p2 失败自动重试
        1,      # p3 自动保存（成功即写参数）
        5,      # p4 起始延迟 5 秒
        0 if no_reboot else 1,  # p5 完成后自动重启
        0, 0)   # p6/p7 保留
    try:
        ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    except (serial.SerialException, OSError) as e:
        print(f"\n⚠ 启动瞬间链路断开: {e}——重连后取消，请直接重跑")
        master, port = connect_any()
        _cancel(master)
        sys.exit(1)
    if ack is None:
        sys.exit("错误：未收到 START_MAG_CAL 的 COMMAND_ACK")
    result = ack.result
    renum = mavutil.mavlink.enums.get("MAV_RESULT", {})
    result_name = renum[result].name if result in renum else str(result)
    if result != 0:  # MAV_RESULT_ACCEPTED
        print(f"⛔ START_MAG_CAL 被拒绝: result={result} ({result_name})")
        if result == 4:
            print("  UNSUPPORTED：确认固件版本（4.7-beta 应支持 DO_START_MAG_CAL）")
        sys.exit(1)
    print("机载端已接受，等待报文...\n")

    # ---- 进度监视（断链自动重连：仍在校准则继续盯，已结束则转回读）----
    progress = {}        # compass_id -> 最新 pct
    reports = {}         # compass_id -> 最终 report
    cal_mask = None if mask == 0 else mask
    last_change = time.time()
    t_start = time.time()
    link_drops = 0
    finished_by_link_loss = False

    def handle_msg(msg):
        nonlocal cal_mask, last_change
        t = msg.get_type()
        if t == "MAG_CAL_PROGRESS":
            cid = msg.compass_id
            cal_mask = msg.cal_mask
            pct = msg.completion_pct
            if progress.get(cid) != pct:
                progress[cid] = pct
                last_change = time.time()
                print("  进度: " + " | ".join(
                    f"罗盘{c + 1} {p}%" for c, p in sorted(progress.items())))
        elif t == "MAG_CAL_REPORT":
            cid = msg.compass_id
            cal_mask = msg.cal_mask or cal_mask
            status = CAL_STATUS.get(msg.cal_status, msg.cal_status)
            saved = getattr(msg, "saved", None)
            fit = getattr(msg, "fitness", None)
            print(f"  ■ 罗盘{cid + 1} 结果: {status} "
                  + (f"saved={saved} " if saved is not None else "")
                  + (f"拟合残差={fit:.1f}" if fit is not None else ""))
            if cid not in reports or msg.cal_status != 2:
                reports[cid] = msg
            last_change = time.time()
        elif t == "STATUSTEXT":
            text = msg.text.decode("utf-8", "replace") if isinstance(msg.text, bytes) else msg.text
            if "ompass" in text:
                print(f"  [飞控] {text}")
        elif t == "HEARTBEAT":
            last_change = time.time()

    try:
        while True:
            if cal_mask is not None and _covered(reports, cal_mask):
                break
            try:
                msg = master.recv_match(blocking=False)
            except (serial.SerialException, OSError) as e:
                link_drops += 1
                print(f"\n⚠ 串口断开（第 {link_drops} 次）: {e}")
                print("  常见原因：翻转拉扯 USB 线/口瞬断、校准收尾飞控自动重启重枚举。"
                      "校准在机载端不受影响，重连跟进...")
                try:
                    master.close()
                except Exception:
                    pass
                master, port = connect_any()
                resubscribe_cal_msgs(master)
                # 听 8 秒：还有进行中报文=校准仍在跑，继续盯；没有=已结束（或已重启完成）
                alive = False
                t_listen = time.time()
                while time.time() - t_listen < 8:
                    try:
                        m2 = master.recv_match(blocking=False)
                    except (serial.SerialException, OSError):
                        alive = None  # 又断了，外层重连
                        break
                    if m2 is None:
                        time.sleep(0.1)
                        continue
                    t2 = m2.get_type()
                    if t2 == "MAG_CAL_PROGRESS":
                        alive = True
                        handle_msg(m2)
                    elif t2 in ("MAG_CAL_REPORT", "HEARTBEAT"):
                        handle_msg(m2)
                if alive is None:
                    continue
                last_change = time.time()
                if alive:
                    print("  校准仍在进行，继续监视（接着翻转！）")
                else:
                    print("  重连后无进行中报文——校准应已在机载端结束，转回读验证")
                    finished_by_link_loss = True
                    break
                continue

            if msg is None:
                time.sleep(0.1)
                if time.time() - last_change > 300:
                    print("\n⛔ 5 分钟无进度——取消校准（干扰过大或某路反复失败）")
                    _cancel(master)
                    break
                continue
            handle_msg(msg)
    except KeyboardInterrupt:
        print("\n用户中断——发送取消命令...")
        _cancel(master)

    # ---- 结果判定 ----
    ok_all = _all_success(reports)
    if reports:
        print("\n=== 校准结果 ===")
        for cid, r in sorted(reports.items()):
            print(f"  罗盘{cid + 1}: {CAL_STATUS.get(r.cal_status, r.cal_status)}")
    if not ok_all:
        # 失败路可能还在机载端无限重试，明确取消再回读
        _cancel(master)
        if reports:
            print("⛔ 存在未成功的罗盘（拟合残差大=干扰/翻转太快/覆盖不足）。")
            print("   已发取消命令停掉机载端重试。建议：")
            print("   ①换场地/离铁磁物更远/翻转更慢、每面停更久后重跑；")
            print("   ②只校失败路省时间：python ops.py compass --mask <失败路号位掩码>"
                  "（第 2 路=2）")

    # ---- 等待自动重启 + 重连（仅全部成功且要求重启时）----
    if ok_all and not no_reboot:
        print("\n等待飞控自动重启...")
        try:
            master.close()
        except Exception:
            pass
        time.sleep(15)
        master, port = connect_any()
    elif finished_by_link_loss and not no_reboot:
        print("\n链路曾断开——若校准收尾触发了自动重启，重连已在上面完成；"
              "若拿不准，回读对比会揭示参数是否已更新。")

    # ---- 校后验证 ----
    print("\n回读校后参数与模长...")
    time.sleep(5)  # 等传感器/EKF 稳定
    after = None
    for attempt in range(2):
        try:
            after = vp.download_all_params(master)
            fields_after = snapshot_fields(master, 3.0)
            break
        except (serial.SerialException, OSError):
            print("  回读时链路又断，重连后重试...")
            master, port = connect_any()
    if after is None:
        sys.exit("错误：回读两次均遇链路断开——手动重插 USB 后单独跑 prearm 复查")

    print("\n=== 前后对比（仅列出有变化的项）===")
    diffs = []
    for i in range(3):
        dev = get_param(after, pbase(i, "COMPASS_DEV_ID"))
        if dev is None or (isinstance(dev, float) and dev == 0):
            continue
        for name in comp_params(i):
            b, a = get_param(before, name), get_param(after, name)
            if b is None and a is None:
                continue
            if b is None or a is None or abs((a or 0) - (b or 0)) > 1e-6:
                print(f"  罗盘{i + 1} {name}: {fmt(b)} → {fmt(a)}")
                diffs.append((i + 1, name, b, a))
    if not diffs:
        print("  （无变化——若上面有「成功」终报，说明保存未生效，需重跑）")

    # 模长一致性（对齐 prearm_diag 口径）
    print("\n=== 校后模长验证 ===")
    live = {}
    for i in range(3):
        dev = get_param(after, pbase(i, "COMPASS_DEV_ID"))
        if dev is None or (isinstance(dev, float) and dev == 0):
            continue
        m = fields_after.get(i, 0)
        live[i] = m
        rng = "✓" if 80 < m < 850 else "⛔ 范围超差(80~850)"
        print(f"  罗盘{i + 1}: 实测模长 {m:.0f} mG {rng}")
    if len(live) >= 2:
        vals = list(live.values())
        diff = max(vals) - min(vals)
        ok = diff <= 150
        print(f"  多路一致性: 最大差 {diff:.0f} mG {'✓' if ok else '⛔ 超差(>150)'}")

    # ---- 报告落档（无论成败）----
    ts = datetime.now().strftime("%Y%m%d_%H%M")
    out_dir = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "参数备份"))
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f"罗盘校准_{ts}.txt")
    with open(path, "w", encoding="utf-8-sig") as f:
        f.write(f"罗盘精细校准报告  {datetime.now().strftime('%Y-%m-%d %H:%M')}\n")
        f.write(f"端口: {port}  位掩码: {mask}  用时: {time.time() - t_start:.0f}s"
                f"  链路中断: {link_drops} 次\n")
        f.write("\n== 校前 COMPASS 参数 ==\n")
        for name in ALL_COMPASS_PARAMS:
            v = get_param(before, name)
            if v is not None:
                f.write(f"{name} = {v:g}\n")
        f.write("\n== 校后 COMPASS 参数 ==\n")
        for name in ALL_COMPASS_PARAMS:
            v = get_param(after, name)
            if v is not None:
                f.write(f"{name} = {v:g}\n")
        f.write("\n== 变化项 ==\n")
        if diffs:
            for c, name, b, a in diffs:
                f.write(f"罗盘{c} {name}: {fmt(b)} -> {fmt(a)}\n")
        else:
            f.write("（无）\n")
        f.write("\n== 校前模长 (mG) ==\n")
        for i, m in sorted(fields_before.items()):
            f.write(f"罗盘{i + 1}: {m:.0f}\n")
        f.write("\n== 校后模长 (mG) ==\n")
        for i, m in sorted(fields_after.items()):
            f.write(f"罗盘{i + 1}: {m:.0f}\n")
        f.write("\n== 终报 ==\n")
        if reports:
            for cid, r in sorted(reports.items()):
                f.write(f"罗盘{cid + 1}: status={CAL_STATUS.get(r.cal_status, r.cal_status)}"
                        f" fitness={getattr(r, 'fitness', None)}"
                        f" saved={getattr(r, 'saved', None)}\n")
        else:
            f.write("（无终报——校准未走到出报阶段或链路中断吞没）\n")
    print(f"\n报告已落档: {path}")
    verdict = "✓ 全部成功" if ok_all else "⛔ 存在失败路（见上）"
    print(f"结论: {verdict}")
    try:
        master.close()
    except Exception:
        pass
    sys.exit(0 if ok_all else 1)


if __name__ == "__main__":
    main()
