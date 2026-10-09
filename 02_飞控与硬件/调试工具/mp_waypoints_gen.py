# -*- coding: utf-8 -*-
"""MP 备份救援航线生成器（CUADC 2026 · Mission Planner .waypoints，QGC WPL 110）

作用：
  把任务剖面（起飞 2.5m → 投放区中心悬停识别 → 搜索蛇形兜底 12 点 →
  侦察蛇形 6 点 → 返航 → 降落）按赛场起飞点经纬度 + 起飞罗盘航向，
  生成为 Mission Planner 可加载的 AUTO 航线文件（QGC WPL 110）。

定位（务必理解再用）：
  - 自主任务主通道 = 机上 cuadc_mission FSM（GUIDED offboard），本航线是
    「任务软件失效时的救援兜底」：机上电脑不可用时飞手在 MP 切 AUTO
    解锁，飞机沿同款航线完成搜索-侦察-返航，全程人工监控；
  - 因此本航线**不含投放/拍照/视觉对准等任何载荷动作**，也没有
    LOITER 之外的等待逻辑——纯航线剖面；
  - 解锁权永远在飞手（仓库红线 2）：加载文件≠授权起飞。⚠️ AUTO 模式
    解锁即自主起飞，加载后请勿在人员未清场时误触解锁。

几何母本（防两处漂移）：
  03_机载软件/cuadc_mission/config/mission_params.yaml（自动直读；
  找不到时回退本文件 CONFIG 默认值并明确警告）。赛场实测修订几何
  只改 yaml，本文件零改动、重跑即可。本工具刻意不提供几何命令行
  覆盖——几何 SSOT 在 yaml。

坐标红线（与 site_geo_check.py 同款纪律）：
  - 只接受 WGS-84 经纬度（MP HUD / `ops.py geo here` 输出）；
    严禁高德/百度 GCJ-02 坐标（偏移约 500m）；
  - --hdg = 起飞方向罗盘航向（0=北、顺时针；机头对准任务前进方向时
    读 MP HUD YAW）。场地系 x=起飞时机头方向、y=左，与任务代码
    route_logic.hpp 同款（自检 --selftest 含其回归锚点）。

⚠️ 贴带飞提醒：FC 的 WP_RADIUS_M 参数基线=2.0m，而搜索带间距 1.6m——
  载入备份航线前建议把 WP_RADIUS_M 调到 ≤0.8，否则切角早转、带形失真
  （备份航线不做精确投放，只求覆盖完整）。

用法：
  python mp_waypoints_gen.py --selftest                # 回归锚点自检（不写文件）
  python mp_waypoints_gen.py --lat 29.3765 --lon 104.6258 --hdg 0 [--alt 345]
      [--loiter 12] [--no-snake] [--no-hover] [--out 文件] [--print-only]
  python ops.py mpwp --lat ... --lon ...               # 统一入口
"""

import argparse
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

# ==================== CONFIG：yaml 缺席时的回退默认值 ====================
# 母本 = 03_机载软件/cuadc_mission/config/mission_params.yaml（2026-10-09 同步）
DEFAULTS = {
    "takeoff_alt_m": 2.5,
    "search_alt_m": 2.5,
    "recon_alt_m": 2.5,
    "return_alt_m": 2.5,
    "transit_speed_m_s": 4.0,
    "search_speed_m_s": 2.0,
    "recon_speed_m_s": 3.0,
    "waypoint_accept_radius_m": 0.40,
    "search_x_min_m": 27.5,
    "search_x_max_m": 32.5,
    "search_half_width_m": 4.0,
    "search_lanes": 6,
    "search_hover_x_m": 30.0,
    "search_hover_y_m": 0.0,
    "search_hover_timeout_s": 12.0,
    "recon_x_m": 52.5,
    "recon_lane_len_m": 5.0,
    "recon_half_width_m": 4.0,
    "recon_lanes": 3,
    "mission_timeout_s": 240.0,
}

PARAMS_REL = os.path.join(
    "..", "..", "03_机载软件", "cuadc_mission", "config", "mission_params.yaml")

# 降落速度：FC LAND_SPD_MS=0.5（V6X_ardupilot_params.param），仅用于时长预算
LAND_SPEED_M_S = 0.5
# 起飞爬升+稳定、落地确认的粗估（秒），仅用于时长预算
TAKEOFF_EST_S = 5.0
LAND_CONFIRM_EST_S = 6.0


def load_params(path):
    """极简解析 mission_params.yaml 的四层缩进数值参数；失败回退 DEFAULTS。

    只取「4 空格缩进键: 数字」行（ros__parameters 层），列表/字符串/注释
    一律忽略——生成器需要的键全是标量。
    """
    vals = dict(DEFAULTS)
    try:
        with open(path, encoding="utf-8") as f:
            for line in f:
                m = re.match(r"^    ([a-z0-9_]+):\s*([^#]+?)\s*$", line)
                if not m:
                    continue
                raw = m.group(2)
                if raw.startswith("["):
                    continue
                try:
                    vals[m.group(1)] = float(raw) if "." in raw else int(raw)
                except ValueError:
                    continue
        print(f"[参数] 几何母本已直读: {os.path.normpath(path)}")
    except OSError as e:
        print(f"[警告] 读不到母本 yaml（{e}），回退本文件 CONFIG 默认值——"
              f"若赛场修订过几何请人工核对！")
    return vals


# ==================== 坐标换算（route_logic.hpp 的 Python 镜像 + 大地线放大） ====

def field_to_enu(compass_hdg_deg, fx, fy):
    """场地系（x=机头，y=左）→ 本地 ENU（东、北），米。

    回归锚点（route_logic.hpp 文档）：hdg=0（机头朝北）时
    field(1,0)→北(0,1)；field(0,1)→西(-1,0)。
    """
    h = math.radians(compass_hdg_deg)
    return (math.sin(h) * fx - math.cos(h) * fy,
            math.cos(h) * fx + math.sin(h) * fy)


def enu_to_latlon(lat0, lon0, east, north):
    """本地平面（米）→ WGS-84 经纬度（平面近似，子午/卯酉圈弧长级数，
    55m 赛场尺度误差毫米级）。"""
    p = math.radians(lat0)
    m_lat = 111132.92 - 559.82 * math.cos(2 * p) + 1.175 * math.cos(4 * p)
    m_lon = 111412.84 * math.cos(p) - 93.5 * math.cos(3 * p) + 0.118 * math.cos(5 * p)
    return (lat0 + north / m_lat, lon0 + east / m_lon)


def build_serpentine(x0, x1, y0, y1, lanes, along_x):
    """弓字形蛇形（route_logic.hpp::build_serpentine 镜像，逐行对照实现）。

    along_x=True ：带沿 x 扫（x0↔x1），带位沿 y 均布（搜索区现状）；
    along_x=False：带沿 y 扫（y0↔y1），带位沿 x 均布（侦察区）；
    奇偶带交替方向，返回 2×lanes 个按飞行顺序排列的 (x, y)。
    """
    route = []
    if lanes < 1:
        return route
    for lane in range(lanes):
        t = 0.5 if lanes == 1 else lane / (lanes - 1)
        forward = lane % 2 == 0
        if along_x:
            y = y0 + (y1 - y0) * t
            route.append((x0 if forward else x1, y))
            route.append((x1 if forward else x0, y))
        else:
            x = x0 + (x1 - x0) * t
            route.append((x, y0 if forward else y1))
            route.append((x, y1 if forward else y0))
    return route


# ==================== 航线拼装 ====================

def build_profile(p, loiter_s, with_hover=True, with_snake=True):
    """任务剖面 → 场地系航点序列 [(场地x, 场地y, 相对高, 段名), ...]。

    段名仅用于打印/时长预算；顺序与 FSM 的 SEARCH→RECON→RETURN 对齐：
    悬停识别（12s 窗）→ 搜索蛇形兜底 → 侦察蛇形 → 返航。
    """
    pts = []
    if with_hover:
        pts.append((p["search_hover_x_m"], p["search_hover_y_m"],
                    p["search_alt_m"], "悬停识别"))
    if with_snake:
        for i, (x, y) in enumerate(build_serpentine(
                min(p["search_x_min_m"], p["search_x_max_m"]),
                max(p["search_x_min_m"], p["search_x_max_m"]),
                -p["search_half_width_m"], p["search_half_width_m"],
                int(p["search_lanes"]), True)):
            pts.append((x, y, p["search_alt_m"], f"搜索{i + 1:02d}"))
    for i, (x, y) in enumerate(build_serpentine(
            p["recon_x_m"], p["recon_x_m"] + p["recon_lane_len_m"],
            -p["recon_half_width_m"], p["recon_half_width_m"],
            int(p["recon_lanes"]), False)):
        pts.append((x, y, p["recon_alt_m"], f"侦察{i + 1:02d}"))
    pts.append((0.0, 0.0, p["return_alt_m"], "返航点"))
    return pts


def build_items(lat0, lon0, alt0, hdg, p, loiter_s, accept_r,
                with_hover=True, with_snake=True):
    """剖面 → QGC WPL 110 行列表。

    帧：home=0(GLOBAL, AMSL)；航点=3(GLOBAL_RELATIVE_ALT)；DO 命令=2。
    航点 param2=接受半径（0.40m，与 FSM waypoint_accept_radius_m 同口径）；
    速度变化用 DO_CHANGE_SPEED(178)，逐段与 FSM 速度同口径（转场/搜索/
    侦察/返航），速度不变化处不发指令。
    """
    def xy(fx, fy):
        e, n = field_to_enu(hdg, fx, fy)
        return enu_to_latlon(lat0, lon0, e, n)

    def spd(v):
        items.append(("SPD", 2, 178, [1, v, -1, 0, 0, 0, 0]))

    def wp(name, fx, fy, fz):
        lat, lon = xy(fx, fy)
        items.append((name, 3, 16, [0, accept_r, 0, 0, lat, lon, fz]))

    items = [("HOME", 0, 16, [0, 0, 0, 0, lat0, lon0, alt0])]
    items.append(("TAKEOFF", 3, 22, [0, 0, 0, 0, 0, 0, p["takeoff_alt_m"]]))
    spd(p["transit_speed_m_s"])

    if with_hover:
        hx, hy = p["search_hover_x_m"], p["search_hover_y_m"]
        lat, lon = xy(hx, hy)
        items.append(("悬停识别", 3, 16,
                      [0, accept_r, 0, 0, lat, lon, p["search_alt_m"]]))
        items.append(("悬停窗", 3, 19,
                      [loiter_s, 0, 0, 0, lat, lon, p["search_alt_m"]]))

    cur_speed = p["transit_speed_m_s"]
    for fx, fy, fz, name in build_profile(p, loiter_s, with_hover, with_snake):
        if name == "悬停识别":
            continue  # 已作为 WP+LOITER 发出，这里只负责它之后的段
        if name.startswith("搜索"):
            want = p["search_speed_m_s"]
        elif name.startswith("侦察"):
            want = p["recon_speed_m_s"]
        else:  # 返航点
            want = p["transit_speed_m_s"]
        if want != cur_speed:
            spd(want)
            cur_speed = want
        wp(name, fx, fy, fz)

    items.append(("LAND", 3, 21, [0, 0, 0, 0, 0, 0, 0]))
    return items


def format_file(items):
    lines = ["QGC WPL 110"]
    for seq, (_name, frame, cmd, params) in enumerate(items):
        cells = [seq, 1 if seq == 0 else 0, frame, cmd]
        # param5/6 是经纬度：必须 6 位以上小数（%.8g 会截成 8 位有效数字，
        # 经度方向损失 ~1m）；其余参数 %g 足够
        cells += [f"{v:.8f}" if i in (4, 5) else f"{v:g}"
                  for i, v in enumerate(params)]
        lines.append("\t".join(str(c) for c in cells) + "\t1")
    return "\n".join(lines) + "\n"


def estimate_seconds(p, loiter_s, with_hover=True, with_snake=True):
    """时长预算（与 SSOT §5.5 同口径的粗估：距离/速度 + 固定项）。"""
    pts = build_profile(p, loiter_s, with_hover, with_snake)
    speed_of = {"悬停识别": p["transit_speed_m_s"], "返航点": p["transit_speed_m_s"]}
    total = TAKEOFF_EST_S + LAND_CONFIRM_EST_S + loiter_s
    prev = (0.0, 0.0)
    for x, y, _z, name in pts:
        d = math.hypot(x - prev[0], y - prev[1])
        v = speed_of.get(name) if name in speed_of else (
            p["search_speed_m_s"] if name.startswith("搜索")
            else p["recon_speed_m_s"] if name.startswith("侦察")
            else p["transit_speed_m_s"])
        total += d / max(v, 0.1)
        prev = (x, y)
    return total


def print_table(items, lat0, lon0, hdg):
    print(f"\n{'seq':>3}  {'cmd':>3} {'名称':<6}  {'场地x':>6} {'场地y':>6}"
          f"  {'相对高':>5}  {'lat':>12} {'lon':>12}")
    for seq, (name, frame, cmd, prm) in enumerate(items):
        if cmd in (178, 22, 21):  # 速度/起飞/降落：无坐标，只打印关键参数
            extra = (f"→ 速度 {prm[1]:.1f} m/s" if cmd == 178
                     else f"→ 相对爬升 {prm[6]:.1f} m" if cmd == 22 else "→ 落地")
            print(f"{seq:>3}  {cmd:>3} {name:<6}  {'':>6} {'':>6}"
                  f"  {'':>5}  {'':>12} {'':>12}  {extra}")
            continue
        lat, lon, rel = prm[4], prm[5], prm[6]
        e, n = _ll_to_enu(lat0, lon0, lat, lon)
        fx, fy = _enu_to_field(hdg, e, n)
        rel_str = f"{rel:>5.2f}" if seq != 0 else " 海拔"
        print(f"{seq:>3}  {cmd:>3} {name:<6}  {fx:>6.1f} {fy:>6.1f}"
              f"  {rel_str}  {lat:>12.8f} {lon:>12.8f}")
    print(f"共 {len(items)} 项（含 HOME）")


def _ll_to_enu(lat0, lon0, lat, lon):
    p = math.radians(lat0)
    m_lat = 111132.92 - 559.82 * math.cos(2 * p) + 1.175 * math.cos(4 * p)
    m_lon = 111412.84 * math.cos(p) - 93.5 * math.cos(3 * p) + 0.118 * math.cos(5 * p)
    return ((lon - lon0) * m_lon, (lat - lat0) * m_lat)


def _enu_to_field(hdg, east, north):
    h = math.radians(hdg)
    return (math.sin(h) * east + math.cos(h) * north,
            -math.cos(h) * east + math.sin(h) * north)


# ==================== 自检（route_logic.hpp 回归锚点 + 剖面守门） ====================

def selftest():
    ok = True

    def chk(name, cond):
        nonlocal ok
        print(f"  [{'PASS' if cond else 'FAIL'}] {name}")
        ok = ok and cond

    print("== ENU 换算回归锚点（route_logic.hpp 文档口径）==")
    e, n = field_to_enu(0.0, 1.0, 0.0)
    chk("hdg=0: field(1,0)→北", abs(e) < 1e-9 and abs(n - 1.0) < 1e-9)
    e, n = field_to_enu(0.0, 0.0, 1.0)
    chk("hdg=0: field(0,1)→西", abs(e + 1.0) < 1e-9 and abs(n) < 1e-9)
    e, n = field_to_enu(90.0, 1.0, 0.0)
    chk("hdg=90: field(1,0)→东", abs(e - 1.0) < 1e-9 and abs(n) < 1e-9)

    print("== 经纬度往返（自贡场地量级）==")
    lat, lon = enu_to_latlon(29.3765, 104.6258, 55.0, 0.0)
    e2, n2 = _ll_to_enu(29.3765, 104.6258, lat, lon)
    chk("东向 55m 往返误差 <1cm", abs(e2 - 55.0) < 0.01 and abs(n2) < 0.01)

    print("== 蛇形航线（build_serpentine 镜像）==")
    s = build_serpentine(27.5, 32.5, -4.0, 4.0, 6, True)
    chk("搜索 12 点", len(s) == 12)
    chk("搜索首带 y=-4 起 x0→x1", s[0] == (27.5, -4.0) and s[1] == (32.5, -4.0))
    ys = [round(s[2 * i][1], 3) for i in range(6)]
    chk("搜索带位 = -4,-2.4,-0.8,0.8,2.4,4", ys == [-4.0, -2.4, -0.8, 0.8, 2.4, 4.0])
    r = build_serpentine(52.5, 57.5, -4.0, 4.0, 3, False)
    chk("侦察 6 点", len(r) == 6)
    chk("侦察蛇形顺序", r == [(52.5, -4), (52.5, 4), (55.0, 4),
                              (55.0, -4), (57.5, -4), (57.5, 4)])

    print("== 剖面守门 ==")
    p = dict(DEFAULTS)
    items = build_items(29.3765, 104.6258, 345.0, 0.0, p,
                        p["search_hover_timeout_s"],
                        p["waypoint_accept_radius_m"])
    chk("全剖面 28 项（含 HOME）", len(items) == 28)
    cmds = [it[2] for it in items]
    chk("首项 TAKEOFF(22) 末项 LAND(21)", cmds[1] == 22 and cmds[-1] == 21)
    chk("速度指令 4 条（4/2/3/4）",
        [prm[1] for it in items if it[2] == 178 for prm in [it[3]]] ==
        [4.0, 2.0, 3.0, 4.0])
    t = estimate_seconds(p, p["search_hover_timeout_s"])
    chk(f"时长预算 {t:.0f}s ≤ 0.9×240s", t <= 0.9 * p["mission_timeout_s"])
    half = p["search_alt_m"] * math.tan(1.5 / 2) * (480 / 848)
    chk(f"2.5m 横向半覆盖 {half:.2f}m ≥ 搜索带距半宽 0.8m", half >= 0.8)
    chk("侦察带距半宽 1.25m ≤ 半覆盖", half >= 1.25)
    print(f"\n自检{'全部通过' if ok else '存在失败项'}")
    return 0 if ok else 1


# ==================== 主流程 ====================

def main():
    ap = argparse.ArgumentParser(
        description="MP 备份救援航线生成器（QGC WPL 110；任务剖面同款）")
    ap.add_argument("--selftest", action="store_true", help="回归自检（不写文件）")
    ap.add_argument("--params", default=PARAMS_REL, help="母本 yaml 路径")
    ap.add_argument("--lat", type=float, help="起飞点 WGS-84 纬度（必填）")
    ap.add_argument("--lon", type=float, help="起飞点 WGS-84 经度（必填）")
    ap.add_argument("--alt", type=float, default=None,
                    help="起飞点海拔 AMSL m（默认占位 345=自贡机场标高，建议实测）")
    ap.add_argument("--hdg", type=float, default=0.0,
                    help="起飞方向罗盘航向 deg（0=北顺时针；MP HUD YAW）")
    ap.add_argument("--loiter", type=float, default=None,
                    help="悬停识别窗秒数（默认取母本 search_hover_timeout_s）")
    ap.add_argument("--accept-radius", type=float, default=None,
                    help="航点接受半径 m（默认取母本 waypoint_accept_radius_m）")
    ap.add_argument("--no-snake", action="store_true",
                    help="跳过搜索蛇形兜底（保留悬停识别）")
    ap.add_argument("--no-hover", action="store_true",
                    help="跳过悬停识别+搜索蛇形（救急直奔侦察段）")
    ap.add_argument("--out", default="mp_backup_profile.waypoints",
                    help="输出文件（默认当前目录 mp_backup_profile.waypoints）")
    ap.add_argument("--print-only", action="store_true", help="只打印不写文件")
    args = ap.parse_args()

    if args.selftest:
        sys.exit(selftest())

    if args.lat is None or args.lon is None:
        ap.error("必须给 --lat --lon（WGS-84，MP HUD 或 ops.py geo here 的读数）")
    if abs(args.lat) < 1.0 or abs(args.lon) < 1.0:
        sys.exit("[拒绝] 经纬度过小，疑似占位值/度分混淆——请核对 WGS-84 十进制度")
    if abs(args.lat) > 90 or abs(args.lon) > 180:
        sys.exit("[拒绝] 经纬度超出范围")

    print("=" * 72)
    print("⚠️ 这是「任务软件失效」的救援兜底航线：不含投放/拍照/视觉动作；")
    print("   AUTO 模式解锁即自主起飞——加载≠授权，解锁权在飞手（仓库红线 2）；")
    print("   只接受 WGS-84 坐标，严禁高德/百度 GCJ-02（偏移 ~500m）。")
    print("=" * 72)

    p = load_params(args.params if os.path.isabs(args.params)
                    else os.path.join(HERE, args.params))
    loiter = args.loiter if args.loiter is not None \
        else p["search_hover_timeout_s"]
    accept_r = args.accept_radius if args.accept_radius is not None \
        else p["waypoint_accept_radius_m"]
    alt0 = args.alt if args.alt is not None else 345.0
    if args.alt is None:
        print("[警告] 未给 --alt，HOME 海拔用自贡占位 345m——建议用实测值重跑")
    with_hover = not args.no_hover
    with_snake = with_hover and not args.no_snake

    items = build_items(args.lat, args.lon, alt0, args.hdg, p,
                        loiter, accept_r, with_hover, with_snake)
    print_table(items, args.lat, args.lon, args.hdg)
    t = estimate_seconds(p, loiter, with_hover, with_snake)
    budget = p["mission_timeout_s"]
    print(f"\n时长预算 ≈{t:.0f}s（任务总超时 {budget:.0f}s 的 "
          f"{t / budget * 100:.0f}%）" +
          ("" if t <= 0.9 * budget else "  ⚠️ 超过 0.9×总超时，警惕！"))
    print(f"⚠️ FC 的 WP_RADIUS_M 现值 2.0 > 搜索带间距一半 0.8m，"
          f"贴带飞前建议调到 ≤0.8（MP 参数树可改）")

    content = format_file(items)
    if args.print_only:
        print("\n--print-only：以下为文件内容预览——")
        print(content)
        return
    out = args.out if os.path.isabs(args.out) else os.path.join(os.getcwd(), args.out)
    with open(out, "w", encoding="ascii", newline="\n") as f:
        f.write(content)
    print(f"\n已生成: {out}")
    print("载入：Mission Planner → 飞行计划 → 右键 → 文件载入/保存 → 载入 WP 文件；")
    print("      载入后核对航点数与上图轨迹，再切回飞行页（切勿写错 HOME）")


if __name__ == "__main__":
    main()
