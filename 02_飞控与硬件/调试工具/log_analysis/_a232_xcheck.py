#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""235#3(及234) 垂直基准交叉验证: GPS原始海拔 vs EKF/baro + 飞机实际水平轨迹 vs carrot."""
import sys, math
sys.path.insert(0, '.')
from _dfparse import iter_messages


def extract(path, lo, hi):
    only = {'GPS', 'POS', 'GUIP', 'CTUN', 'BARO', 'MODE'}
    d = {k: [] for k in only}
    boot = None
    for tus, name, rec in iter_messages(path, only):
        if boot is None and tus is not None:
            boot = tus
        d[name].append((tus, rec))
    s0 = lambda t: (t - boot) / 1e6
    print(f'\n===== {path} =====')

    # 起飞前地面 GPS 基线 (lo-40s ~ lo)
    def gps_stats(a, b, label):
        alts = [(s0(t), r.get('Alt')) for t, r in d['GPS'] if a <= s0(t) <= b and r.get('Alt')]
        if not alts:
            print(f'  GPS {label}: 无数据')
            return None
        vals = [v for _, v in alts]
        m = sum(vals) / len(vals)
        print(f'  GPS {label}: n={len(vals)} 均值={m/100:.2f}m p50={sorted(vals)[len(vals)//2]/100:.2f}m '
              f'范围={min(vals)/100:.2f}~{max(vals)/100:.2f}')
        return m

    g_pre = gps_stats(lo - 40, lo - 2, '起飞前地面基线')
    g_fly = gps_stats(lo + 2, hi, '飞行窗口')
    if g_pre and g_fly:
        print(f'  GPS 爬升量(飞行均值-地面均值) = {(g_fly - g_pre)/100:.2f} m')

    # EKF AMSL (POS.Alt) 同窗对比
    pos_all = [(s0(t), r) for t, r in d['POS']]
    pre = [r.get('Alt') for t, r in pos_all if lo - 40 <= t <= lo - 2]
    fly = [(t, r) for t, r in pos_all if lo + 2 <= t <= hi]
    if pre and fly:
        pm = sum(pre) / len(pre)
        print(f'  EKF AMSL: 地面均值={pm:.2f}  飞行 {fly[0][0]:.1f}s={fly[0][1].get("Alt"):.2f}'
              f' → {fly[-1][0]:.1f}s={fly[-1][1].get("Alt"):.2f}')
    # BARO 原始
    bars = [(s0(t), r.get('Alt')) for t, r in d['BARO'] if lo - 10 <= s0(t) <= hi]
    if bars:
        step = max(1, len(bars) // 25)
        print(f'  BARO 原始采样(-10s起):')
        for i in range(0, len(bars), step):
            print(f'    [{bars[i][0]:8.2f}] {bars[i][1]:8.2f}')

    # 飞机实际水平轨迹 (POS lat/lng → 米, 相对窗口起点) + RelHomeAlt
    print('  --- 飞机轨迹(POS, 相对起点) vs carrot(GUIP) vs 高度 ---')
    ref = None
    rows_pos = [(t, r) for t, r in pos_all if lo - 1 <= t <= hi]
    guip = [(s0(t), r) for t, r in d['GUIP'] if lo <= s0(t) <= hi]
    for i in range(0, len(rows_pos), max(1, len(rows_pos) // 40)):
        t, r = rows_pos[i]
        if ref is None:
            ref = (r.get('Lat') / 1e7, r.get('Lng') / 1e7)
        lat = r.get('Lat') / 1e7 - ref[0]
        lng = r.get('Lng') / 1e7 - ref[1]
        ny = lat * 111320.0
        nx = lng * 111320.0 * math.cos(math.radians(ref[0]))
        print(f'    [{t:8.2f}] 飞机 N={nx:7.2f} E={ny:7.2f} RelHome={r.get("RelHomeAlt"):5.2f} '
              f'RelOrigin={r.get("RelOriginAlt"):5.2f}')
    print('  --- carrot (GUIP) 轨迹采样 ---')
    for i in range(0, len(guip), max(1, len(guip) // 20)):
        t, r = guip[i]
        print(f'    [{t:8.2f}] carrot N={r.get("pX"):7.2f} E={r.get("pY"):7.2f} z={r.get("pZ"):6.2f}')


if __name__ == '__main__':
    # 235 第三次飞窗口 (route 326.0~333.5); 234 (route 47.3~51.9)
    extract('LOGS/00000235.BIN', 325.0, 334.0)
    extract('LOGS/00000234.BIN', 46.5, 56.5)
