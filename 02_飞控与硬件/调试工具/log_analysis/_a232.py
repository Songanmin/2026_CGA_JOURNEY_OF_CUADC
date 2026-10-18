#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""232~235 号日志细提取: 电机窗口 + CTUN高度/GUIP引导目标/POS/BAT/RFND + MSG/MAVC."""
import sys
sys.path.insert(0, '.')
from _dfparse import iter_messages

MODES = {0:'STAB',2:'ALTHOLD',3:'AUTO',4:'GUIDED',5:'LOITER',6:'RTL',9:'LAND',16:'POSHOLD'}


def extract(path):
    only = {'MSG','MAVC','MODE','EV','ERR','CTUN','GUIP','POS','BAT','RCOU','RFND','ARM'}
    boot = None
    d = {k: [] for k in only}
    for tus, name, rec in iter_messages(path, only):
        if boot is None and tus is not None:
            boot = tus
        d[name].append((tus, rec))
    print(f'\n===== {path}  boot={boot} =====')
    s0 = lambda t: (t - boot) / 1e6

    # MAVC 全量
    print('-- MAVC 收到的指令 --')
    for t, r in d['MAVC']:
        print(f"   [{s0(t):8.2f}] cmd={r.get('Cmd')} p1={r.get('P1')} p2={r.get('P2')} p3={r.get('P3')} x={r.get('X')} y={r.get('Y')} z={r.get('Z')}")

    # 电机窗口(RCOU C1>1150 连续段)
    wins = []
    cur = None
    for t, r in d['RCOU']:
        on = r.get('C1', 1000) > 1150
        if on and cur is None:
            cur = [t, t]
        elif on:
            cur[1] = t
        elif cur and (t - cur[1]) > 3_000_000:
            if cur[1] - cur[0] > 2_000_000:
                wins.append(cur)
            cur = None
    if cur and cur[1] - cur[0] > 2_000_000:
        wins.append(cur)
    print(f'-- 电机窗口: {[(round(s0(a),1), round(s0(b),1)) for a,b in wins]}')

    for a, b in wins:
        print(f'\n-- 飞行窗口 [{s0(a):.1f}s ~ {s0(b):.1f}s] --')
        # 事件
        for nm, fmt in (('MODE', lambda r: f"MODE {MODES.get(r.get('ModeNum'), r.get('ModeNum'))}"),
                        ('EV', lambda r: f"EV {r.get('Id')}"),
                        ('ERR', lambda r: f"ERR subsys={r.get('Subsys')} code={r.get('ECode')}"),
                        ('ARM', lambda r: f"ARM state={r.get('ArmState')} m={r.get('Method')}")):
            for t, r in d[nm]:
                if a - 5_000_000 <= t <= b + 8_000_000:
                    print(f"   [{s0(t):8.2f}] {fmt(r)}")
        # MSG
        for t, r in d['MSG']:
            if a - 5_000_000 <= t <= b + 8_000_000:
                print(f"   [{s0(t):8.2f}] MSG {r.get('Message','')}")
        # CTUN 1s 采样
        ct = [(t, r) for t, r in d['CTUN'] if a <= t <= b]
        if ct:
            print(f'   --- CTUN {len(ct)}条 (DAlt目标/Alt实际/DSAlt/ThO) ---')
            step = max(1, len(ct) // 60)
            for i in range(0, len(ct), step):
                t, r = ct[i]
                print(f"   [{s0(t):8.2f}] DAlt={r.get('DAlt'):6.2f} Alt={r.get('Alt'):6.2f} DSAlt={r.get('DSAlt',0):6.2f} ThO={r.get('ThO'):4.2f}")
        # GUIP 1s 采样 (guided 目标)
        gp = [(t, r) for t, r in d['GUIP'] if a <= t <= b]
        if gp:
            print(f'   --- GUIP {len(gp)}条 (guided目标 pZ) ---')
            step = max(1, len(gp) // 30)
            for i in range(0, len(gp), step):
                t, r = gp[i]
                print(f"   [{s0(t):8.2f}] Type={r.get('Type')} pX={r.get('pX'):7.2f} pY={r.get('pY'):7.2f} pZ={r.get('pZ'):6.2f}")
        # POS 3s 采样
        pp = [(t, r) for t, r in d['POS'] if a <= t <= b]
        if pp:
            print(f'   --- POS {len(pp)}条 ---')
            step = max(1, len(pp) // 20)
            for i in range(0, len(pp), step):
                t, r = pp[i]
                print(f"   [{s0(t):8.2f}] RelHomeAlt={r.get('RelHomeAlt'):6.2f} RelOriginAlt={r.get('RelOriginAlt'):6.2f} Alt={r.get('Alt'):8.2f}")
        # BAT 5s
        bt = [(t, r) for t, r in d['BAT'] if a <= t <= b]
        if bt:
            step = max(1, len(bt) // 12)
            for i in range(0, len(bt), step):
                t, r = bt[i]
                print(f"   [{s0(t):8.2f}] BAT V={r.get('Volt'):6.2f} A={r.get('Curr'):6.1f}")
        # RFND 1s
        rf = [(t, r) for t, r in d['RFND'] if a <= t <= b]
        if rf:
            print(f'   --- RFND {len(rf)}条 ---')
            step = max(1, len(rf) // 20)
            for i in range(0, len(rf), step):
                t, r = rf[i]
                print(f"   [{s0(t):8.2f}] Dist={r.get('Dist')} Stat={r.get('Stat')}")


if __name__ == '__main__':
    for f in sys.argv[1:]:
        extract(f)
