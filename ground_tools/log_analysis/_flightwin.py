#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""提取指定日志飞行窗口的细节: MSG/CTUN高度/GPS/视觉位姿/CMD任务指令."""
import sys
sys.path.insert(0, '.')
from _dfparse import iter_messages

MODES = {0:'STAB',2:'ALTHOLD',3:'AUTO',4:'GUIDED',5:'LOITER',6:'RTL',9:'LAND',16:'POSHELP'}


def windows(path, wins):
    only = {'MSG','CTUN','POS','CMD','VISP','ARM','MODE','EV','ERR','AHR2'}
    boot = None
    data = {k: [] for k in only}
    for tus, name, rec in iter_messages(path, only):
        if boot is None and tus:
            boot = tus
        data[name].append((tus, rec))
    print(f'== {path}  boot={boot}')
    for (t0, t1, label) in wins:
        a = boot + int(t0*1e6)
        b = boot + int(t1*1e6)
        print(f'\n-- 窗口 [{t0:.0f}s ~ {t1:.0f}s] {label}')
        # MSG 序列
        for t, rec in data['MSG']:
            if a <= t <= b:
                s = (t-boot)/1e6
                print(f'   [{s:8.2f}] MSG {rec.get("Message","")}')
        # CTUN 高度每秒采样
        ct = [(t, r) for t, r in data['CTUN'] if a <= t <= b]
        if ct:
            step = max(1, len(ct)//50)
            print(f'   --- CTUN {len(ct)}条, 采样: ---')
            for i in range(0, len(ct), step):
                t, r = ct[i]
                s = (t-boot)/1e6
                print(f'   [{s:8.2f}] DAlt={r.get("DAlt"):6.2f} Alt={r.get("Alt"):6.2f} DSAlt={r.get("DSAlt",0):6.2f} ThO={r.get("ThO"):4.2f}')
        # CMD
        for t, rec in data['CMD']:
            if a <= t <= b:
                s = (t-boot)/1e6
                print(f'   [{s:8.2f}] CMD CTot={rec.get("CTot")} CNum={rec.get("CNum")} CId={rec.get("CId")} p1={rec.get("Prm1")} p2={rec.get("Prm2")}')
        # VISP 视觉位姿计数
        visp = [t for t, r in data['VISP'] if a <= t <= b]
        if visp:
            print(f'   VISP 视觉位姿: {len(visp)} 条 ({(visp[0]-boot)/1e6:.1f}s ~ {(visp[-1]-boot)/1e6:.1f}s)')
        # POS 首末
        pp = [(t, r) for t, r in data['POS'] if a <= t <= b]
        if pp:
            t0p, r0 = pp[0]; t1p, r1 = pp[-1]
            print(f'   POS: 起 lat={r0.get("Lat")/1e7:.7f} lng={r0.get("Lng")/1e7:.7f} alt={r0.get("Alt"):.2f} → 末 alt={r1.get("Alt"):.2f} ({len(pp)}条)')
        # 事件
        for nm in ('ARM','MODE','EV','ERR'):
            for t, rec in data[nm]:
                if a <= t <= b:
                    s = (t-boot)/1e6
                    if nm == 'ARM':
                        print(f'   [{s:8.2f}] ARM state={rec.get("ArmState")} m={rec.get("Method")}')
                    elif nm == 'MODE':
                        print(f'   [{s:8.2f}] MODE {MODES.get(rec.get("ModeNum"), rec.get("ModeNum"))}')
                    elif nm == 'EV':
                        print(f'   [{s:8.2f}] EV {rec.get("Id")}')
                    else:
                        print(f'   [{s:8.2f}] ERR subsys={rec.get("Subsys")} code={rec.get("ECode")}')


if __name__ == '__main__':
    path = sys.argv[1]
    wins = []
    for w in sys.argv[2:]:
        t0, t1, label = w.split(',', 2) if ',' in w else (w, None, '')
        pass
    # 硬编码本批窗口
    import re
    m = re.search(r'(\d+)\.BIN', path)
    ln = int(m.group(1))
    WINS = {
        199: [(90, 130, '第1次解锁试飞'), (575, 745, '第6次解锁 2.5min'), (2260, 2290, 'STAB跳变'),
              (2400, 2560, '末段循环A'), (2580, 2700, '末段循环B(最终坠机?)')],
        200: [(100, 215, '解锁飞行+急停')],
        202: [(280, 415, '解锁飞行 111s')],
        203: [(1390, 1440, '解锁前'), (1440, 1580, '自主飞行+EStop坠机')],
    }[ln]
    windows(path, WINS)
