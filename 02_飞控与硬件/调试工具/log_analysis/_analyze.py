#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""190~203 号日志批量分析: 每份日志的启动/解锁/模式/错误/结束摘要 + 飞行高度剖面."""
import sys
sys.path.insert(0, '.')
from _dfparse import iter_messages

# ArduPilot 飞行模式号(部分)
MODES = {0:'STABILIZE',1:'ACRO',2:'ALT_HOLD',3:'AUTO',4:'GUIDED',5:'LOITER',6:'RTL',
         7:'CIRCLE',9:'LAND',16:'POSHOLD',17:'BRAKE',18:'THROW',20:'SMART_RTL',
         21:'FLOWHOLD',25:'ZIGZAG',26:'AUTOTUNE'}
# 关键 EV 事件号
EVTS = {10:'ARMED', 11:'DISARMED', 15:'AutoStored', 16:'TakeoffStart',
        17:'Not_Landed', 18:'LandComplete', 19:'LandStart', 25:'TakeoffComplete',
        36:'Landed?', 45:'EKFvariance', 46:'EKFvarFail?',
        51:'EKFreset?', 54:'LeakStart?', 61:'EKFAltReset?'}


def summarize(path):
    boot = None            # 首条 TimeUS
    msgs = []              # (t, text)
    arms = []
    modes = []
    errs = []
    evs = []
    ctun = []              # (t, DAlt, Alt, DesThr, Thr, AN)
    pos = []               # (t, lat, lng, alt)
    t_end = 0
    n = 0
    only = {'MSG','EV','MODE','ERR','ARM','CTUN','POS'}
    for tus, name, rec in iter_messages(path, only):
        n += 1
        if tus:
            t_end = max(t_end, tus)
            if boot is None:
                boot = tus
        if name == 'MSG':
            msgs.append((tus, rec.get('Message', '')))
        elif name == 'ARM':
            arms.append((tus, rec))
        elif name == 'MODE':
            modes.append((tus, rec))
        elif name == 'ERR':
            errs.append((tus, rec))
        elif name == 'EV':
            evs.append((tus, rec.get('Id')))
        elif name == 'CTUN':
            ctun.append((tus, rec))
        elif name == 'POS':
            pos.append((tus, rec))
    return dict(path=path, boot=boot, t_end=t_end, n=n, msgs=msgs, arms=arms,
                modes=modes, errs=errs, evs=evs, ctun=ctun, pos=pos)


def ts(tus, boot):
    if tus is None:
        return '  --  '
    s = (tus - (boot or 0)) / 1e6
    if s < 0:
        s = tus / 1e6
    return '%3d:%06.3f' % (int(s // 60), s % 60)


def report(r, verbose_ctun=False):
    out = []
    out.append(f"\n===== {r['path']} =====")
    dur = (r['t_end'] - (r['boot'] or 0)) / 1e6 if r['boot'] else 0
    out.append(f"  时间跨度 {dur:.0f}s, 消息 {r['n']}")
    # 模式变化(去重相邻)
    prev = None
    for t, rec in r['modes']:
        m = rec.get('ModeNum')
        if m != prev:
            out.append(f"  [{ts(t, r['boot'])}] MODE {MODES.get(m, m)} (Rsn={rec.get('Rsn')})")
            prev = m
    for t, rec in r['arms']:
        st = rec.get('ArmState')
        out.append(f"  [{ts(t, r['boot'])}] ARM {'ARM' if st==1 else 'DISARM'} checks=0x{rec.get('ArmChecks',0):08x} forced={rec.get('Forced')} m={rec.get('Method')}")
    # EV 去重相邻
    prev = None
    for t, e in r['evs']:
        if e != prev:
            out.append(f"  [{ts(t, r['boot'])}] EV {EVTS.get(e, e)}")
            prev = e
    for t, rec in r['errs'][:20]:
        out.append(f"  [{ts(t, r['boot'])}] ERR subsys={rec.get('Subsys')} code={rec.get('ECode')}")
    # 首尾 MSG
    m = r['msgs']
    if m:
        out.append(f"  首3条MSG: {[x[1][:40] for x in m[:3]]}")
        out.append(f"  末6条MSG: {[x[1][:60] for x in m[-6:]]}")
    # 高度剖面(起飞前后)
    c = r['ctun']
    if verbose_ctun and c:
        out.append('  --- CTUN 高度采样 (DAlt目标/Alt实际) ---')
        step = max(1, len(c)//40)
        for i in range(0, len(c), step):
            t, rec = c[i]
            out.append(f"    [{ts(t, r['boot'])}] DAlt={rec.get('DAlt'):6.2f} Alt={rec.get('Alt'):6.2f} ThO={rec.get('ThO'):4.2f}")
    return '\n'.join(out)


if __name__ == '__main__':
    import glob
    files = sys.argv[1:] or sorted(glob.glob('LOGS/0000019*.BIN') + glob.glob('LOGS/0000020*.BIN'))
    for f in files:
        r = summarize(f)
        print(report(r, verbose_ctun=('-v' in sys.argv)))
