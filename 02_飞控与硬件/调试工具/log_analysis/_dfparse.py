#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ArduPilot dataflash 新版格式(.BIN)解析器（2026-10 190~203 号日志实测逆向）。

格式规则（对 190 号日志全文件验证）:
  定义消息 89 字节: a3 95 80 <Type@3> <LEN@4> <name@5:4> <fmt@9:15> <?@24> <cols@25:63> <pad>
    - Type = 数据消息类型字节; LEN = 数据消息总跨度(经典语义 payload+3)
    - fmt/cols 间偶有多余 NUL(如 2 字符名 PL), 故扫描解析
  数据消息: a3 95 <Type> + 载荷(按 fmt), 跨度 = LEN
  变长消息(FMTU/UNIT/MULT, 尾部字符串被裁剪): 按字符串实际长度估跨度+验证
"""
import struct

FMTSZ = {'b':1,'B':1,'h':2,'H':2,'i':4,'I':4,'f':4,'d':8,'Q':8,'q':8,'M':1,
         'n':4,'N':16,'Z':64,'c':2,'C':2,'e':2,'E':2,'L':4,'C':2,'S':4,'s':8,'a':32}
# 注: c/C/e/E = int16/uint16 带百倍缩放(2字节); n/N/Z/s = 定长字符串
VARLEN = set()   # 实测所有消息均为定长(跨度=定义块 LEN 字节)


def fmt_size(f):
    return sum(FMTSZ.get(ch, 0) for ch in f)


def _parse_def(data, p):
    """校验并解析 p 处定义消息, 返回 (log_id, name, fmt, cols, span) 或 None."""
    if p + 89 > len(data) or data[p+2] != 0x80:
        return None
    log_id = data[p+3]
    dlen = data[p+4]                      # 数据消息跨度(经典语义)
    i = p + 5
    j = i
    while j < i + 4 and (chr(data[j]).isupper() or chr(data[j]).isdigit()):
        j += 1
    name = data[i:j].decode('ascii', 'replace')
    if not (2 <= len(name) <= 4) or j < i + 2:
        return None
    while j < p + 40 and data[j] == 0:    # 跳过名称后填充
        j += 1
    i = j
    while j < p + 40 and chr(data[j]) in FMTSZ:
        j += 1
    f = data[i:j].decode('ascii', 'replace')
    if not f or j - i > 16:
        return None
    while j < p + 88 and data[j] == 0:
        j += 1
    i = j
    while j < p + 89 and data[j] != 0:
        j += 1
    if j == i:
        return None
    cols = data[i:j].decode('ascii', 'replace').split(',')
    return log_id, name, f, cols, dlen


def load_defs(data):
    """顺序解析头部定义(允许穿插数据消息), 返回 (defs, pos)."""
    defs = {}
    pos = 0
    n = len(data)
    while pos + 4 <= n and data[pos:pos+2] == b'\xa3\x95':
        if data[pos+2] == 0x80:
            d = _parse_def(data, pos)
            if d is None:
                break
            log_id, name, f, cols, dlen = d
            defs.setdefault(log_id, (name, f, cols, dlen))
            pos += 89
            continue
        d = defs.get(data[pos+2])
        if d is None:
            break
        pos += _span(data, pos, d)
    return defs, pos


def _span(data, pos, d):
    """计算数据消息跨度; 变长消息按字符串估长+验证."""
    name, f, cols, dlen = d
    if name not in VARLEN:
        return dlen
    # FMTU: Q+B+N16+N16(尾裁) / UNIT: Q+b+Z / MULT: Q+b+d —— 按下一头回推
    nxt = data.find(b'\xa3\x95', pos + 12)
    return (nxt - pos) if nxt > 0 else dlen


def unpack_msg(body, f):
    vals = []
    off = 0
    try:
        for ch in f:
            if ch == 'b':
                v, = struct.unpack_from('<b', body, off); off += 1
            elif ch == 'B':
                v, = struct.unpack_from('<B', body, off); off += 1
            elif ch == 'h':
                v, = struct.unpack_from('<h', body, off); off += 2
            elif ch == 'H':
                v, = struct.unpack_from('<H', body, off); off += 2
            elif ch == 'i':
                v, = struct.unpack_from('<i', body, off); off += 4
            elif ch == 'I':
                v, = struct.unpack_from('<I', body, off); off += 4
            elif ch == 'f':
                v, = struct.unpack_from('<f', body, off); off += 4
            elif ch == 'd':
                v, = struct.unpack_from('<d', body, off); off += 8
            elif ch in 'qQ':
                v, = struct.unpack_from('<Q', body, off); off += 8
            elif ch == 'M':
                v, = struct.unpack_from('<B', body, off); off += 1
            elif ch == 'n':
                v = body[off:off+4].split(b'\x00')[0].decode('ascii', 'replace'); off += 4
            elif ch in 'Nc':
                v = body[off:off+16].split(b'\x00')[0].decode('ascii', 'replace'); off += 16
            elif ch in 'ZS':
                v = body[off:off+64].split(b'\x00')[0].decode('ascii', 'replace'); off += 64
            elif ch == 's':
                v = body[off:off+8].split(b'\x00')[0].decode('ascii', 'replace'); off += 8
            elif ch in 'eE':
                v, = struct.unpack_from('<h', body, off); off += 2
            elif ch in 'LC':
                v, = struct.unpack_from('<I', body, off); off += 4
            elif ch == 'a':
                off += 32; v = ''
            else:
                v = None
            vals.append(v)
    except struct.error:
        return None
    return vals


def iter_messages(path, only=None):
    """顺序走查整个日志, 产出 (t_us, name, dict{col:val})."""
    data = open(path, 'rb').read()
    n = len(data)
    defs, _ = load_defs(data)
    if not defs:
        return
    misses = 0
    pos = 0
    while pos + 3 <= n:
        if data[pos:pos+2] != b'\xa3\x95':
            nxt = data.find(b'\xa3\x95', pos+1)
            if nxt < 0:
                break
            pos = nxt
            continue
        t = data[pos+2]
        if t == 0x80:
            d = _parse_def(data, pos)
            if d is not None:
                defs.setdefault(d[0], (d[1], d[2], d[3], d[4]))
            pos += 89
            continue
        d = defs.get(t)
        if d is None:
            misses += 1
            if misses > 200:
                break
            nxt = data.find(b'\xa3\x95', pos+3)
            if nxt < 0:
                break
            pos = nxt
            continue
        name, f, cols, dlen = d
        span = _span(data, pos, d)
        vals = unpack_msg(data[pos+3:pos+span], f)
        pos += span
        if vals is None:
            continue
        if only and name not in only:
            continue
        yield vals[0] if f[0] == 'Q' else None, name, dict(zip(cols, vals))


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--types', default='MSG,EV,MODE,ERR,ARM',
                    help='关注的类型, 逗号分隔')
    args = ap.parse_args()
    only = set(args.types.split(',')) if args.types else None
    for path in args.logs:
        print(f'\n===== {path} =====')
        cnt = 0
        for tus, name, rec in iter_messages(path, only):
            cnt += 1
            if tus is None:
                continue
            s = tus / 1e6
            ts = '%02d:%06.3f' % (int(s // 60), s % 60)
            if name == 'MSG':
                print(f'  [{ts}] MSG {rec.get("Message","")}')
            elif name == 'EV':
                print(f'  [{ts}] EV  Id={rec.get("Id")}')
            elif name == 'MODE':
                print(f'  [{ts}] MODE Mode={rec.get("Mode")} Num={rec.get("ModeNum")} Rsn={rec.get("Rsn")}')
            elif name == 'ERR':
                print(f'  [{ts}] ERR Subsys={rec.get("Subsys")} ECode={rec.get("ECode")}')
            elif name == 'ARM':
                print(f'  [{ts}] ARM state={rec.get("ArmState")} checks=0x{rec.get("ArmChecks",0):x} forced={rec.get("Forced")} method={rec.get("Method")}')
        if cnt == 0:
            print('  (无匹配消息)')
