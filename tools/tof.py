#!/usr/bin/env python3
"""Receive, check, view and record VL53L5CX skin boards over UDP.

    python tools/tof.py status                 # rate / nearest / invalid % per sensor (text)
    python tools/tof.py status --seconds 5 --json
    python tools/tof.py view                   # live 8x8 heatmaps, one panel per sensor
    python tools/tof.py record run1.npz --seconds 60
    python tools/tof.py fake                   # pretend to be a board (no hardware needed)

Wire format, one datagram per sensor frame, little-endian:
    [uint8 sensor index][64 x uint16 filtered mm][64 x uint16 raw mm]   (257 bytes)
"filtered" is 4000 where the sensor reported no target. 129-byte packets
(no raw block) are accepted too. Here, a zone with no target reads 0.

Boards send to multicast 239.0.0.1 (or unicast to this PC) on one port each.
By default this listens on ports 5004-5016 for both. Needs numpy; `view` needs
matplotlib.
"""
import argparse
import json
import math
import select
import socket
import sys
import time
from collections import defaultdict, deque

import numpy as np

GROUP = '239.0.0.1'
ZONES = 64
FIRMWARE_NO_TARGET_MM = 4000
NO_TARGET = 0
SHORT_PACKET = 1 + 2 * ZONES
FULL_PACKET = 1 + 4 * ZONES
DEFAULT_PORTS = '5004-5016'
FOV_DEG = 45.0


# -- wire format ------------------------------------------------------------------

def parse_packet(data: bytes):
    """-> (sensor index, mm (64,) uint16 with 0 = no target, raw mm (64,) or None).
    None if the datagram is too short to be a frame."""
    if len(data) < SHORT_PACKET:
        return None
    filt = np.frombuffer(data, dtype='<u2', count=ZONES, offset=1)
    raw = (np.frombuffer(data, dtype='<u2', count=ZONES, offset=SHORT_PACKET).copy()
           if len(data) >= FULL_PACKET else None)
    mm = np.where(filt >= FIRMWARE_NO_TARGET_MM, NO_TARGET, filt).astype(np.uint16)
    return int(data[0]), mm, raw


def encode_packet(index: int, mm, raw=None) -> bytes:
    """A frame exactly as the firmware sends it (mm uses 0 = no target)."""
    mm = np.asarray(mm, dtype=np.uint16).reshape(ZONES)
    filt = np.where(mm == NO_TARGET, FIRMWARE_NO_TARGET_MM, mm).astype('<u2')
    raw = filt if raw is None else np.asarray(raw, dtype='<u2').reshape(ZONES)
    return bytes([index]) + filt.tobytes() + raw.tobytes()


def zone_rays():
    """(64, 3) unit rays in the sensor frame (z out of the sensor), packet order.
    Zone k is row k // 8, column k % 8. Multiply by distance (m) for 3-D points."""
    half = math.radians(FOV_DEG / 2)
    a = np.linspace(half, -half, 8)
    az, el = np.meshgrid(a, a)
    return np.stack([np.sin(az) * np.cos(el), np.sin(el), np.cos(az) * np.cos(el)], -1).reshape(ZONES, 3)


# -- receiving -------------------------------------------------------------------

def parse_ports(spec):
    ports = []
    for part in str(spec).replace(',', ' ').split():
        if '-' in part:
            lo, hi = part.split('-')
            ports += range(int(lo), int(hi) + 1)
        else:
            ports.append(int(part))
    return sorted(set(ports))


def open_sockets(ports, iface='0.0.0.0', multicast=True):
    socks = []
    for port in ports:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        try:
            s.bind(('0.0.0.0', port))
        except OSError as e:
            print(f'[warn] port {port}: {e} (another program listening?)', file=sys.stderr)
            s.close()
            continue
        if multicast:
            try:
                s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                             socket.inet_aton(GROUP) + socket.inet_aton(iface))
            except OSError as e:
                print(f'[warn] port {port}: multicast join failed ({e}); unicast only. '
                      f'Try --iface <this PC\'s IP on the boards\' network>', file=sys.stderr)
        s.setblocking(False)
        socks.append(s)
    if not socks:
        sys.exit('[error] could not open any port')
    return socks


def frames(args):
    """Yield (t_ns, port, sensor, mm, raw, packet, source_ip) forever / until --seconds."""
    socks = open_sockets(parse_ports(args.ports), args.iface, not args.no_multicast)
    end = time.monotonic() + args.seconds if getattr(args, 'seconds', None) else None
    while end is None or time.monotonic() < end:
        timeout = 0.2 if end is None else max(0.0, min(0.2, end - time.monotonic()))
        ready, _, _ = select.select(socks, [], [], timeout)
        if not ready:
            yield None
            continue
        for s in ready:
            while True:
                try:
                    data, addr = s.recvfrom(2048)
                except (BlockingIOError, ConnectionResetError):
                    break
                p = parse_packet(data)
                if p is not None:
                    yield (time.time_ns(), s.getsockname()[1], p[0], p[1], p[2], data, addr[0])


class Stats:
    RATE_WINDOW = 2.0

    def __init__(self):
        self.n = defaultdict(int)
        self.last = {}
        self.src = {}
        self.invalid = defaultdict(int)
        self.recent = defaultdict(deque)   # receive times over the last RATE_WINDOW s

    def add(self, port, sensor, mm, src):
        k = (port, sensor)
        self.n[k] += 1
        self.last[k] = mm
        self.src[k] = src
        self.invalid[k] += int((mm == NO_TARGET).sum())
        self.recent[k].append(time.monotonic())

    def rows(self):
        now = time.monotonic()
        out = []
        for k in sorted(self.n):
            q = self.recent[k]
            while q and now - q[0] > self.RATE_WINDOW:
                q.popleft()
            hz = (len(q) - 1) / (q[-1] - q[0]) if len(q) > 1 and q[-1] > q[0] else 0.0
            mm = self.last[k]
            valid = mm[mm != NO_TARGET]
            out.append(dict(port=k[0], sensor=k[1], source=self.src[k], frames=self.n[k],
                            hz=round(hz, 1),
                            nearest_mm=int(valid.min()) if valid.size else None,
                            invalid_pct=round(100 * self.invalid[k] / (self.n[k] * ZONES), 1)))
        return out


def cmd_status(args):
    st, last_print = Stats(), time.monotonic()
    for f in frames(args):
        if f is not None:
            st.add(f[1], f[2], f[3], f[6])
        if not args.json and time.monotonic() - last_print >= 1.0:
            last_print = time.monotonic()
            rows = st.rows()
            print(f'--- {len(rows)} sensor(s) ---' if rows else
                  f'--- no frames yet on ports {args.ports} (see README "Troubleshooting") ---')
            for r in rows:
                near = f"{r['nearest_mm']:5d} mm" if r['nearest_mm'] is not None else '   none'
                print(f"port {r['port']} sensor {r['sensor']:2d}  {r['hz']:5.1f} Hz  "
                      f"nearest {near}  invalid {r['invalid_pct']:5.1f}%  from {r['source']}")
    if args.json:
        print(json.dumps({'ports': args.ports, 'seconds': args.seconds, 'sensors': st.rows()}, indent=2))
    return 0 if st.n else 1


def cmd_record(args):
    rec = defaultdict(list)
    t_end = time.monotonic() + (args.seconds or float('inf'))
    print(f'[record] -> {args.out}  (Ctrl+C to stop)')
    try:
        for f in frames(args):
            if f is None:
                continue
            t, port, sensor, mm, raw, pkt, src = f
            rec['timestamps'].append(t)
            rec['ports'].append(port)
            rec['sensor_ids'].append(sensor)
            rec['readings_mm'].append(mm)
            rec['raw_mm'].append(raw if raw is not None else np.zeros(ZONES, np.uint16))
            rec['sources'].append(src)
            if len(rec['timestamps']) % 500 == 0:
                print(f"[record] {len(rec['timestamps'])} frames")
    except KeyboardInterrupt:
        pass
    n = len(rec['timestamps'])
    np.savez_compressed(
        args.out,
        timestamps=np.array(rec['timestamps'], np.uint64),
        ports=np.array(rec['ports'], np.uint16),
        sensor_ids=np.array(rec['sensor_ids'], np.uint8),
        readings_mm=np.array(rec['readings_mm'], np.uint16).reshape(n, ZONES),
        raw_mm=np.array(rec['raw_mm'], np.uint16).reshape(n, ZONES),
        sources=np.array(rec['sources'], dtype='U15'))
    print(f'[record] saved {n} frames to {args.out}')
    return 0 if n else 1


def cmd_view(args):
    import matplotlib
    if args.save:
        matplotlib.use('Agg')   # headless: draw for --seconds, write a PNG, exit
    import matplotlib.pyplot as plt
    from matplotlib.colors import Normalize

    st = Stats()
    panels = {}
    plt.ion()
    fig = plt.figure('VL53L5CX skin', figsize=(10, 6))
    cmap = plt.get_cmap('turbo_r').copy()
    cmap.set_bad('#444444')
    norm = Normalize(0, args.max_mm)
    gen = frames(args)

    def relayout():
        fig.clf()
        keys = sorted(panels)
        cols = min(len(keys), args.cols)
        rows = math.ceil(len(keys) / cols)
        for i, k in enumerate(keys):
            ax = fig.add_subplot(rows, cols, i + 1)
            im = ax.imshow(np.full((8, 8), np.nan), cmap=cmap, norm=norm)
            ax.set_xticks([]); ax.set_yticks([])
            txt = [[ax.text(c, r, '', ha='center', va='center', fontsize=6, color='w')
                    for c in range(8)] for r in range(8)] if args.numbers else None
            panels[k] = (ax, im, txt)
        fig.colorbar(plt.cm.ScalarMappable(norm, cmap), ax=fig.axes, label='distance (mm)',
                     shrink=0.8)

    print('[view] waiting for frames... close the window to quit')
    last_draw = 0.0
    t_end = time.monotonic() + (args.seconds or (5 if args.save else float('inf')))
    while plt.fignum_exists(fig.number) and time.monotonic() < t_end:
        new = False
        for _ in range(500):
            f = next(gen)
            if f is None:
                break
            k = (f[1], f[2])
            if k not in panels:
                panels[k] = None
                new = True
            st.add(f[1], f[2], f[3], f[6])
        if new:
            relayout()
        if time.monotonic() - last_draw > 1 / 30 and panels:
            last_draw = time.monotonic()
            for r in st.rows():
                k = (r['port'], r['sensor'])
                ax, im, txt = panels[k]
                mm = st.last[k].astype(float).reshape(8, 8)
                mm[mm == NO_TARGET] = np.nan
                im.set_data(mm)
                near = f"{r['nearest_mm']} mm" if r['nearest_mm'] is not None else 'none'
                ax.set_title(f"port {k[0]}  s{k[1]}\n{r['hz']:.0f} Hz  near {near}", fontsize=8)
                if txt:
                    for z in range(ZONES):
                        v = mm.flat[z]
                        txt[z // 8][z % 8].set_text('' if np.isnan(v) else f'{v:.0f}')
        if args.save:
            time.sleep(0.01)
        else:
            plt.pause(0.01)
    if args.save:
        fig.savefig(args.save, dpi=110, bbox_inches='tight')
        print(f'[view] {len(panels)} sensor panel(s) -> {args.save}')
        return 0 if panels else 1
    return 0


def cmd_fake(args):
    """Send synthetic frames like a real board: a target sweeping across each sensor."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
    print(f'[fake] {args.sensors} sensor(s) -> {args.dest}:{args.port} at {args.hz} Hz (Ctrl+C to stop)')
    rows, cols = np.mgrid[0:8, 0:8]
    t0, n = time.monotonic(), 0
    try:
        while args.seconds is None or time.monotonic() - t0 < args.seconds:
            t = time.monotonic() - t0
            for i in range(args.sensors):
                cx, cy = 3.5 + 3 * math.sin(t + i), 3.5 + 3 * math.cos(0.7 * t + i)
                d2 = (rows - cy) ** 2 + (cols - cx) ** 2
                mm = np.where(d2 < 6, 150 + 40 * d2, 1500 + 50 * i).astype(np.uint16)
                mm[np.random.rand(8, 8) < 0.05] = NO_TARGET  # a little invalid-zone flicker
                s.sendto(encode_packet(i, mm), (args.dest, args.port))
            n += 1
            time.sleep(max(0.0, t0 + n / args.hz - time.monotonic()))
    except KeyboardInterrupt:
        pass
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    def rx(p):
        p.add_argument('--ports', default=DEFAULT_PORTS, help='e.g. 5005 or 5004-5016 or "5005,5007"')
        p.add_argument('--iface', default='0.0.0.0', help="this PC's IP on the boards' network (multicast join)")
        p.add_argument('--no-multicast', action='store_true', help='unicast only')

    p = sub.add_parser('status', help='text summary per sensor'); rx(p)
    p.add_argument('--seconds', type=float, help='stop after N s (exit 1 if no frames)')
    p.add_argument('--json', action='store_true', help='print one JSON summary at the end (use with --seconds)')
    p.set_defaults(fn=cmd_status)

    p = sub.add_parser('view', help='live heatmaps'); rx(p)
    p.add_argument('--max-mm', type=float, default=1000, help='colour scale top (mm)')
    p.add_argument('--cols', type=int, default=4)
    p.add_argument('--numbers', action='store_true', help='print mm in each zone')
    p.add_argument('--seconds', type=float, help='close after N s')
    p.add_argument('--save', metavar='PNG', help='headless: run --seconds (default 5), save a snapshot, exit')
    p.set_defaults(fn=cmd_view)

    p = sub.add_parser('record', help='save frames to .npz'); rx(p)
    p.add_argument('out')
    p.add_argument('--seconds', type=float)
    p.set_defaults(fn=cmd_record)

    p = sub.add_parser('fake', help='send synthetic frames (test without hardware)')
    p.add_argument('--dest', default='127.0.0.1', help=f'127.0.0.1, or {GROUP} for multicast')
    p.add_argument('--port', type=int, default=5005)
    p.add_argument('--sensors', type=int, default=4)
    p.add_argument('--hz', type=float, default=15)
    p.add_argument('--seconds', type=float)
    p.set_defaults(fn=cmd_fake)

    args = ap.parse_args(argv)
    return args.fn(args)


if __name__ == '__main__':
    sys.exit(main())
