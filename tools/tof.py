#!/usr/bin/env python3
"""Receive, check, view, record and time VL53L5CX skin boards (UDP or USB serial).

    python tools/tof.py status                 # rate / nearest / invalid % per sensor (text)
    python tools/tof.py status --seconds 5 --json
    python tools/tof.py live                   # fast live 8x8 view (Tk, nothing to install)
    python tools/tof.py view                   # live 8x8 heatmaps in matplotlib (slower)
    python tools/tof.py record run1.npz --seconds 60
    python tools/tof.py timing --seconds 20 --save timing.png   # data over time + sampling report
    python tools/tof.py timing --from run1.npz --save timing.png
    python tools/tof.py fake                   # pretend to be a board (no hardware needed)

Add --serial COM4 (or /dev/ttyACM0) to any receiving command to read a board
over its USB cable instead of Wi-Fi (firmware built with SEND_SERIAL 1).

Wire format, one datagram (or serial frame) per sensor frame, little-endian:
    [u8 sensor][64 x u16 filtered mm][64 x u16 raw mm][u32 board us][u16 frame counter]
"filtered" is 4000 where the sensor reported no target. The last 6 bytes come from
this repo's firmware; 257-byte (classic) and 129-byte packets are accepted too.
Here, a zone with no target reads 0. Serial frames are wrapped [A5 5A][u16 len][packet].

Boards send to multicast 239.0.0.1 (or unicast to this PC) on one port each.
By default this listens on ports 5004-5016 for both. Needs numpy; `view` and
`timing --save` need matplotlib; `--serial` needs pyserial.
"""
import argparse
import json
import math
import select
import socket
import sys
import time
from collections import defaultdict, deque, namedtuple

import numpy as np

GROUP = '239.0.0.1'
ZONES = 64
FIRMWARE_NO_TARGET_MM = 4000
NO_TARGET = 0
SHORT_PACKET = 1 + 2 * ZONES          # [id][filtered]
FULL_PACKET = 1 + 4 * ZONES           # [id][filtered][raw]
META_PACKET = FULL_PACKET + 6         # ...[board us][counter]
SERIAL_MAGIC = b'\xa5\x5a'
DEFAULT_PORTS = '5004-5016'
FOV_DEG = 45.0

# Categorical slots (validated for CVD separation on a light surface) and a
# single-hue sequential ramp for distance: dark = near, light = far.
SERIES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300', '#6250d6', '#e34948']
DIST_RAMP = ['#0d366b', '#184f95', '#256abf', '#3987e5', '#6da7ec', '#9ec5f4', '#cde2fb']
NO_TARGET_COLOR = '#c9c7c0'
INK, INK2, GRID = '#0b0b0b', '#52514e', '#e4e3df'

Frame = namedtuple('Frame', 't_ns port sensor mm raw packet source board_us seq')


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


def parse_meta(data: bytes):
    """-> (board time us (uint32, wraps every ~71 min), frame counter (uint16)),
    or (None, None) for packets without the trailer."""
    if len(data) < META_PACKET:
        return None, None
    us = int.from_bytes(data[FULL_PACKET:FULL_PACKET + 4], 'little')
    seq = int.from_bytes(data[FULL_PACKET + 4:META_PACKET], 'little')
    return us, seq


def encode_packet(index: int, mm, raw=None, board_us=None, seq=None) -> bytes:
    """A frame exactly as the firmware sends it (mm uses 0 = no target)."""
    mm = np.asarray(mm, dtype=np.uint16).reshape(ZONES)
    filt = np.where(mm == NO_TARGET, FIRMWARE_NO_TARGET_MM, mm).astype('<u2')
    raw = filt if raw is None else np.asarray(raw, dtype='<u2').reshape(ZONES)
    pkt = bytes([index]) + filt.tobytes() + raw.tobytes()
    if board_us is not None:
        pkt += int(board_us % 2**32).to_bytes(4, 'little') + int((seq or 0) % 2**16).to_bytes(2, 'little')
    return pkt


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


def _make_frame(t_ns, port, data, source):
    p = parse_packet(data)
    if p is None:
        return None
    us, seq = parse_meta(data)
    return Frame(t_ns, port, p[0], p[1], p[2], data, source, us, seq)


def _udp_frames(args, end, idle=0.2):
    socks = open_sockets(parse_ports(args.ports), args.iface, not args.no_multicast)
    while end is None or time.monotonic() < end:
        timeout = idle if end is None else max(0.0, min(idle, end - time.monotonic()))
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
                f = _make_frame(time.time_ns(), s.getsockname()[1], data, addr[0])
                if f is not None:
                    yield f


def _serial_frames(args, end, idle=0.2):
    try:
        import serial
    except ImportError:
        sys.exit('[error] --serial needs pyserial: pip install pyserial')
    ser = serial.Serial(None, 921600, timeout=0.2)
    ser.port = args.serial
    ser.dtr = ser.rts = False        # opening the port must not reset an ESP32
    ser.open()
    buf = bytearray()
    text = bytearray()
    while end is None or time.monotonic() < end:
        if idle == 0 and ser.in_waiting == 0:      # non-blocking: hand control back at once
            yield None
            continue
        chunk = ser.read(max(1, ser.in_waiting))   # return as soon as bytes arrive
        t_ns = time.time_ns()
        if not chunk:
            yield None
            continue
        buf += chunk
        while True:
            i = buf.find(SERIAL_MAGIC)
            if i < 0:                       # keep a possible half magic at the end
                text += buf[:-1]; del buf[:-1]
                break
            text += buf[:i]
            if len(buf) < i + 4:
                del buf[:i]
                break
            n = int.from_bytes(buf[i + 2:i + 4], 'little')
            if not SHORT_PACKET <= n <= 1024:  # magic inside a log line or noise: skip it
                del buf[:i + 1]
                continue
            if len(buf) < i + 4 + n:
                del buf[:i]
                break
            f = _make_frame(t_ns, args.serial, bytes(buf[i + 4:i + 4 + n]), args.serial)
            del buf[:i + 4 + n]
            if f is not None:
                yield f
        while b'\n' in text:                 # the board's log lines
            line, _, rest = text.partition(b'\n')
            text = bytearray(rest)
            if getattr(args, 'log', False) and line.strip():
                print('[board]', line.decode('utf-8', 'replace').rstrip(), file=sys.stderr)


def frames(args, idle=0.2):
    """Yield Frame tuples (or None when idle) forever / until --seconds.
    idle: how long to wait for data before yielding None. 0 = never block (viewers)."""
    end = time.monotonic() + args.seconds if getattr(args, 'seconds', None) else None
    if getattr(args, 'serial', None):
        return _serial_frames(args, end, idle)
    return _udp_frames(args, end, idle)


def drain(gen, budget_s=0.01):
    """Pull every frame that is already waiting (up to budget_s), never blocking.
    -> list of frames, or None when the source has ended."""
    out, t_end = [], time.monotonic() + budget_s
    while time.monotonic() < t_end:
        try:
            f = next(gen)
        except StopIteration:
            return out or None
        if f is None:
            break
        out.append(f)
    return out


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
        for k in sorted(self.n, key=str):
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


def _where(args):
    return args.serial if getattr(args, 'serial', None) else f'ports {args.ports}'


def cmd_status(args):
    st, last_print = Stats(), time.monotonic()
    for f in frames(args):
        if f is not None:
            st.add(f.port, f.sensor, f.mm, f.source)
        if not args.json and time.monotonic() - last_print >= 1.0:
            last_print = time.monotonic()
            rows = st.rows()
            print(f'--- {len(rows)} sensor(s) ---' if rows else
                  f'--- no frames yet on {_where(args)} (see README "Troubleshooting") ---')
            for r in rows:
                near = f"{r['nearest_mm']:5d} mm" if r['nearest_mm'] is not None else '   none'
                print(f"port {r['port']} sensor {r['sensor']:2d}  {r['hz']:5.1f} Hz  "
                      f"nearest {near}  invalid {r['invalid_pct']:5.1f}%  from {r['source']}")
    if args.json:
        print(json.dumps({'source': _where(args), 'seconds': args.seconds, 'sensors': st.rows()}, indent=2))
    return 0 if st.n else 1


# -- recordings --------------------------------------------------------------------

def collect(args, verbose=True):
    """Receive until --seconds / Ctrl+C -> dict of arrays (the .npz layout)."""
    rec = defaultdict(list)
    try:
        for f in frames(args):
            if f is None:
                continue
            rec['timestamps'].append(f.t_ns)
            rec['ports'].append(f.port if isinstance(f.port, int) else 0)
            rec['sensor_ids'].append(f.sensor)
            rec['readings_mm'].append(f.mm)
            rec['raw_mm'].append(f.raw if f.raw is not None else np.zeros(ZONES, np.uint16))
            rec['sources'].append(f.source)
            rec['board_us'].append(f.board_us if f.board_us is not None else 0)
            rec['seq'].append(f.seq if f.seq is not None else 0)
            rec['has_meta'].append(f.board_us is not None)
            if verbose and len(rec['timestamps']) % 500 == 0:
                print(f"[rx] {len(rec['timestamps'])} frames", file=sys.stderr)
    except KeyboardInterrupt:
        pass
    n = len(rec['timestamps'])
    return dict(
        timestamps=np.array(rec['timestamps'], np.uint64),
        ports=np.array(rec['ports'], np.uint16),
        sensor_ids=np.array(rec['sensor_ids'], np.uint8),
        readings_mm=np.array(rec['readings_mm'], np.uint16).reshape(n, ZONES),
        raw_mm=np.array(rec['raw_mm'], np.uint16).reshape(n, ZONES),
        sources=np.array(rec['sources'], dtype='U32'),
        board_us=np.array(rec['board_us'], np.uint32),
        seq=np.array(rec['seq'], np.uint16),
        has_meta=np.array(rec['has_meta'], bool))


def load_recording(path):
    """Load a tof.py (or HIRO-style) .npz into the collect() layout."""
    z = np.load(path, allow_pickle=False)
    n = len(z['timestamps'])
    if 'readings_mm' in z:
        mm = z['readings_mm']
    else:  # HIRO recordings: raw packets only
        mm = np.stack([parse_packet(bytes(p))[1] for p in z['packets']]) if n else np.zeros((0, ZONES), np.uint16)
    get = lambda k, d: z[k] if k in z else d
    return dict(timestamps=z['timestamps'].astype(np.uint64),
                ports=get('ports', np.zeros(n, np.uint16)),
                sensor_ids=z['sensor_ids'].astype(np.uint8),
                readings_mm=mm.astype(np.uint16),
                sources=get('sources', np.array([''] * n)),
                board_us=get('board_us', np.zeros(n, np.uint32)),
                seq=get('seq', np.zeros(n, np.uint16)),
                has_meta=get('has_meta', np.zeros(n, bool)),
                # HIRO's recorder doesn't store which board (port) a frame came from
                boards_known='ports' in z)


def cmd_record(args):
    print(f'[record] -> {args.out}  (Ctrl+C to stop)')
    rec = collect(args)
    np.savez_compressed(args.out, **rec)
    n = len(rec['timestamps'])
    print(f'[record] saved {n} frames to {args.out}' + ('' if rec['has_meta'].all() or not n else
          '  (no board timestamps: timing will use PC receive time)'))
    return 0 if n else 1


# -- timing analysis ---------------------------------------------------------------

def _unwrap_us(us):
    """uint32 microsecond counter (in arrival order) -> monotonic seconds."""
    d = np.diff(us.astype(np.int64)) % 2**32
    d = np.where(d >= 2**31, d - 2**32, d)
    return np.concatenate([[0], np.cumsum(d)]) / 1e6


def _q(x, p):
    return float(np.percentile(x, p)) if len(x) else float('nan')


PAUSE_S = 1.0          # intervals longer than this (and 10x the median) are recording pauses
LOCK_MS_PER_S = 0.5    # sensors whose relative phase drifts less than this share a read loop
LOCK_RESID_MS = 3.0    # ... and whose (2 s averaged) phase wanders less than this around that line
CHUNK_S = 2.0          # phase is averaged over chunks this long before fitting drift


def _phase(ref_t, t):
    """Phase of frames t after the most recent ref_t frame, and its drift.
    -> (times, phase s, drift s/s or None, wander s or None)."""
    if len(ref_t) < 2 or len(t) < 1:
        return t[:0], t[:0], None, None
    period = np.median(np.diff(ref_t))
    j = np.searchsorted(ref_t, t, side='right') - 1
    ok = j >= 0
    tt, ph = t[ok], (t[ok] - ref_t[j[ok]]) % period
    # Drift from the circular mean phase of ~2 s chunks: averages out transport
    # jitter (Wi-Fi arrival times scatter by several ms) but keeps clock drift.
    edges = np.arange(tt[0], tt[-1] + CHUNK_S, CHUNK_S)
    ang = ph / period * 2 * np.pi
    ct, cp = [], []
    for a, b in zip(edges[:-1], edges[1:]):
        sel = (tt >= a) & (tt < b)
        if sel.sum() >= 5:
            ct.append(tt[sel].mean())
            cp.append(np.angle(np.exp(1j * ang[sel]).mean()))
    if len(ct) < 3:
        return tt, ph, None, None
    cp = np.unwrap(np.array(cp)) / (2 * np.pi) * period
    fit = np.polyfit(ct, cp, 1)
    resid = cp - np.polyval(fit, ct)          # robust spread: a Wi-Fi outage scrambles a few chunks
    return tt, ph, float(fit[0]), float(1.4826 * np.median(np.abs(resid - np.median(resid))))


def parse_boards(spec):
    """'71-74;75,77,81,82' -> [[71, 72, 73, 74], [75, 77, 81, 82]]"""
    groups = []
    for g in spec.split(';'):
        ids = []
        for part in g.replace(' ', '').split(','):
            if '-' in part:
                lo, hi = part.split('-')
                ids += range(int(lo), int(hi) + 1)
            elif part:
                ids.append(int(part))
        if ids:
            groups.append(ids)
    return groups


def analyze(rec, boards_spec=None):
    """Per-sensor sampling report. Times are seconds from the first frame.

    Board clock (when packets carry it) = when the board read the frame, so its
    intervals are the sensor's own sampling. PC clock = when the frame arrived,
    so PC intervals = sensor sampling + transport jitter.

    Which board a sensor is on comes from its port/source. Recordings that don't
    store that (HIRO's .npz) take boards from boards_spec, or are grouped by
    timing: HIRO's firmware reads its sensors in a fixed loop, so sensors on one
    board stay phase-locked (drift ~0) while separate boards drift apart."""
    t_pc = (rec['timestamps'].astype(np.int64) - int(rec['timestamps'].min())) / 1e9
    links = sorted(set(zip(rec['ports'].tolist(), rec['sources'].tolist())))
    # board time on the PC's axis: unwrap per board, then align its start to the PC clock
    t_board = np.full(len(t_pc), np.nan)
    for port, src in links:
        m = (rec['ports'] == port) & (rec['sources'] == src) & rec['has_meta']
        if m.sum() > 1:
            idx = np.flatnonzero(m)
            tb = _unwrap_us(rec['board_us'][idx])
            t_board[idx] = tb + np.min(t_pc[idx] - tb)   # board time shifted to the earliest arrival
    use_board = bool(np.isfinite(t_board).any())
    t = np.where(np.isfinite(t_board), t_board, t_pc)

    keys = sorted(set(zip(rec['ports'].tolist(), rec['sources'].tolist(), rec['sensor_ids'].tolist())))
    multi = len(links) > 1
    sensors = []
    for port, src, sid in keys:
        m = np.flatnonzero((rec['ports'] == port) & (rec['sources'] == src) & (rec['sensor_ids'] == sid))
        ts, tp = t[m], t_pc[m]
        order = np.argsort(ts, kind='stable')
        m, ts, tp = m[order], ts[order], tp[order]
        dt_all = np.diff(ts) * 1e3
        dtp = np.diff(np.sort(tp)) * 1e3
        med = float(np.median(dt_all)) if len(dt_all) else float('nan')
        pause = dt_all > max(PAUSE_S * 1e3, 10 * med) if len(dt_all) else np.zeros(0, bool)
        dt = dt_all[~pause]
        r = dict(label=(f'{src or port}:s{sid}' if multi else f's{sid}'), port=port, source=src, sensor=int(sid),
                 board=f'{src or port}' if multi else None,
                 frames=int(len(m)), clock='board' if np.isfinite(t_board[m]).all() else 'pc',
                 rate_hz=round(len(dt) / (dt.sum() / 1e3), 3) if len(dt) and dt.sum() > 0 else None,
                 interval_ms=dict(mean=round(float(dt.mean()), 2) if len(dt) else None,
                                  median=round(med, 2), std=round(float(dt.std()), 2) if len(dt) else None,
                                  min=round(_q(dt, 0), 2), p99=round(_q(dt, 99), 2), max=round(_q(dt, 100), 2)),
                 pc_interval_std_ms=round(float(dtp.std()), 2) if len(dtp) else None,
                 gaps=int((dt > 1.5 * med).sum()) if len(dt) else 0,
                 pauses=int(pause.sum()),
                 invalid_pct=round(100 * float((rec['readings_mm'][m] == NO_TARGET).mean()), 1))
        if rec['has_meta'][m].all() and len(m) > 1:
            steps = (np.diff(rec['seq'][m].astype(np.int64)) % 2**16)[~pause]
            r['lost'] = int((steps - 1).clip(0).sum())
            if len(dt) and dt.sum() > 0:      # frames the sensor made, dropped in transport or not
                r['received_hz'] = r['rate_hz']
                r['rate_hz'] = round(steps.sum() / (dt.sum() / 1e3), 3)
            lat = (tp - ts) * 1e3
            r['transport_jitter_ms'] = dict(p50=round(_q(lat - lat.min(), 50), 2),
                                            p99=round(_q(lat - lat.min(), 99), 2))
        r['_idx'], r['_t'] = m, ts
        sensors.append(r)

    # which board each sensor is on
    boards_source = 'port/source'
    if not rec.get('boards_known', True) and not multi and len(sensors) > 1:
        if boards_spec:
            boards_source = '--boards'
            for b, ids in enumerate(parse_boards(boards_spec), 1):
                for r in sensors:
                    if r['sensor'] in ids:
                        r['board'] = f'b{b}'
            for r in sensors:
                r['board'] = r['board'] or 'other'
        else:
            boards_source = 'inferred from timing'
            todo, b = list(sensors), 0
            while todo:
                b += 1
                ref = todo.pop(0)
                ref['board'] = f'b{b}'
                for r in list(todo):
                    _, _, drift, wander = _phase(ref['_t'], r['_t'])
                    if drift is not None and abs(drift) * 1e3 < LOCK_MS_PER_S and wander * 1e3 < LOCK_RESID_MS:
                        r['board'] = f'b{b}'
                        todo.remove(r)

    # phase of each sensor against the first sensor on the same board
    for board in dict.fromkeys(r['board'] for r in sensors):
        group = [r for r in sensors if r['board'] == board]
        ref = group[0]
        for r in group:
            tt, ph, drift, _ = _phase(ref['_t'], r['_t'])
            if not len(tt):
                continue
            r['_phase_t'], r['_phase_ms'] = tt, ph * 1e3
            r['phase_vs'] = ref['label']
            r['phase_ms_median'] = round(float(np.median(ph)) * 1e3, 2)
            if drift is not None and r is not ref:   # drift: unwrapped phase slope = rate difference
                r['drift_ms_per_s'] = round(drift * 1e3, 3)
    return dict(clock='board' if use_board else 'pc', duration_s=round(float(t.max() - t.min()), 2),
                frames=int(len(t)), t=t, sensors=sensors, boards_from=boards_source)


def print_report(an):
    print(f"\n{an['frames']} frames over {an['duration_s']} s; timing from the "
          f"{'board clock (sensor sampling)' if an['clock'] == 'board' else 'PC receive clock (includes transport jitter)'}")
    groups = {}
    for r in an['sensors']:
        groups.setdefault(r['board'], []).append(r['label'])
    if len(groups) > 1 or None not in groups:
        print(f"boards ({an['boards_from']}): " + '   '.join(f"{b}: {' '.join(v)}" for b, v in groups.items()))
    if any(r['pauses'] for r in an['sensors']):
        print(f"recording pauses over {PAUSE_S:.0f} s are left out of rates and intervals")
    print(f"{'sensor':>8} {'board':>5} {'frames':>6} {'rate Hz':>8} {'interval ms':>12} {'std':>6} {'p99':>6} {'max':>7} "
          f"{'gaps':>4} {'lost':>4} {'phase ms':>8} {'drift ms/s':>10} {'PC jit p99':>10} {'invalid%':>8}")
    for r in an['sensors']:
        iv = r['interval_ms']
        f = lambda v, w, p=1: f'{v:{w}.{p}f}' if isinstance(v, (int, float)) and v is not None else f"{'-':>{w}}"
        print(f"{r['label']:>8} {r['board'] or '-':>5} {r['frames']:6d} {f(r['rate_hz'], 8, 2)} {f(iv['mean'], 12, 2)} {f(iv['std'], 6, 2)} "
              f"{f(iv['p99'], 6)} {f(iv['max'], 7)} {r['gaps']:4d} {r.get('lost', '-'):>4} "
              f"{f(r.get('phase_ms_median'), 8)} {f(r.get('drift_ms_per_s'), 10, 3)} "
              f"{f((r.get('transport_jitter_ms') or {}).get('p99'), 10)} {r['invalid_pct']:8.1f}")


def plot_timing(rec, an, path, window_s=1.0):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap, Normalize

    plt.rcParams.update({'font.size': 9, 'axes.edgecolor': GRID, 'axes.labelcolor': INK2,
                         'xtick.color': INK2, 'ytick.color': INK2, 'axes.titlecolor': INK,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6,
                         'figure.facecolor': '#fcfcfb', 'axes.facecolor': '#fcfcfb'})
    S = an['sensors']
    n = len(S)
    boards = list(dict.fromkeys(r['board'] for r in S))
    by_board = len(boards) > 1
    # colour = board when there are several boards, else sensor; past 8, identity comes from labels
    slot = (lambda i: boards.index(S[i]['board'])) if by_board else (lambda i: i)
    col = lambda i: SERIES[slot(i)] if slot(i) < len(SERIES) else INK2
    cmap = LinearSegmentedColormap.from_list('dist', DIST_RAMP)
    cmap.set_bad(NO_TARGET_COLOR)
    t_all = an['t']
    t0 = t_all.min()

    bottom = max(3.4, 0.32 * n)
    fig = plt.figure(figsize=(12, 4.2 + 0.6 * n + 1.2 + bottom))
    gs = fig.add_gridspec(4 + n, 2, height_ratios=[3.2, 0.9] + [0.55] * n + [0.9, bottom],
                          hspace=0.3, wspace=0.42)
    fig.suptitle(f"VL53L5CX skin: {n} sensor(s), {an['duration_s']} s, timing from the "
                 f"{'board clock' if an['clock'] == 'board' else 'PC receive clock'}",
                 x=0.06, ha='left', fontsize=12, color=INK)

    # A: nearest valid distance over time, per sensor (one board) or per board (several)
    ax = fig.add_subplot(gs[0, :])

    def nearest(r):
        mm = rec['readings_mm'][r['_idx']].astype(float)
        mm[mm == NO_TARGET] = np.inf
        near = mm.min(axis=1)
        near[np.isinf(near)] = np.nan
        return r['_t'] - t0, near

    if by_board:
        for b, board in enumerate(boards):
            members = [r for r in S if r['board'] == board]
            tt = np.concatenate([nearest(r)[0] for r in members])
            nn = np.concatenate([nearest(r)[1] for r in members])
            order = np.argsort(tt)
            tt, nn = tt[order], nn[order]
            # nearest across the board's sensors, over each ~1 frame period
            step = float(np.median([np.median(np.diff(r['_t'])) for r in members]))
            bins = np.floor(tt / step).astype(int)
            ub, first = np.unique(bins, return_index=True)
            mins = np.fmin.reduceat(nn, first)
            rate = np.mean([r['rate_hz'] or 0 for r in members])
            ax.plot(ub * step, mins, lw=1.5, color=SERIES[b] if b < len(SERIES) else INK2,
                    label=f"{board}: {' '.join(r['label'] for r in members)}  ~{rate:.2f} Hz")
        ax.set_title('Nearest valid zone per board over time', loc='left')
        ax.legend(ncol=1, frameon=False, loc='upper right', fontsize=8)
    else:
        for i, r in enumerate(S):
            ax.plot(*nearest(r), lw=1.5, color=col(i), label=f"{r['label']}  {r['rate_hz'] or 0:.2f} Hz")
        ax.set_title('Nearest valid zone per sensor over time', loc='left')
        ax.legend(ncol=min(n, 4), frameon=False, loc='upper right', fontsize=8)
    ax.set_ylabel('nearest target (mm)')
    ax.set_xlim(0, an['duration_s'])
    ax.set_xlabel('time (s)')
    hx = fig.add_subplot(gs[1, :]); hx.axis('off')      # spacer that holds the kymograph title
    hx.text(0, 0.05, 'All 64 zones over time (rows = zones 0-63; dark = near, grey = no target)',
            transform=hx.transAxes, fontsize=plt.rcParams['axes.titlesize'] and 10, color=INK)

    # B: zone kymographs (64 zones x frames), one strip per sensor, same time axis
    norm = Normalize(0, 1500)
    for i, r in enumerate(S):
        kx = fig.add_subplot(gs[2 + i, :], sharex=ax)
        mm = rec['readings_mm'][r['_idx']].astype(float).T
        mm[mm == NO_TARGET] = np.nan
        tt = r['_t'] - t0
        edges = np.concatenate([tt, [tt[-1] + (np.median(np.diff(tt)) if len(tt) > 1 else 0.07)]])
        kx.pcolormesh(edges, np.arange(ZONES + 1), mm, cmap=cmap, norm=norm, shading='flat', rasterized=True)
        kx.set_yticks([]); kx.grid(False)
        kx.set_ylabel(r['label'] + (f" ({r['board']})" if by_board else ''), rotation=0, ha='right',
                      va='center', color=INK)
        kx.tick_params(labelbottom=(i == n - 1))
    cax = hx.inset_axes([0.72, 0.35, 0.28, 0.3])
    cb = fig.colorbar(plt.cm.ScalarMappable(norm, cmap), cax=cax, orientation='horizontal')
    cb.set_label('distance (mm)', color=INK2, fontsize=8); cb.outline.set_visible(False)
    cax.tick_params(labelsize=7)

    # C: inter-frame interval distribution per sensor
    cx = fig.add_subplot(gs[3 + n, 0])
    rng = np.random.default_rng(0)
    for i, r in enumerate(S):
        dt = np.diff(r['_t']) * 1e3
        if not len(dt):
            continue
        cx.scatter(dt, i + rng.uniform(-0.25, 0.25, len(dt)), s=8, color=col(i), alpha=0.35, lw=0)
        cx.plot([np.median(dt)] * 2, [i - 0.38, i + 0.38], color=INK, lw=2)
    sep = '  ' if n > 6 else '\n'
    cx.set_yticks(range(n), [f"{r['label']}{sep}{r['interval_ms']['mean']:.1f}±{r['interval_ms']['std']:.1f} ms"
                             for r in S], fontsize=8)
    cx.invert_yaxis()
    cx.set_xlabel('interval between frames (ms); bar = median')

    # D: each sensor's phase against the first sensor on its board, over time.
    # Flat = locked together; a sawtooth = the sensor's clock runs fast/slow (slope = drift).
    dx = fig.add_subplot(gs[3 + n, 1], sharex=ax)
    period_ms = None
    for i, r in enumerate(S):
        if '_phase_ms' not in r or r.get('phase_vs') == r['label']:
            continue
        period_ms = period_ms or float(np.median(np.diff(next(q for q in S if q['label'] == r['phase_vs'])['_t']))) * 1e3
        drift = f"  {r['drift_ms_per_s']:+.1f} ms/s" if 'drift_ms_per_s' in r else ''
        vs = f" vs {r['phase_vs']}" if r['phase_vs'] != S[0]['label'] else ''
        dx.scatter(r['_phase_t'] - t0, r['_phase_ms'], s=5, color=col(i), lw=0, label=f"{r['label']}{vs}{drift}")
    if period_ms:
        dx.set_ylim(0, period_ms)
    dx.set_ylabel("ms after its board's first sensor")
    dx.set_xlabel('time (s)')
    dx.legend(frameon=False, fontsize=8, loc='upper left', bbox_to_anchor=(1.0, 1.0), markerscale=2.5)
    dx.set_title("Phase vs the first sensor on the same board\n(flat = in step; sawtooth = clock drift)", loc='left')
    cx.set_title('Sampling interval per sensor\n(dots = frames; gaps at 2x = lost frames)', loc='left')

    fig.savefig(path, dpi=110, bbox_inches='tight')
    plt.close(fig)


def cmd_timing(args):
    if args.from_npz:
        rec = load_recording(args.from_npz)
    else:
        print(f'[timing] capturing {args.seconds:.0f} s from {_where(args)} ...', file=sys.stderr)
        rec = collect(args, verbose=False)
        if args.out:
            np.savez_compressed(args.out, **rec)
            print(f'[timing] raw frames -> {args.out}', file=sys.stderr)
    if len(rec['timestamps']) < 3:
        print(f'[timing] only {len(rec["timestamps"])} frames: nothing to analyse '
              f'(see README "Troubleshooting")', file=sys.stderr)
        return 1
    an = analyze(rec, args.boards)
    if args.json:
        clean = [{k: v for k, v in r.items() if not k.startswith('_')} for r in an['sensors']]
        print(json.dumps({'clock': an['clock'], 'duration_s': an['duration_s'], 'frames': an['frames'],
                          'boards_from': an['boards_from'],
                          'sensors': clean}, indent=2))
    else:
        print_report(an)
    if args.save:
        plot_timing(rec, an, args.save)
        print(f'[timing] figure -> {args.save}', file=sys.stderr)
    return 0


# -- live view ---------------------------------------------------------------------

class RenderMeter:
    """Counts redraws and how stale the shown data was (render time - receive time)."""

    def __init__(self):
        self.t0, self.draws, self.lags = time.monotonic(), 0, []

    def drew(self, newest_t_ns):
        self.draws += 1
        if newest_t_ns:
            self.lags.append((time.time_ns() - newest_t_ns) / 1e6)

    def report(self, name):
        dt = max(time.monotonic() - self.t0, 1e-6)
        lag = np.array(self.lags[len(self.lags) // 10:] or [np.nan])   # skip the warm-up
        print(f'[{name}] {self.draws / dt:.0f} redraws/s, display lag p50 {np.median(lag):.0f} ms, '
              f'p95 {np.percentile(lag, 95):.0f} ms', file=sys.stderr)


def cmd_view(args):
    import matplotlib
    if args.save:
        matplotlib.use('Agg')   # headless: draw for --seconds, write a PNG, exit
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap, Normalize

    st, meter = Stats(), RenderMeter()
    panels, newest = {}, {}
    fig = plt.figure('VL53L5CX skin', figsize=(10, 6))
    closed = []
    fig.canvas.mpl_connect('close_event', lambda e: closed.append(True))
    cmap = LinearSegmentedColormap.from_list('dist', DIST_RAMP)
    cmap.set_bad(NO_TARGET_COLOR)
    norm = Normalize(0, args.max_mm)
    gen = frames(args, idle=0)
    # Blitting: the axes, colourbar and frame are drawn once; each update repaints only
    # the images and text (~3x faster than a full matplotlib redraw).
    blit = not args.save
    bg = []
    if args.numbers:
        print('[view] --numbers is slow in matplotlib (it redraws 64 labels per sensor); '
              '`tof.py live --numbers` is much faster', file=sys.stderr)

    def grab_background(_=None):
        bg[:] = [fig.canvas.copy_from_bbox(fig.bbox)]

    def relayout():
        fig.clf()
        keys = sorted(panels, key=str)
        cols = min(len(keys), args.cols)
        rows = math.ceil(len(keys) / cols)
        for i, k in enumerate(keys):
            ax = fig.add_subplot(rows, cols, i + 1)
            im = ax.imshow(np.full((8, 8), np.nan), cmap=cmap, norm=norm, interpolation='nearest', animated=blit)
            ax.set_xticks([]); ax.set_yticks([])
            title = ax.text(0, 1.03, f'{k[0]}  s{k[1]}', transform=ax.transAxes, fontsize=8, va='bottom',
                            animated=blit)
            txt = [[ax.text(c, r, '', ha='center', va='center', fontsize=6, color='w', animated=blit)
                    for c in range(8)] for r in range(8)] if args.numbers else None
            panels[k] = (ax, im, txt, title)
        cb = fig.colorbar(plt.cm.ScalarMappable(norm, cmap), ax=fig.axes, label='distance (mm)', shrink=0.8)
        cb.outline.set_visible(False)
        if blit:
            fig.canvas.draw()
            grab_background()

    if blit:
        fig.canvas.mpl_connect('resize_event', lambda e: (fig.canvas.draw(), grab_background()))

    if not args.save:
        plt.show(block=False)
    print('[view] waiting for frames... close the window to quit')
    last_draw = last_titles = 0.0
    t_end = time.monotonic() + (args.seconds or (5 if args.save else float('inf')))
    while not closed and time.monotonic() < t_end:
        got = drain(gen)                       # everything waiting, never blocks
        if got is None:
            break
        new = False
        for f in got:
            k = (f.port, f.sensor)
            if k not in panels:
                panels[k] = None
                new = True
            st.add(f.port, f.sensor, f.mm, f.source)
            newest[k] = f.t_ns
        if new:
            relayout()
        now = time.monotonic()
        if got and panels and now - last_draw > 1 / 30:
            last_draw = now
            titles = now - last_titles > 0.5    # text layout is slow: titles twice a second
            if titles:
                last_titles = now
            for r in st.rows() if titles else [dict(port=k[0], sensor=k[1]) for k in st.last]:
                k = (r['port'], r['sensor'])
                ax, im, txt, title = panels[k]
                mm = st.last[k].astype(float).reshape(8, 8)
                mm[mm == NO_TARGET] = np.nan
                im.set_data(mm)
                if titles:
                    near = f"{r['nearest_mm']} mm" if r['nearest_mm'] is not None else 'none'
                    title.set_text(f"{k[0]}  s{k[1]}\n{r['hz']:.1f} Hz  near {near}")
                if txt:
                    for z in range(ZONES):
                        v = mm.flat[z]
                        txt[z // 8][z % 8].set_text('' if np.isnan(v) else f'{v:.0f}')
            if blit and bg:
                fig.canvas.restore_region(bg[0])
                for ax, im, txt, title in panels.values():
                    ax.draw_artist(im)
                    ax.draw_artist(title)
                    for row in txt or []:
                        for t in row:
                            ax.draw_artist(t)
                fig.canvas.blit(fig.bbox)
            meter.drew(max(newest.values()))
        if args.save:
            time.sleep(0.005)
        else:
            fig.canvas.flush_events()          # far cheaper than plt.pause()
            time.sleep(0.005)
    meter.report('view')
    if args.save:
        fig.canvas.draw()
        fig.savefig(args.save, dpi=110, bbox_inches='tight')
        print(f'[view] {len(panels)} sensor panel(s) -> {args.save}')
        return 0 if panels else 1
    if not closed:
        plt.close(fig)
    return 0


def _ramp_lut(n=256):
    """n hex colours along DIST_RAMP (index 0 = 0 mm)."""
    rgb = np.array([[int(h[i:i + 2], 16) for i in (1, 3, 5)] for h in DIST_RAMP], float)
    x = np.linspace(0, 1, len(DIST_RAMP))
    q = np.linspace(0, 1, n)
    cols = np.stack([np.interp(q, x, rgb[:, c]) for c in range(3)], 1).round().astype(int)
    return ['#%02x%02x%02x' % tuple(c) for c in cols]


def cmd_live(args):
    """Light live viewer: one Tk canvas of coloured cells (no matplotlib, nothing to install).
    Keys: n = numbers on/off, + / - = colour range, q / Esc = quit."""
    import tkinter as tk

    CELL, GAP, HEAD = args.cell, 2, 34
    lut = _ramp_lut()
    st, meter = Stats(), RenderMeter()
    gen = frames(args, idle=0)
    state = dict(max_mm=float(args.max_mm), numbers=args.numbers, last_titles=0.0, dirty=set(), newest={})
    panels = {}   # key -> dict(rects, texts, title, last colour indices)

    root = tk.Tk()
    root.title('VL53L5CX skin (live)')
    root.configure(bg='#fcfcfb')
    canvas = tk.Canvas(root, bg='#fcfcfb', highlightthickness=0, width=4 * (8 * CELL + 24), height=8 * CELL + HEAD + 40)
    canvas.pack(fill='both', expand=True)
    status = canvas.create_text(8, 8, anchor='nw', fill=INK2, font=('Segoe UI', 9),
                                text=f'waiting for frames on {_where(args)} ...   keys: n numbers, +/- range, q quit')

    def add_panel(new_key):
        """Lay every panel out again in sorted order (only when a new sensor appears)."""
        for p in panels.values():
            canvas.delete(*p['rects'], *p['texts'], p['title'])
        keys = sorted(list(panels) + [new_key], key=str)
        panels.clear()
        for i, k in enumerate(keys):
            col, row = i % args.cols, i // args.cols
            x0 = 12 + col * (8 * CELL + 24)
            y0 = 28 + row * (8 * CELL + HEAD + 16) + HEAD
            rects, texts = [], []
            for z in range(ZONES):
                r, c = divmod(z, 8)
                x, y = x0 + c * CELL, y0 + r * CELL
                rects.append(canvas.create_rectangle(x, y, x + CELL - GAP, y + CELL - GAP, width=0,
                                                     fill=NO_TARGET_COLOR))
                texts.append(canvas.create_text(x + (CELL - GAP) / 2, y + (CELL - GAP) / 2, text='', fill='white',
                                                font=('Segoe UI', max(6, CELL // 5))))
            title = canvas.create_text(x0, y0 - 6, anchor='sw', fill=INK, font=('Segoe UI', 9), text=f'{k[0]}  s{k[1]}')
            panels[k] = dict(rects=rects, texts=texts, title=title, idx=[None] * ZONES, txt=[None] * ZONES)
            if k in st.last:
                state['dirty'].add(k)
        w = 24 + min(len(panels), args.cols) * (8 * CELL + 24)
        h = 28 + math.ceil(len(panels) / args.cols) * (8 * CELL + HEAD + 16) + 8
        canvas.config(width=w, height=h)

    def draw_panel(k):
        p, mm = panels[k], st.last[k]
        scale = (len(lut) - 1) / state['max_mm']
        for z in range(ZONES):
            v = int(mm[z])
            idx = -1 if v == NO_TARGET else min(int(v * scale), len(lut) - 1)
            if idx != p['idx'][z]:                       # only touch cells that changed colour
                p['idx'][z] = idx
                canvas.itemconfigure(p['rects'][z], fill=NO_TARGET_COLOR if idx < 0 else lut[idx])
            t = (str(v) if v != NO_TARGET else '') if state['numbers'] else ''
            if t != p['txt'][z]:
                p['txt'][z] = t
                canvas.itemconfigure(p['texts'][z], text=t, fill='white' if 0 <= idx < len(lut) * 0.55 else INK)

    def tick():
        got = drain(gen)
        if got is None:
            return finish()
        for f in got:
            k = (f.port, f.sensor)
            if k not in panels:
                add_panel(k)
            st.add(f.port, f.sensor, f.mm, f.source)
            state['dirty'].add(k)
            state['newest'][k] = f.t_ns
        if state['dirty']:
            for k in state['dirty']:
                draw_panel(k)
            meter.drew(max(state['newest'][k] for k in state['dirty']))
            state['dirty'].clear()
        now = time.monotonic()
        if now - state['last_titles'] > 0.5 and panels:
            state['last_titles'] = now
            for r in st.rows():
                near = f"{r['nearest_mm']} mm" if r['nearest_mm'] is not None else 'none'
                canvas.itemconfigure(panels[(r['port'], r['sensor'])]['title'],
                                     text=f"{r['port']}  s{r['sensor']}    {r['hz']:.1f} Hz    near {near}")
            canvas.itemconfigure(status, text=f"{len(panels)} sensor(s) on {_where(args)}    colour 0-{state['max_mm']:.0f} mm"
                                              f"    keys: n numbers, +/- range, q quit")
        if args.seconds and now - meter.t0 > args.seconds:
            return finish()
        root.after(5, tick)

    def redraw_all():
        for p in panels.values():
            p['idx'] = [None] * ZONES
            p['txt'] = [None] * ZONES
        for k in panels:
            draw_panel(k)

    def key(e):
        if e.keysym in ('q', 'Escape'):
            finish()
        elif e.keysym == 'n':
            state['numbers'] = not state['numbers']; redraw_all()
        elif e.keysym in ('plus', 'equal', 'KP_Add'):
            state['max_mm'] *= 1.5; redraw_all()
        elif e.keysym in ('minus', 'KP_Subtract'):
            state['max_mm'] = max(100.0, state['max_mm'] / 1.5); redraw_all()

    def finish():
        meter.report('live')
        root.destroy()

    root.bind('<Key>', key)
    root.protocol('WM_DELETE_WINDOW', finish)
    root.after(5, tick)
    root.mainloop()
    return 0


# -- fake board ----------------------------------------------------------------------

def cmd_fake(args):
    """Send synthetic frames like a real board: each sensor free-runs on its own
    slightly-off clock (so phases drift), a target sweeps across, a few zones flicker
    invalid, and --drop loses packets. Packets carry board time + frame counters."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
    rng = np.random.default_rng(args.seed)
    print(f'[fake] {args.sensors} sensor(s) -> {args.dest}:{args.port} at ~{args.hz} Hz (Ctrl+C to stop)')
    rows, cols = np.mgrid[0:8, 0:8]
    period = 1.0 / args.hz
    ppm = rng.normal(0, 1500, args.sensors)                  # each sensor's oscillator error
    nxt = rng.uniform(0, period, args.sensors)                # independent start phases
    seq = np.zeros(args.sensors, int)
    t0 = time.monotonic()
    board0 = int(rng.integers(0, 2**32))                      # exercise counter wrap
    try:
        while args.seconds is None or time.monotonic() - t0 < args.seconds:
            i = int(np.argmin(nxt))
            time.sleep(max(0.0, t0 + nxt[i] - time.monotonic()))
            t = nxt[i]
            cx, cy = 3.5 + 3 * math.sin(t + i), 3.5 + 3 * math.cos(0.7 * t + i)
            d2 = (rows - cy) ** 2 + (cols - cx) ** 2
            mm = np.where(d2 < 6, 150 + 40 * d2, 1500 + 50 * i).astype(np.uint16)
            mm[rng.random((8, 8)) < 0.05] = NO_TARGET
            if rng.random() >= args.drop:
                s.sendto(encode_packet(i, mm, board_us=board0 + int(t * 1e6), seq=seq[i]), (args.dest, args.port))
            seq[i] += 1
            nxt[i] += period * (1 + ppm[i] * 1e-6) + rng.normal(0, 0.0003)
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
        p.add_argument('--serial', metavar='PORT', help='read a board over USB instead (e.g. COM4, /dev/ttyACM0)')
        p.add_argument('--log', action='store_true', help="with --serial: echo the board's log lines")

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

    p = sub.add_parser('live', help='light, fast live view (Tk; nothing to install)'); rx(p)
    p.add_argument('--max-mm', type=float, default=1000, help='colour scale top (mm); +/- keys change it')
    p.add_argument('--cols', type=int, default=4)
    p.add_argument('--cell', type=int, default=36, help='zone size in pixels')
    p.add_argument('--numbers', action='store_true', help='start with mm shown in each zone (n toggles)')
    p.add_argument('--seconds', type=float, help='close after N s')
    p.set_defaults(fn=cmd_live)

    p = sub.add_parser('record', help='save frames to .npz'); rx(p)
    p.add_argument('out')
    p.add_argument('--seconds', type=float)
    p.set_defaults(fn=cmd_record)

    p = sub.add_parser('timing', help='data over time + per-sensor sampling report'); rx(p)
    p.add_argument('--seconds', type=float, default=20, help='live capture length (default 20)')
    p.add_argument('--from', dest='from_npz', metavar='NPZ', help='analyse a recording instead of capturing')
    p.add_argument('--out', metavar='NPZ', help='also save the captured frames')
    p.add_argument('--save', metavar='PNG', help='write the figure')
    p.add_argument('--json', action='store_true', help='machine-readable report')
    p.add_argument('--boards', metavar='SPEC', help="which sensors share a board, for recordings that don't "
                   'store it (HIRO .npz), e.g. "71-74;75,77,81,82;76,78-80". Default: inferred from timing')
    p.set_defaults(fn=cmd_timing)

    p = sub.add_parser('fake', help='send synthetic frames (test without hardware)')
    p.add_argument('--dest', default='127.0.0.1', help=f'127.0.0.1, or {GROUP} for multicast')
    p.add_argument('--port', type=int, default=5005)
    p.add_argument('--sensors', type=int, default=4)
    p.add_argument('--hz', type=float, default=15)
    p.add_argument('--drop', type=float, default=0.0, help='fraction of packets to drop')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--seconds', type=float)
    p.set_defaults(fn=cmd_fake)

    args = ap.parse_args(argv)
    return args.fn(args)


if __name__ == '__main__':
    sys.exit(main())
