"""python -m pytest tests  (or: python tests/test_tof.py). No hardware needed."""
import json
import os
import subprocess
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
import tof  # noqa: E402

TOF = os.path.join(ROOT, 'tools', 'tof.py')


def test_roundtrip():
    mm = np.arange(64, dtype=np.uint16) * 10
    mm[5] = 0
    pkt = tof.encode_packet(3, mm)
    assert len(pkt) == 257
    idx, got, raw = tof.parse_packet(pkt)
    assert idx == 3 and (got == mm).all()
    assert raw[5] == 4000                       # firmware's no-target value on the wire
    assert tof.parse_packet(pkt[:129])[2] is None   # short packets: no raw block
    assert tof.parse_packet(pkt[:100]) is None


def test_meta_trailer():
    pkt = tof.encode_packet(1, np.zeros(64), board_us=2**32 + 5, seq=70000)
    assert len(pkt) == 263
    assert tof.parse_meta(pkt) == (5, 70000 % 2**16)
    assert tof.parse_meta(pkt[:257]) == (None, None)


def test_timing_analysis():
    # two sensors, 15 Hz and 1% slower, one lost frame, board clock wrapping mid-run
    rec = {k: [] for k in ('timestamps', 'ports', 'sensor_ids', 'readings_mm', 'sources', 'board_us', 'seq', 'has_meta')}
    for sid, period in ((0, 1 / 15), (1, 1.01 / 15)):
        for k in range(150):
            if sid == 1 and k == 50:
                continue
            t = 0.01 * sid + k * period
            rec['timestamps'].append(int(1e18 + t * 1e9)); rec['ports'].append(5005); rec['sensor_ids'].append(sid)
            rec['readings_mm'].append(np.full(64, 500)); rec['sources'].append('x')
            rec['board_us'].append((2**32 - 3_000_000 + int(t * 1e6)) % 2**32); rec['seq'].append(k); rec['has_meta'].append(True)
    rec = {k: np.array(v) for k, v in rec.items()}
    an = tof.analyze(rec)
    s0, s1 = an['sensors']
    assert an['clock'] == 'board'
    assert abs(s0['rate_hz'] - 15) < 0.05 and abs(s1['rate_hz'] - 15 / 1.01) < 0.05
    assert s1['lost'] == 1 and s1['gaps'] == 1
    assert abs(s1['drift_ms_per_s'] - 1000 * 0.01 / 1.01) < 0.5   # slower sensor drifts later: ~ +9.9 ms/s


def test_rays():
    r = tof.zone_rays()
    assert r.shape == (64, 3) and np.allclose(np.linalg.norm(r, axis=1), 1)


def test_fake_board_end_to_end():
    port = '5099'
    fake = subprocess.Popen([sys.executable, TOF, 'fake', '--port', port, '--sensors', '3', '--seconds', '4'],
                            stdout=subprocess.DEVNULL)
    try:
        out = subprocess.run([sys.executable, TOF, 'status', '--ports', port, '--seconds', '2', '--json'],
                             capture_output=True, text=True, timeout=30)
    finally:
        fake.wait(timeout=10)
    assert out.returncode == 0, out.stderr
    sensors = json.loads(out.stdout)['sensors']
    assert [s['sensor'] for s in sensors] == [0, 1, 2]
    assert all(s['hz'] > 5 for s in sensors), sensors


if __name__ == '__main__':
    test_roundtrip(); test_meta_trailer(); test_timing_analysis(); test_rays(); test_fake_board_end_to_end()
    print('all tests passed')
