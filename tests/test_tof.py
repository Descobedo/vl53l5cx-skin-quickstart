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
    test_roundtrip(); test_rays(); test_fake_board_end_to_end()
    print('all tests passed')
