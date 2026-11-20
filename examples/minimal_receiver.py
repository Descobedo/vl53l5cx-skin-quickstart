"""Smallest useful receiver: drop this into another project. Needs only numpy.

    python examples/minimal_receiver.py 5005
"""
import socket
import struct
import sys

import numpy as np

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('0.0.0.0', PORT))
# Join the multicast group the boards send to (harmless if they send unicast).
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                struct.pack('4s4s', socket.inet_aton('239.0.0.1'), socket.inet_aton('0.0.0.0')))

while True:
    data, (src, _) = sock.recvfrom(2048)
    if len(data) < 129:
        continue
    sensor = data[0]
    mm = np.frombuffer(data, '<u2', 64, 1).reshape(8, 8)      # row-major, zone k = (k//8, k%8)
    valid = mm[mm < 4000]                                     # 4000 = no target
    print(f'{src} sensor {sensor}: nearest {valid.min() if valid.size else "none"} mm')
