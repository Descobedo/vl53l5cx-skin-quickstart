"""ToF theremin: hand distance sets the volume (closer = louder), hand position sets the note.

    python examples/theremin.py --serial COM4          # board over USB
    python examples/theremin.py                         # board over Wi-Fi (ports 5004-5016)
    python examples/theremin.py --serial COM4 --sensors 0 1 2   # forward sensors, left to right

Keep your hands away for the first 2 s: it learns what every zone normally sees.
Sensors that see something close in most zones (e.g. one facing the table) are ignored,
and so is anything that was already there. Only things closer than that background count.

Pitch comes from where the hand is across the forward sensors, left to right (sensor
order, then zone column), snapped to a pentatonic scale. --flip reverses it if it feels
backwards on your mounting. Keys in the window: c = recalibrate, f = flip, q = quit.

Needs numpy, sounddevice (pip install sounddevice) and, for --serial, pyserial.
"""
import argparse
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
import tof  # noqa: E402

RATE = 44100
NOTE_NAMES = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
SCALES = {'pentatonic': [0, 2, 4, 7, 9], 'major': [0, 2, 4, 5, 7, 9, 11], 'chromatic': list(range(12))}


def scale_notes(root_midi, octaves, scale):
    steps = SCALES[scale]
    return [root_midi + 12 * o + s for o in range(octaves) for s in steps] + [root_midi + 12 * octaves]


def midi_hz(m):
    return 440.0 * 2 ** ((m - 69) / 12)


def midi_name(m):
    return f'{NOTE_NAMES[m % 12]}{m // 12 - 1}'


class Synth:
    """A sine with a little 2nd/3rd harmonic. Pitch glides and volume ramps per sample,
    so changes never click."""

    def __init__(self, glide_s=0.04, fade_s=0.03):
        self.freq = self.target_freq = 261.63
        self.amp = self.target_amp = 0.0
        self.phase = 0.0
        self.k_f = 1 - math.exp(-1 / (glide_s * RATE))
        self.k_a = 1 - math.exp(-1 / (fade_s * RATE))

    def render(self, n):
        out = np.empty(n, np.float32)
        f, a, ph = self.freq, self.amp, self.phase
        tf, ta, kf, ka = self.target_freq, self.target_amp, self.k_f, self.k_a
        # per-sample smoothing, vectorised as a geometric approach to the target
        decay_f = (1 - kf) ** np.arange(1, n + 1)
        decay_a = (1 - ka) ** np.arange(1, n + 1)
        freqs = tf + (f - tf) * decay_f
        amps = ta + (a - ta) * decay_a
        phases = ph + 2 * np.pi * np.cumsum(freqs) / RATE
        out[:] = amps * (np.sin(phases) + 0.25 * np.sin(2 * phases) + 0.1 * np.sin(3 * phases)) / 1.35
        self.freq, self.amp, self.phase = freqs[-1], amps[-1], phases[-1] % (2 * np.pi)
        return out


class Hand:
    """Background model + hand estimate from the forward sensors."""

    def __init__(self, args):
        self.args = args
        self.calib = {}           # key -> list of (64,) frames during calibration
        self.baseline = {}        # key -> (64,) mm, 4000 where the zone normally sees nothing
        self.forward = []         # keys used for the hand, left to right
        self.ignored = {}         # key -> reason
        self.last = {}            # key -> latest (64,) mm
        self.t_calib_end = None
        self.done = False

    def start_calibration(self):
        """The clock starts at the first frame, so a board that is still booting doesn't eat it."""
        self.calib, self.baseline, self.forward, self.ignored = {}, {}, [], {}
        self.t_calib_end, self.done = None, False

    @property
    def waiting(self):
        return not self.done and self.t_calib_end is None

    @property
    def calibrating(self):
        return not self.done and self.t_calib_end is not None

    def add(self, f):
        k = (f.port, f.sensor)
        mm = f.mm.astype(float)
        mm[mm == tof.NO_TARGET] = tof.FIRMWARE_NO_TARGET_MM
        self.last[k] = mm
        if self.done:
            return
        now = time.monotonic()
        if self.t_calib_end is None:
            self.t_calib_end = now + self.args.calib
        if now < self.t_calib_end:
            self.calib.setdefault(k, []).append(mm)
        else:
            self.finish_calibration()
            self.done = True

    def finish_calibration(self):
        keys = sorted(self.calib, key=str)
        for k in keys:
            self.baseline[k] = np.median(np.stack(self.calib[k]), axis=0)
        if self.args.sensors:
            want = [s for s in self.args.sensors]
            self.forward = [k for s in want for k in keys if k[1] == s]
            for k in keys:
                if k not in self.forward:
                    self.ignored[k] = 'not in --sensors'
        else:
            for k in keys:
                near = (self.baseline[k] < self.args.blocked_mm).mean()
                if near > 0.5:
                    self.ignored[k] = f'sees a surface in {near:.0%} of zones (table?)'
                else:
                    self.forward.append(k)
        print('[theremin] calibrated. forward (left to right): '
              + (', '.join(f's{k[1]}' for k in self.forward) or 'none')
              + ''.join(f'\n           ignored s{k[1]}: {why}' for k, why in self.ignored.items()), file=sys.stderr)

    def estimate(self):
        """-> (distance mm, x in 0..1) of the hand, or None."""
        n = len(self.forward)
        if not n:
            return None
        ds, xs = [], []
        for i, k in enumerate(self.forward):
            if k not in self.last:
                continue
            mm, base = self.last[k], self.baseline[k]
            fg = (mm < tof.FIRMWARE_NO_TARGET_MM) & (mm < base - np.maximum(self.args.margin_mm, 0.1 * base)) \
                & (mm < self.args.far_mm)
            if not fg.any():
                continue
            z = np.flatnonzero(fg)
            ds.append(mm[z])
            col = z % 8
            xs.append(np.stack([(i * 8 + col + 0.5) / (8 * n), mm[z]], 1))
        if not ds:
            return None
        d = np.concatenate(ds)
        if len(d) < self.args.min_zones:
            return None
        dist = float(np.percentile(d, 20))                     # near edge of the hand, robust to one noisy zone
        pts = np.concatenate(xs)
        w = 1.0 / np.maximum(pts[:, 1], 30.0)                  # nearer zones count more
        x = float((pts[:, 0] * w).sum() / w.sum())
        return dist, (1 - x if self.args.flip else x)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--serial', metavar='PORT', help='board over USB (e.g. COM4)')
    ap.add_argument('--ports', default=tof.DEFAULT_PORTS, help='UDP ports when not using --serial')
    ap.add_argument('--iface', default='0.0.0.0')
    ap.add_argument('--no-multicast', action='store_true')
    ap.add_argument('--sensors', type=int, nargs='*', help='forward sensors, left to right (default: auto)')
    ap.add_argument('--flip', action='store_true', help='reverse the pitch direction')
    ap.add_argument('--near-mm', type=float, default=40, help='full volume at this distance')
    ap.add_argument('--far-mm', type=float, default=450, help='silent beyond this distance')
    ap.add_argument('--root', default='C4', help='lowest note, e.g. C4, A3')
    ap.add_argument('--octaves', type=int, default=2)
    ap.add_argument('--scale', choices=sorted(SCALES), default='pentatonic')
    ap.add_argument('--calib', type=float, default=2.0, help='seconds to learn the background')
    ap.add_argument('--blocked-mm', type=float, default=150, help='a sensor seeing this close in most zones is ignored')
    ap.add_argument('--margin-mm', type=float, default=60, help='how much closer than background counts as a hand')
    ap.add_argument('--min-zones', type=int, default=2, help='zones needed to count as a hand')
    ap.add_argument('--volume', type=float, default=0.5, help='master volume 0..1')
    ap.add_argument('--headless', action='store_true', help='no window, no sound: print what it would play')
    ap.add_argument('--seconds', type=float, help='stop after N s')
    args = ap.parse_args()
    args.log = False

    name = args.root.upper()
    root_midi = 12 * (int(name[-1]) + 1) + NOTE_NAMES.index(name[:-1])
    notes = scale_notes(root_midi, args.octaves, args.scale)

    synth, hand = Synth(), Hand(args)
    gen = tof.frames(args, idle=0)
    hand.start_calibration()
    state = dict(note=None, vol=0.0, x=None, dist=None, last_seen=0.0, x_smooth=None, t0=time.monotonic())

    def update():
        got = tof.drain(gen)
        if got is None:
            return False
        for f in got:
            hand.add(f)
        if not hand.done:
            synth.target_amp = 0.0
            return True
        est = hand.estimate()
        now = time.monotonic()
        if est is not None:
            dist, x = est
            state['last_seen'] = now
            xs = state['x_smooth']
            state['x_smooth'] = x if xs is None else 0.6 * xs + 0.4 * x
            idx = min(int(state['x_smooth'] * len(notes)), len(notes) - 1)
            # hysteresis: only change note once the hand is well into the next one
            if state['note'] is not None and abs(state['x_smooth'] * len(notes) - (notes.index(state['note']) + 0.5)) < 0.7:
                idx = notes.index(state['note'])
            state['note'] = notes[idx]
            closeness = np.clip((args.far_mm - dist) / (args.far_mm - args.near_mm), 0, 1)
            state['vol'] = float(closeness ** 1.5)
            state['x'], state['dist'] = state['x_smooth'], dist
            synth.target_freq = midi_hz(state['note'])
            synth.target_amp = args.volume * state['vol']
        elif now - state['last_seen'] > 0.25:                  # hand gone: fade out
            synth.target_amp = 0.0
            state['vol'], state['dist'] = 0.0, None
        return True

    if args.headless:
        last_print = 0.0
        while args.seconds is None or time.monotonic() - state['t0'] < args.seconds:
            if not update():
                break
            synth.render(441)                                   # keep the synth state moving
            if time.monotonic() - last_print > 0.25 and hand.done:
                last_print = time.monotonic()
                d = f"{state['dist']:.0f} mm" if state['dist'] else '   -   '
                print(f"note {midi_name(state['note']) if state['note'] else '-':>4}  vol {state['vol']:.2f}  "
                      f"hand {d}  x {state['x'] if state['x'] is not None else float('nan'):.2f}")
            time.sleep(0.01)
        return 0

    import sounddevice as sd
    import tkinter as tk

    stream = sd.OutputStream(samplerate=RATE, channels=1, dtype='float32', blocksize=256, latency='low',
                             callback=lambda out, n, t, st: out.__setitem__((slice(None), 0), synth.render(n)))
    stream.start()

    root = tk.Tk()
    root.title('ToF theremin')
    root.configure(bg='#fcfcfb')
    W, H = 640, 300
    cv = tk.Canvas(root, width=W, height=H, bg='#fcfcfb', highlightthickness=0)
    cv.pack()
    note_txt = cv.create_text(W / 2, 60, text='', font=('Segoe UI', 44, 'bold'), fill=tof.INK)
    info_txt = cv.create_text(W / 2, 112, text='', font=('Segoe UI', 11), fill=tof.INK2)
    # keyboard strip: one cell per note, hand marker on top
    kx0, kx1, ky0, ky1 = 20, W - 20, 150, 200
    cw = (kx1 - kx0) / len(notes)
    keys = [cv.create_rectangle(kx0 + i * cw, ky0, kx0 + (i + 1) * cw - 2, ky1, width=0, fill='#e4e3df')
            for i in range(len(notes))]
    for i, m in enumerate(notes):
        cv.create_text(kx0 + (i + 0.5) * cw, ky1 + 12, text=midi_name(m), font=('Segoe UI', 8), fill=tof.INK2)
    marker = cv.create_line(0, ky0 - 8, 0, ky1 + 2, width=3, fill=tof.SERIES[1], state='hidden')
    cv.create_text(20, 240, anchor='w', text='volume', font=('Segoe UI', 9), fill=tof.INK2)
    cv.create_rectangle(80, 232, W - 20, 248, width=0, fill='#e4e3df')
    vol_bar = cv.create_rectangle(80, 232, 80, 248, width=0, fill=tof.SERIES[0])
    sens_txt = cv.create_text(20, 278, anchor='w', text='', font=('Segoe UI', 9), fill=tof.INK2)

    def tick():
        if not update() or (args.seconds and time.monotonic() - state['t0'] > args.seconds):
            return quit_()
        if hand.waiting:
            cv.itemconfigure(note_txt, text='no data')
            cv.itemconfigure(info_txt, text=f"waiting for frames on {args.serial or 'ports ' + args.ports}"
                                            " (a board just plugged in takes ~6 s)")
        elif hand.calibrating:
            cv.itemconfigure(note_txt, text='hands away')
            cv.itemconfigure(info_txt, text=f'learning the background ... '
                                            f'{max(0.0, hand.t_calib_end - time.monotonic()):.1f} s')
        elif not hand.forward:
            cv.itemconfigure(note_txt, text='no forward sensors')
            cv.itemconfigure(info_txt, text='every sensor sees a surface: press c to recalibrate, or use --sensors')
        else:
            playing = state['vol'] > 0.01 and state['note'] is not None
            cv.itemconfigure(note_txt, text=midi_name(state['note']) if playing else '·')
            cv.itemconfigure(info_txt, text=(f"{state['dist']:.0f} mm   {midi_hz(state['note']):.0f} Hz"
                                             if playing else 'move a hand in front of the forward sensors'))
            for i, r in enumerate(keys):
                cv.itemconfigure(r, fill=tof.SERIES[0] if playing and notes[i] == state['note'] else '#e4e3df')
            if playing:
                mx = kx0 + state['x'] * (kx1 - kx0)
                cv.coords(marker, mx, ky0 - 8, mx, ky1 + 2)
                cv.itemconfigure(marker, state='normal')
            else:
                cv.itemconfigure(marker, state='hidden')
            cv.coords(vol_bar, 80, 232, 80 + state['vol'] * (W - 100), 248)
            cv.itemconfigure(sens_txt, text='forward: ' + (' '.join(f's{k[1]}' for k in hand.forward) or 'none')
                             + ('   ignored: ' + ' '.join(f's{k[1]}' for k in hand.ignored) if hand.ignored else '')
                             + ('   (flipped)' if args.flip else '') + '      c recalibrate   f flip   q quit')
        root.after(10, tick)

    def key(e):
        if e.keysym in ('q', 'Escape'):
            quit_()
        elif e.keysym == 'c':
            synth.target_amp = 0.0
            hand.start_calibration()
        elif e.keysym == 'f':
            args.flip = not args.flip

    def quit_():
        synth.target_amp = 0.0
        time.sleep(0.05)
        stream.stop(); stream.close()
        root.destroy()

    root.bind('<Key>', key)
    root.protocol('WM_DELETE_WINDOW', quit_)
    root.after(10, tick)
    root.mainloop()
    return 0


if __name__ == '__main__':
    sys.exit(main())
