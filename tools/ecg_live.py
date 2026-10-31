"""Live plot of the ECG and heart rate streamed by max30003_node (ECG_STREAM build).

    python tools/ecg_live.py --port COM5
    python tools/ecg_live.py --port COM5 --window 8 --csv results/stream.csv

The firmware sends one line per sample at FS Hz, about 11 s behind real time
(one 10 s detector block plus 1 s margin):

    ecg:<uV>,hr:<bpm>,r:<0|1>

The same lines can be shown by any serial plotter that understands
"label:value" pairs (Arduino IDE 2, VS Code Serial Plotter, ...). This script
puts the ECG and the heart rate on separate axes and marks the R-peaks.
Other lines (start-up and status messages) are printed to the terminal.
Needs pyserial and matplotlib (pip install pyserial matplotlib).
"""
import argparse
import collections
import csv
import sys
import threading

import matplotlib.pyplot as plt
import serial
from matplotlib.animation import FuncAnimation

FS = 200  # Hz, MAX30003_FS_HZ in the firmware


def parse(line: str):
    """Return (ecg_uv, hr_bpm, r) for a stream line, or None."""
    if not line.startswith("ecg:"):
        return None
    try:
        fields = dict(part.split(":", 1) for part in line.split(","))
        return int(fields["ecg"]), float(fields["hr"]), int(fields["r"])
    except (ValueError, KeyError):
        return None


class Reader(threading.Thread):
    """Reads the serial port in the background and keeps the last window."""

    def __init__(self, port, n, writer):
        super().__init__(daemon=True)
        self.port = port
        self.writer = writer
        self.lock = threading.Lock()
        self.t = collections.deque(maxlen=n)
        self.ecg = collections.deque(maxlen=n)
        self.hr = collections.deque(maxlen=n)
        self.r = collections.deque(maxlen=n)
        self.count = 0
        self.stop = False

    def run(self):
        while not self.stop:
            raw = self.port.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            sample = parse(line)
            if sample is None:
                if line:
                    print(line)
                    if "restarted" in line or line.startswith("max30003_node"):
                        with self.lock:   # firmware restarted at sample 0
                            for d in (self.t, self.ecg, self.hr, self.r):
                                d.clear()
                            self.count = 0
                continue
            ecg, hr, r = sample
            with self.lock:
                self.t.append(self.count / FS)
                self.ecg.append(ecg)
                self.hr.append(hr if hr > 0 else float("nan"))
                self.r.append(r)
                self.count += 1
            if self.writer is not None:
                self.writer.writerow([self.count - 1, f"{(self.count - 1) / FS:.3f}", ecg, hr, r])

    def snapshot(self):
        with self.lock:
            return list(self.t), list(self.ecg), list(self.hr), list(self.r)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", required=True, help="serial port, e.g. COM5 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--window", type=float, default=10.0, help="seconds shown (default 10)")
    ap.add_argument("--csv", help="also log every sample to this CSV file")
    args = ap.parse_args()

    out = writer = None
    if args.csv:
        out = open(args.csv, "w", newline="")
        writer = csv.writer(out)
        writer.writerow(["sample", "time_s", "ecg_uv", "hr_bpm", "r_peak"])

    port = serial.Serial(args.port, args.baud, timeout=0.2)
    reader = Reader(port, int(args.window * FS), writer)
    reader.start()
    print(f"listening on {args.port} at {args.baud} baud; the first samples arrive "
          f"about 11 s after start. Close the window to stop.")

    fig, (ax_ecg, ax_hr) = plt.subplots(2, 1, sharex=True, figsize=(11, 6),
                                        gridspec_kw={"height_ratios": [3, 1]})
    (ecg_line,) = ax_ecg.plot([], [], lw=1, color="C0")
    (peak_dots,) = ax_ecg.plot([], [], "o", ms=5, color="C3", label="R-peak (rdetect)")
    (hr_line,) = ax_hr.plot([], [], lw=1.5, color="C1", drawstyle="steps-post")
    hr_text = ax_hr.text(0.01, 0.85, "", transform=ax_hr.transAxes, fontsize=12)
    ax_ecg.set_ylabel("ECG (µV)")
    ax_ecg.legend(loc="upper right")
    ax_ecg.grid(alpha=0.3)
    ax_hr.set_ylabel("HR (bpm)")
    ax_hr.set_xlabel("time since start (s)")
    ax_hr.grid(alpha=0.3)
    fig.tight_layout()

    def update(_frame):
        t, ecg, hr, r = reader.snapshot()
        if not t:
            return ecg_line, peak_dots, hr_line, hr_text
        ecg_line.set_data(t, ecg)
        pk = [i for i, v in enumerate(r) if v]
        peak_dots.set_data([t[i] for i in pk], [ecg[i] for i in pk])
        hr_line.set_data(t, hr)
        last_hr = next((h for h in reversed(hr) if h == h), None)
        hr_text.set_text(f"{last_hr:.1f} bpm" if last_hr is not None else "– bpm")

        right = max(t[-1], args.window)
        ax_ecg.set_xlim(right - args.window, right)
        lo, hi = min(ecg), max(ecg)
        pad = max(50, 0.1 * (hi - lo))
        ax_ecg.set_ylim(lo - pad, hi + pad)
        valid = [h for h in hr if h == h]
        if valid:
            ax_hr.set_ylim(min(40, min(valid) - 5), max(140, max(valid) + 5))
        return ecg_line, peak_dots, hr_line, hr_text

    _anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    reader.stop = True
    reader.join(timeout=1)
    port.close()
    if out is not None:
        out.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
