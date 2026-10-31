"""Read R-peaks and pulse reported by max30003_node over the ST-LINK virtual COM port.

    python tools/ecg_monitor.py --port COM5
    python tools/ecg_monitor.py --port COM5 --csv results/peaks.csv

The firmware prints one line per detector block:

    block <k> @ <first>: <n> samples, <count> peaks, pulse <bpm|-> bpm: <p1> <p2> ...

where the peaks are running sample indices at FS Hz since start (or since the
last press of the USER button), and, with MAX30003_RTOR_ENABLE, one line per
beat from the MAX30003's own R-to-R detector:

    rtor <ms> ms, <bpm> bpm

Every line is echoed. With --csv, each R-peak is appended as
block,sample,time_s,pulse_bpm. Needs pyserial (pip install pyserial).
"""
import argparse
import csv
import re
import sys

import serial

FS = 200  # Hz, MAX30003_FS_HZ / RDETECT_FS_HZ in the firmware

BLOCK_RE = re.compile(
    r"block (\d+) @ (\d+): (\d+) samples, (\d+) peaks, pulse ([\d.]+|-) bpm:(.*)")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", required=True, help="serial port, e.g. COM5 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--csv", help="append every R-peak to this CSV file")
    args = ap.parse_args()

    out = None
    writer = None
    if args.csv:
        out = open(args.csv, "a", newline="")
        writer = csv.writer(out)
        if out.tell() == 0:
            writer.writerow(["block", "sample", "time_s", "pulse_bpm"])

    with serial.Serial(args.port, args.baud, timeout=1) as port:
        print(f"listening on {args.port} at {args.baud} baud, Ctrl+C to stop")
        try:
            while True:
                line = port.readline().decode("ascii", errors="replace").strip()
                if not line:
                    continue
                print(line)
                m = BLOCK_RE.match(line)
                if m and writer is not None:
                    block, pulse = int(m.group(1)), m.group(5)
                    for p in m.group(6).split():
                        s = int(p)
                        writer.writerow([block, s, f"{s / FS:.3f}", "" if pulse == "-" else pulse])
                    out.flush()
        except KeyboardInterrupt:
            pass
    if out is not None:
        out.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
