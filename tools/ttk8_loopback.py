#!/usr/bin/env python3
"""
PC side of the TTK8 sensor board loopback test (table 2, tests 2, 3, 5, 6).

Test 5 (generated and received signal match) is judged by eye in the plot.

The MCU (firmware/ecg_node built with TTK8_LOOPBACK, see Core/Src/ttk8_test.c)
generates a synthetic ECG on its DAC, reads it back through the MAX30003 and
streams both over the ST-LINK virtual COM port, followed by the R-peaks and the
heart rate its detector found. This script records one run, saves the raw log,
and plots the generated signal, the received signal, the detected peaks and
the estimated heart rate in one figure.

It also checks the acceptance criteria:
    2  estimated heart rate within 5 % of the generated rate
    3  all samples of the run arrived
    6  MAX30003 identified and configured over SPI

Usage:
    python tools/ttk8_loopback.py --port COM5            # then reset the MCU or press USER
    python tools/ttk8_loopback.py --replay results/ttk8/loopback_<time>.txt
"""

import argparse
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "python" / "scripts"))

DEFAULT_PORT = "COM5"
DEFAULT_BAUD = 115200
DEFAULT_OUT_DIR = REPO / "results" / "ttk8"
MAX_LAG_S = 1.0           # search window for the delay through the AFE
MAX30003_VREF_V = 1.0     # ECG LSB = VREF / (2^17 * gain)
HR_TOLERANCE = 0.05       # criterion 2


@dataclass
class Run:
    info: dict = field(default_factory=dict)
    gen: dict = field(default_factory=dict)       # sample index -> microvolt
    rx: dict = field(default_factory=dict)        # sample index -> MAX30003 code
    peaks: list = field(default_factory=list)
    hr_bpm: float | None = None
    counters: dict = field(default_factory=dict)
    errors: list = field(default_factory=list)
    bad_lines: int = 0
    complete: bool = False


def parse_line(run: Run, line: str) -> None:
    """Add one record of the MCU output to run. See ttk8_test.c for the format."""
    parts = line.strip().split(",")
    tag = parts[0]
    try:
        if tag == "G":
            run.gen[int(parts[1])] = int(parts[2])
        elif tag == "E":
            run.rx[int(parts[1])] = int(parts[2])
        elif tag == "P":
            run.peaks.append(int(parts[1]))
        elif tag == "H":
            run.hr_bpm = float(parts[1])
        elif tag == "I":
            run.info[parts[1]] = parts[2]
        elif tag == "X":
            run.counters[parts[1]] = int(parts[2])
        elif tag == "ERR":
            run.errors.append(",".join(parts[1:]))
        elif tag == "END":
            run.complete = True
        elif tag in ("S", "") or tag.startswith("#"):
            pass
        else:
            run.bad_lines += 1
    except (IndexError, ValueError):
        run.bad_lines += 1


def parse_log(lines) -> Run:
    """Parse the last complete run in a log (a run starts at '#TTK8')."""
    run = None
    for line in lines:
        if line.startswith("#TTK8"):
            run = Run()
        elif run is not None:
            parse_line(run, line)
    if run is None:
        sys.exit("no '#TTK8' header found in the log")
    return run


def record(port: str, baud: int, log_path: Path, timeout_s: float) -> Run:
    import serial  # only needed for a live run

    log_path.parent.mkdir(parents=True, exist_ok=True)
    run = None
    with serial.Serial(port, baud, timeout=1) as ser, open(log_path, "w", newline="\n") as log:
        print(f"Listening on {port} at {baud} baud. Reset the MCU or press USER to start a run.")
        t_last = time.monotonic()
        n_rx_reported = 0
        while True:
            raw = ser.readline()
            if not raw:
                if time.monotonic() - t_last > timeout_s:
                    sys.exit(f"no data for {timeout_s:.0f} s, giving up (log in {log_path})")
                continue
            t_last = time.monotonic()
            line = raw.decode("ascii", errors="replace").strip()
            if line.startswith("#TTK8"):
                if run is not None:
                    print("New run started before the previous one ended; discarding it.")
                run = Run()
                log.seek(0)
                log.truncate()
            if run is None:
                continue
            log.write(line + "\n")
            parse_line(run, line)
            if line.startswith(("I,", "ERR")):
                print(line)
            if line == "I,regs,ok":
                print("Capturing; the data follows after the capture (30 s plus a short wait).")
            n_rx = len(run.rx)
            if n_rx - n_rx_reported >= 1000:
                n_rx_reported = n_rx
                total = int(run.info.get("samples", 0)) or "?"
                print(f"  {n_rx}/{total} samples")
            if run.complete:
                break
    print(f"Raw log saved to {log_path}")
    return run


def to_array(samples: dict, n: int) -> np.ndarray:
    """Index-keyed samples to an array; missing samples become NaN."""
    a = np.full(n, np.nan)
    for i, v in samples.items():
        if 0 <= i < n:
            a[i] = v
    return a


def best_lag(gen: np.ndarray, rx: np.ndarray, max_lag: int) -> tuple[int, float]:
    """Delay (samples) of rx relative to gen that maximises |Pearson r|, and r there.
    Only used to line the two signals up in the plot."""
    best = (0, 0.0)
    for lag in range(0, max_lag + 1):
        r = correlation(gen, rx, lag)
        if abs(r) > abs(best[1]):
            best = (lag, r)
    return best


def correlation(gen: np.ndarray, rx: np.ndarray, lag: int) -> float:
    g = gen[: len(gen) - lag] if lag else gen
    x = rx[lag:]
    ok = ~(np.isnan(g) | np.isnan(x))
    if ok.sum() < 10:
        return 0.0
    return float(np.corrcoef(g[ok], x[ok])[0, 1])


def evaluate(run: Run) -> dict:
    fs = float(run.info.get("fs", 200))
    n = int(run.info.get("samples", max(len(run.rx), len(run.gen))))
    gain = float(run.info.get("gain", 20))
    bpm_ref = float(run.info.get("bpm", "nan"))

    gen_uv = to_array(run.gen, n)
    rx_uv = to_array(run.rx, n) * MAX30003_VREF_V * 1e6 / (2**17 * gain)

    lag, _ = best_lag(gen_uv, rx_uv, int(MAX_LAG_S * fs))
    hr = run.hr_bpm
    hr_err = abs(hr - bpm_ref) / bpm_ref if hr is not None else None
    info_ok = run.info.get("regs") == "ok"

    return {
        "fs": fs, "n": n, "bpm_ref": bpm_ref, "gen_uv": gen_uv, "rx_uv": rx_uv,
        "lag": lag, "hr": hr, "hr_err": hr_err,
        "missing_rx": int(np.isnan(rx_uv).sum()), "missing_gen": int(np.isnan(gen_uv).sum()),
        "pass": {
            "2 heart rate": hr_err is not None and hr_err <= HR_TOLERANCE,
            "3 full stream": run.complete and not np.isnan(rx_uv).any() and not np.isnan(gen_uv).any(),
            "6 SPI": info_ok and len(run.rx) > 0,
        },
    }


def report(run: Run, ev: dict) -> None:
    print()
    print(f"MAX30003 INFO      {run.info.get('info', '?')}  registers {run.info.get('regs', 'not verified')}")
    print(f"samples            generated {len(run.gen)}, received {len(run.rx)} of {ev['n']}")
    print(f"delay through AFE  {ev['lag']} samples ({1000 * ev['lag'] / ev['fs']:.0f} ms)")
    print(f"R-peaks            {len(run.peaks)}")
    if ev["hr"] is not None:
        print(f"heart rate         {ev['hr']:.2f} bpm, reference {ev['bpm_ref']:.0f} bpm, "
              f"error {100 * ev['hr_err']:.2f} %")
    for k, v in run.counters.items():
        print(f"{k:<18} {v}")
    for e in run.errors:
        print(f"MCU error          {e}")
    if run.bad_lines:
        print(f"unparsed lines     {run.bad_lines}")
    print()
    for name, ok in ev["pass"].items():
        print(f"criterion {name:<16} {'PASS' if ok else 'FAIL'}")


def plot(run: Run, ev: dict, out: Path, show: bool, window: tuple[float, float | None]) -> None:
    try:
        from thesis_style import apply_thesis_style
        apply_thesis_style()
    except ImportError:
        pass

    fs, lag = ev["fs"], ev["lag"]
    t = np.arange(ev["n"]) / fs
    peaks = np.array([p for p in run.peaks if 0 <= p < ev["n"]], dtype=int)

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(t + lag / fs, ev["gen_uv"] / 1000, color="0.6", lw=1.6,
            label=f"generated (DAC, shifted {1000 * lag / fs:.0f} ms)")
    ax.plot(t, ev["rx_uv"] / 1000, color="C0", lw=0.9, label="received (MAX30003)")
    if len(peaks):
        ax.plot(t[peaks], ev["rx_uv"][peaks] / 1000, "v", color="C3", ms=6, label="detected R-peaks")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("ECG input [mV]")
    t_end = t[-1] if len(t) else 1.0
    ax.set_xlim(window[0], min(window[1], t_end) if window[1] else t_end)
    # Leave the top of the axes free for the heart-rate traces.
    lo, hi = np.nanmin(ev["rx_uv"]) / 1000, np.nanmax(ev["rx_uv"]) / 1000
    if np.isfinite(lo) and np.isfinite(hi) and hi > lo:
        ax.set_ylim(lo - 0.05 * (hi - lo), hi + 0.7 * (hi - lo))

    hr_ax = ax.twinx()
    hr_ax.grid(False)
    ref = ev["bpm_ref"]
    hr_ax.axhspan(ref * (1 - HR_TOLERANCE), ref * (1 + HR_TOLERANCE), color="C2", alpha=0.12, lw=0,
                  label=f"reference {ref:.0f} bpm \u00b1{100 * HR_TOLERANCE:.0f} %")
    if len(peaks) >= 2:
        rr = np.diff(peaks) / fs
        hr_ax.step(t[peaks[1:]], 60 / rr, where="post", color="C2", lw=1.0, label="instantaneous HR")
    if ev["hr"] is not None:
        hr_ax.axhline(ev["hr"], color="C1", ls="--", lw=1.2, label=f"estimated HR (MCU) {ev['hr']:.2f} bpm")
    hr_ax.set_ylabel("Heart rate [bpm]")
    hr_ax.set_ylim(ref * 0.4, ref * 1.25)

    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = hr_ax.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc="upper center", bbox_to_anchor=(0.5, -0.14), ncol=3, fontsize=8)

    err = f"{100 * ev['hr_err']:.2f} %" if ev["hr_err"] is not None else "n/a"
    ax.set_title(f"MAX30003 loopback: HR error = {err}")
    fig.tight_layout()

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out)
    print(f"Figure saved to {out}")
    if show:
        plt.show()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=DEFAULT_PORT, help=f"ST-LINK virtual COM port (default {DEFAULT_PORT})")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"baud rate (default {DEFAULT_BAUD})")
    ap.add_argument("--replay", type=Path, help="parse a saved log instead of reading the serial port")
    ap.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR, help="where the log and figure go")
    ap.add_argument("--timeout", type=float, default=120.0, help="seconds without data before giving up")
    ap.add_argument("--format", default="pdf", help="figure format (pdf, png, ...)")
    ap.add_argument("--start", type=float, default=0.0, help="start of the plotted window in s")
    ap.add_argument("--end", type=float, default=None, help="end of the plotted window in s (default: whole run)")
    ap.add_argument("--no-show", action="store_true", help="only save the figure")
    args = ap.parse_args()

    if args.replay:
        with open(args.replay) as f:
            run = parse_log(f)
        stem = args.replay.stem
    else:
        stem = "loopback_" + time.strftime("%Y%m%d_%H%M%S")
        run = record(args.port, args.baud, args.out_dir / f"{stem}.txt", args.timeout)

    ev = evaluate(run)
    report(run, ev)
    plot(run, ev, args.out_dir / f"{stem}.{args.format}", show=not args.no_show,
         window=(args.start, args.end))


if __name__ == "__main__":
    main()
