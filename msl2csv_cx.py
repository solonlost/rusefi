#!/usr/bin/env python3
"""
Convert a TunerStudio MSL cranking datalog into a rusEFI unit-test CSV
(Time [s],crank,cam edge list) for test_real_citroen_cx.cpp.

MSL logs contain sampled counters, not edge timestamps: this script
distributes the counted crank teeth evenly across each sample interval,
so tooth COUNTS are real (from the log) but intra-interval timing is
synthetic. Fine for decoder-logic testing; use a TunerStudio composite
tooth-logger capture if edge-accurate timing is ever needed.

Usage:
    ./msl2csv_cx.py input.msl output.csv
    ./msl2csv_cx.py --synthetic output.csv   # ideal 200 RPM cranking, no MSL needed

Expected MSL columns (case-insensitive substring match):
    Time, triggerPrimaryRise, vvtEventRiseCounter 1
"""

import sys
import csv


def find_col(header, *needles):
    for i, name in enumerate(header):
        n = name.strip().lower()
        if all(x.lower() in n for x in needles):
            return i
    raise SystemExit(f"column matching {needles} not found in: {header}")


def emit_edges(rows_out, t0, t1, count, channel_states, channel):
    """Distribute `count` rise+fall pairs evenly in (t0, t1] on `channel`."""
    if count <= 0:
        return
    dt = (t1 - t0) / count
    for k in range(count):
        rise = t0 + (k + 0.5) * dt
        fall = rise + dt * 0.4
        channel_states[channel] = 1
        rows_out.append((rise, tuple(channel_states)))
        channel_states[channel] = 0
        rows_out.append((fall, tuple(channel_states)))


def convert_msl(msl_path, csv_path):
    with open(msl_path, "r", errors="replace") as f:
        lines = [ln.rstrip("\n") for ln in f]

    # MSL: comment lines, then a header row, then a units row, then data (tab-separated)
    header_idx = None
    for i, ln in enumerate(lines):
        if "\t" in ln and "time" in ln.lower():
            header_idx = i
            break
    if header_idx is None:
        raise SystemExit("no header row found")

    header = lines[header_idx].split("\t")
    c_time = find_col(header, "time")
    c_crank = find_col(header, "triggerprimaryrise")
    c_cam = find_col(header, "vvteventrisecounter", "1")

    samples = []
    for ln in lines[header_idx + 1:]:
        parts = ln.split("\t")
        if len(parts) <= max(c_time, c_crank, c_cam):
            continue
        try:
            samples.append((float(parts[c_time]),
                            int(float(parts[c_crank])),
                            int(float(parts[c_cam]))))
        except ValueError:
            continue  # units row / garbage

    rows_out = [(0.0, (0, 0))]
    states = [0, 0]
    for (t0, crank0, cam0), (t1, crank1, cam1) in zip(samples, samples[1:]):
        d_crank = crank1 - crank0
        d_cam = cam1 - cam0
        if d_crank < 0 or d_cam < 0:
            continue  # counter wrap/reset; skip interval
        emit_edges(rows_out, t0, t1, d_crank, states, 0)
        emit_edges(rows_out, t0, t1, d_cam, states, 1)

    write_csv(csv_path, rows_out)


def synthetic(csv_path, rpm=200, revolutions=8):
    """Ideal 145-tooth crank + once-per-720 cam at constant cranking speed.
    Cam rise placed between crank teeth ~40% into revolution pairs 1,3,5..."""
    rows_out = [(0.0, (0, 0))]
    states = [0, 0]
    rev_period = 60.0 / rpm
    tooth_dt = rev_period / 145
    t = 0.05
    for rev in range(revolutions):
        for tooth in range(145):
            rise = t
            fall = t + tooth_dt * 0.4
            states[0] = 1
            rows_out.append((rise, tuple(states)))
            states[0] = 0
            rows_out.append((fall, tuple(states)))
            # cam pulse once per 720 deg, mid-tooth-58 of odd revolutions
            if rev % 2 == 1 and tooth == 58:
                cam_rise = t + tooth_dt * 0.45
                cam_fall = cam_rise + tooth_dt * 2  # a few teeth wide, like a Hall vane
                states[1] = 1
                rows_out.append((cam_rise, tuple(states)))
                states[1] = 0
                rows_out.append((cam_fall, tuple(states)))
            t += tooth_dt
    write_csv(csv_path, rows_out)


def write_csv(csv_path, rows_out):
    rows_out.sort(key=lambda r: r[0])
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["Time [s]", "crank", "cam"])
        for t, (crank, cam) in rows_out:
            w.writerow([f"{t:.9f}", crank, cam])
    print(f"wrote {len(rows_out)} rows to {csv_path}")


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--synthetic":
        synthetic(sys.argv[2])
    elif len(sys.argv) == 3:
        convert_msl(sys.argv[1], sys.argv[2])
    else:
        print(__doc__)
        sys.exit(1)
