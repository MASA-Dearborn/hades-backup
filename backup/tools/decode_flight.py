#!/usr/bin/env python3
"""Decode a binary flight log (FLTnn.BIN) into CSV.

The firmware writes fixed-size LogRecord structs (see backup/include/datalogger.h),
packed little-endian, 97 bytes each. This converts them to a CSV you can open in
Excel / Sheets / pandas.

Usage:
    python decode_flight.py FLT00.BIN              # -> FLT00.csv
    python decode_flight.py FLT00.BIN out.csv      # custom output name
"""

import struct
import sys
import csv

# Must match LogRecord in backup/include/datalogger.h, in order.
#   uint32 timeUs, uint8 phase, then 23 floats.
RECORD_FMT = "<IB23f"
RECORD_SIZE = struct.calcsize(RECORD_FMT)
assert RECORD_SIZE == 97, f"record size {RECORD_SIZE} != 97 — struct mismatch"

COLUMNS = [
    "time_us", "phase",
    "ax", "ay", "az",
    "gx", "gy", "gz",
    "mx", "my", "mz",
    "hpa", "temp_c",
    "h", "v", "a",
    "q0", "q1", "q2", "q3", "tilt_deg",
    "estimated_apogee_m", "apogee_error_m", "target_position_cm",
    "actual_position_cm",
]

PHASE_NAMES = {0: "IDLE", 1: "LAUNCHED", 2: "COASTING", 3: "APOGEE", 4: "DESCENT"}


def decode(in_path, out_path):
    with open(in_path, "rb") as f:
        blob = f.read()

    n_chunks = len(blob) // RECORD_SIZE
    rows = 0
    with open(out_path, "w", newline="") as out:
        w = csv.writer(out)
        w.writerow(COLUMNS + ["phase_name"])
        for i in range(n_chunks):
            chunk = blob[i * RECORD_SIZE:(i + 1) * RECORD_SIZE]
            # The SD file may be pre-allocated; trailing records are all zeros
            # if you pulled the card without pressing 's' (stop & truncate).
            if chunk.count(0) == RECORD_SIZE:
                break
            rec = struct.unpack(RECORD_FMT, chunk)
            phase = rec[1]
            w.writerow(list(rec) + [PHASE_NAMES.get(phase, phase)])
            rows += 1

    tail = len(blob) - n_chunks * RECORD_SIZE
    print(f"{in_path}: {rows} records -> {out_path}")
    if tail:
        print(f"  note: {tail} trailing bytes were not a full record (ignored)")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) > 2 else src.rsplit(".", 1)[0] + ".csv"
    decode(src, dst)
