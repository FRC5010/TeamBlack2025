#!/usr/bin/env python3
"""Swerve azimuth diagnostics from an AdvantageKit WPILOG.

Reads a ``.wpilog`` (no external dependencies) and summarizes the
``Swerve/Diag/*`` signals added in GenericSwerveDrivetrain/YAGSLSwerveDrivetrain,
so a wheel that fails to reach its commanded rotation can be diagnosed offline.

Usage:
    python3 tools/swerve_log_analysis.py <path-to-log.wpilog> [--error-deg 5]

For each per-module signal it reports min/max/range; it also lists the moments
where a module is flagged ``misaligned`` (settled but off target) and prints the
concurrent signals (absolute-vs-relative encoder gap, absolute-encoder read
issue, steer applied output / voltage / velocity) so the failure can be mapped
to a cause:
  * abs-vs-rel gap grows           -> relative/absolute encoder desync
  * absoluteEncoderReadIssue true  -> intermittent CAN read failure
  * steer output railed, vel ~0    -> saturation / mechanical stall
  * small steady error, ~0 output  -> weak angle PID

The signals only exist in logs from the instrumented build (the commit that
added ``Swerve/Diag/*``). For older logs, fall back to YAGSL's ``swerve/``
desired/measured states in AdvantageScope.
"""

import bisect
import struct
import sys

DIAG = "/RealOutputs/Swerve/Diag/"


def _read_str(buf, off):
    n = struct.unpack_from("<I", buf, off)[0]
    off += 4
    return buf[off : off + n].decode("utf-8", "replace"), off + n


def _decode(payload, typ):
    try:
        if typ == "double":
            return struct.unpack("<d", payload)[0]
        if typ in ("int64", "int"):
            return int.from_bytes(payload, "little", signed=True)
        if typ == "boolean":
            return payload[0] != 0
        if typ == "double[]":
            return list(struct.unpack("<%dd" % (len(payload) // 8), payload))
        if typ == "boolean[]":
            return [b != 0 for b in payload]
        if typ in ("string", "json"):
            return payload.decode("utf-8", "replace")
    except Exception:
        return None
    return None


def parse(path):
    """Return (entries, data): id->(name,type) and name->[(ts_us, value), ...]."""
    buf = open(path, "rb").read()
    if buf[:6] != b"WPILOG":
        raise ValueError("not a WPILOG file")
    extra_len = struct.unpack_from("<I", buf, 8)[0]
    off = 12 + extra_len
    entries = {}
    data = {}
    n = len(buf)
    while off < n:
        head = buf[off]
        off += 1
        id_len = (head & 0x3) + 1
        size_len = ((head >> 2) & 0x3) + 1
        ts_len = ((head >> 4) & 0x7) + 1
        if off + id_len + size_len + ts_len > n:
            break
        eid = int.from_bytes(buf[off : off + id_len], "little")
        off += id_len
        psize = int.from_bytes(buf[off : off + size_len], "little")
        off += size_len
        ts = int.from_bytes(buf[off : off + ts_len], "little")
        off += ts_len
        payload = buf[off : off + psize]
        off += psize
        if eid == 0:  # control record
            if not payload:
                continue
            if payload[0] == 0:  # Start
                start_id = struct.unpack_from("<I", payload, 1)[0]
                name, p = _read_str(payload, 5)
                typ, _ = _read_str(payload, p)
                entries[start_id] = (name, typ)
                data.setdefault(name, [])
            continue
        ent = entries.get(eid)
        if ent:
            name, typ = ent
            data[name].append((ts, _decode(payload, typ)))
    return entries, data


def value_at(series, ts):
    """Last value at or before ts in a [(ts, value), ...] series, else None."""
    if not series:
        return None
    times = [t for t, _ in series]
    i = bisect.bisect_right(times, ts) - 1
    return series[i][1] if i >= 0 else series[0][1]


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    path = sys.argv[1]
    error_deg = 5.0
    if "--error-deg" in sys.argv:
        error_deg = float(sys.argv[sys.argv.index("--error-deg") + 1])

    _, data = parse(path)
    all_ts = [t for v in data.values() for t, _ in v]
    if not all_ts:
        print("No records decoded.")
        return
    t0, t1 = min(all_ts), max(all_ts)
    print("file: %s" % path)
    print("duration: %.1f s   entries: %d" % ((t1 - t0) / 1e6, len(data)))

    enabled = data.get("/DriverStation/Enabled", [])
    if enabled:
        frac = sum(1 for _, x in enabled if x) / len(enabled)
        print("enabled fraction: %.2f (%d transitions)" % (frac, len(enabled)))
        if frac == 0:
            print("WARNING: robot never enabled in this log - no driving data.")

    diag = {k: v for k, v in data.items() if k.startswith(DIAG)}
    if not diag:
        print(
            "\nNo Swerve/Diag/* signals - log predates the instrumented build.\n"
            "Use YAGSL's swerve/ desired vs measured states in AdvantageScope instead."
        )
        return

    print("\n--- per-signal ranges ---")
    for name in sorted(diag):
        short = name[len(DIAG):]
        vals = [x for _, x in diag[name] if x is not None]
        if not vals:
            print("%-26s (no data)" % short)
            continue
        if isinstance(vals[0], list):
            cols = max(len(a) for a in vals)
            spans = []
            for c in range(cols):
                col = [a[c] for a in vals if len(a) > c]
                spans.append("[% .1f,% .1f]" % (min(col), max(col)))
            print("%-26s %s" % (short, "  ".join(spans)))
        elif isinstance(vals[0], bool):
            print("%-26s true=%d / %d samples" % (short, sum(vals), len(vals)))
        else:
            print("%-26s min=% .3f max=% .3f" % (short, min(vals), max(vals)))

    mis = data.get(DIAG + "misaligned", [])
    err = data.get(DIAG + "errorDeg", [])
    absrel = data.get(DIAG + "absMinusRelDeg", [])
    readissue = data.get(DIAG + "absoluteEncoderReadIssue", [])
    out = data.get(DIAG + "steerAppliedOutput", [])
    vel = data.get(DIAG + "steerVelocityDps", [])

    print("\n--- misalignment events (settled but off target) ---")
    events = 0
    for ts, flags in mis:
        if not flags or not any(flags):
            continue
        events += 1
        rel = (ts - t0) / 1e6
        bad = [i for i, f in enumerate(flags) if f]
        e = value_at(err, ts) or []
        ar = value_at(absrel, ts) or []
        ri = value_at(readissue, ts) or []
        ao = value_at(out, ts) or []
        sv = value_at(vel, ts) or []

        def g(arr, i):
            return arr[i] if i < len(arr) else float("nan")

        print("t=%7.2fs  modules %s" % (rel, bad))
        for i in bad:
            print(
                "    mod%d: err=% 7.1f  absMinusRel=% 7.1f  readIssue=%s  "
                "out=% .2f  vel=% .1f"
                % (i, g(e, i), g(ar, i), bool(g(ri, i)), g(ao, i), g(sv, i))
            )
        if events >= 40:
            print("    ... (more events truncated)")
            break
    if events == 0:
        print("none flagged (no module exceeded %.0f deg while settled)" % error_deg)


if __name__ == "__main__":
    main()
