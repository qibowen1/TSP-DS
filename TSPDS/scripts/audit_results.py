#!/usr/bin/env python3
"""Audit TSP-DS result CSVs against instance data and BKV values.

The script recomputes route feasibility and objective values independently from
CSV instance files. It treats rows with Valid=0 as invalid and ignores them for
best-valid summaries.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import math
import re
from collections import Counter, defaultdict
from pathlib import Path


OPTIMAL_KEYS = {
    ("pr152", "C", 1), ("pr152", "C", 3), ("pr152", "C", 5),
    ("pr152", "L", 1), ("pr152", "L", 3), ("pr152", "L", 5),
    ("rat195", "C", 1),
    ("gr229", "C", 1), ("gr229", "C", 3), ("gr229", "C", 5),
    ("gr229", "L", 1), ("gr229", "L", 3),
    ("pr264", "L", 1),
}




def round_half_up(value: float, decimals: int) -> float:
    scale = 10 ** decimals
    if value >= 0:
        return math.floor(value * scale + 0.5 + 1e-12) / scale
    return math.ceil(value * scale - 0.5 - 1e-12) / scale

def parse_opt_file(path: Path) -> dict[tuple[str, str, int], float]:
    opt: dict[tuple[str, str, int], float] = {}
    if not path.exists():
        return opt
    with path.open(newline="") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = re.split(r"[\s,]+", line)
            if len(parts) < 3:
                continue
            parsed = False
            try:
                # Repository format: gr120_C 1 1419
                key = parts[0].split("_")
                if len(key) >= 2 and len(parts) >= 3:
                    inst = "_".join(key[:-1])
                    depot = key[-1].upper()
                    drones = int(parts[1])
                    value = float(parts[2])
                    parsed = True
            except Exception:
                parsed = False
            if not parsed:
                try:
                    # Expanded format: gr120 C 1 1419
                    inst = parts[0]
                    depot = parts[1].upper()
                    drones = int(parts[2])
                    value = float(parts[3])
                    parsed = True
                except Exception:
                    parsed = False
            if not parsed:
                try:
                    # Compact format: gr120_C_1 1419
                    key = parts[0].split("_")
                    inst = "_".join(key[:-2])
                    depot = key[-2].upper()
                    drones = int(key[-1])
                    value = float(parts[1])
                    parsed = True
                except Exception:
                    continue
            opt[(inst, depot, drones)] = value
    return opt


def csv_instance_key(instance_file: str) -> tuple[str, str]:
    stem = Path(instance_file).stem
    if stem.endswith("_0_80"):
        return stem[:-5], "C"
    if stem.endswith("_l_80"):
        return stem[:-5], "L"
    return stem, ""


def parse_instance(path: Path):
    recs = []
    with path.open(newline="") as f:
        for row in csv.reader(f):
            if len(row) < 4:
                continue
            cells = row[-4:]
            try:
                recs.append((int(float(cells[0])), float(cells[1]), float(cells[2]), int(float(cells[3]))))
            except Exception:
                continue
    if not recs:
        raise ValueError(f"no valid rows in {path}")
    max_id = max(r[0] for r in recs)
    coords = [None] * (max_id + 1)
    flags = [0] * (max_id + 1)
    seen = [False] * (max_id + 1)
    for node_id, x, y, flag in recs:
        coords[node_id] = (x, y)
        flags[node_id] = flag
        seen[node_id] = True
    depot = 0
    dup = -1
    for i in range(max_id + 1):
        if seen[i] and i != depot and coords[i] == coords[depot]:
            dup = max(dup, i)
    keep = [i for i in range(max_id + 1) if seen[i] and i != dup]
    if depot in keep:
        keep.remove(depot)
    keep = [depot] + keep
    new_coords = [coords[i] for i in keep]
    new_flags = [flags[i] for i in keep]
    try:
        station = new_flags.index(2)
    except ValueError as exc:
        raise ValueError(f"no station flag=2 in {path}") from exc
    return keep, new_coords, new_flags, station


def manhattan(coords, a: int, b: int) -> float:
    return abs(coords[a][0] - coords[b][0]) + abs(coords[a][1] - coords[b][1])


def euclidean(coords, a: int, b: int) -> float:
    return math.hypot(coords[a][0] - coords[b][0], coords[a][1] - coords[b][1])


def parse_route(route_s: str) -> list[int]:
    return [int(x) for x in route_s.split("->") if x.strip()]


def parse_drone_tasks(tasks_s: str) -> dict[int, list[int]]:
    out: dict[int, list[int]] = {}
    for d, body in re.findall(r"D(\d+):\{([^}]*)\}", tasks_s or ""):
        out[int(d)] = [int(x) for x in body.split() if x.strip()]
    return out


def audit_row(row: dict[str, str], data_dir: Path, speed_ratio: float, tolerance: float):
    instance = row["Instance"]
    keep, coords, flags, station = parse_instance(data_dir / instance)
    n = len(coords)
    route = parse_route(row.get("TruckRoute", ""))
    assignments = parse_drone_tasks(row.get("DroneTasks", ""))
    errors = []
    if len(route) < 2 or route[0] != 0 or route[-1] != 0:
        errors.append("truck route must start/end at depot")
    if station not in route:
        errors.append("station missing from route")
        station_pos = 0
    else:
        station_pos = route.index(station)
        if route.count(station) != 1:
            errors.append("station duplicated in route")

    served = Counter()
    truck_time = 0.0
    for i, node in enumerate(route):
        if node < 0 or node >= n:
            errors.append(f"invalid truck node {node}")
            continue
        if 0 < i < len(route) - 1:
            served[node] += 1
        if i + 1 < len(route) and 0 <= route[i + 1] < n:
            truck_time += manhattan(coords, node, route[i + 1])

    active = 0.0
    for i in range(station_pos):
        active += manhattan(coords, route[i], route[i + 1])

    max_load = 0.0
    drone_count = int(row["DroneCount"])
    if len(assignments) > drone_count:
        errors.append("too many drone assignment buckets")
    for d, nodes in assignments.items():
        if d < 0 or d >= drone_count:
            errors.append(f"invalid drone id {d}")
        load = 0.0
        for node in nodes:
            if node < 0 or node >= n:
                errors.append(f"invalid drone node {node}")
                continue
            if node == 0 or node == station:
                errors.append(f"depot/station served by drone {node}")
            if flags[node] != 0:
                errors.append(f"node {node} flag={flags[node]} served by drone")
            served[node] += 1
            load += 2.0 * euclidean(coords, station, node) * speed_ratio
        max_load = max(max_load, load)

    for node in range(1, n):
        if served[node] == 0:
            errors.append(f"missing node {node}")
        elif served[node] > 1:
            errors.append(f"duplicated node {node}")

    drone_complete = active + max_load
    makespan = max(truck_time, drone_complete)
    reported = float(row["Makespan"])
    if abs(makespan - reported) > tolerance:
        errors.append(f"makespan mismatch recomputed={makespan:.10g} reported={reported:.10g}")
    return {
        "valid": not errors and row.get("Valid", "1") not in {"0", "false", "FALSE"},
        "errors": errors,
        "makespan": makespan,
        "truck": truck_time,
        "drone": drone_complete,
        "active": active,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("result_csv", type=Path)
    ap.add_argument("--data-dir", type=Path, default=Path("data/tsp_origan"))
    ap.add_argument("--opt-file", type=Path, default=Path("data/TSP-DS-OPT.txt"))
    ap.add_argument("--speed-ratio", type=float, default=0.5)
    ap.add_argument("--tolerance", type=float, default=5e-1, help="Allowed difference from rounded CSV makespan output")
    ap.add_argument("--bkv-decimals", type=int, default=1, help="Decimals used when comparing final makespan to published BKV")
    ap.add_argument("--hit-tolerance", type=float, default=0.5, help="Treat published gaps within this tolerance as hits")
    args = ap.parse_args()

    opt = parse_opt_file(args.opt_file)
    rows = []
    with args.result_csv.open(newline="") as f:
        for row in csv.DictReader(f):
            if row.get("Instance") == "SUMMARY" or row.get("DepotPosition") == "BEST":
                continue
            if not row.get("Instance", "").endswith(".csv"):
                continue
            rows.append(row)

    hashes = defaultdict(list)
    for p in args.data_dir.glob("*.csv"):
        hashes[hashlib.md5(p.read_bytes()).hexdigest()].append(p.name)
    blocked = {name for names in hashes.values() if len(names) > 1 for name in names}

    best_valid = {}
    invalid = []
    below = []
    for row in rows:
        audit = audit_row(row, args.data_dir, args.speed_ratio, args.tolerance)
        inst_base, depot = csv_instance_key(row["Instance"])
        key = (inst_base, depot, int(row["DroneCount"]))
        bkv = opt.get(key)
        row_id = (row["Instance"], row["DepotPosition"], row["DroneCount"], row.get("RunId", ""))
        if row["Instance"] in blocked and depot == "L":
            audit["valid"] = False
            audit["errors"].append("data-blocked: L csv duplicates another input file")
        if not audit["valid"]:
            invalid.append((row_id, audit["errors"]))
            continue
        published_ms = round_half_up(audit["makespan"], args.bkv_decimals)
        if bkv is not None and key in OPTIMAL_KEYS and published_ms < bkv - args.hit_tolerance - 1e-9:
            below.append((row_id, audit["makespan"], published_ms, bkv))
        if key not in best_valid or audit["makespan"] < best_valid[key][0]:
            best_valid[key] = (audit["makespan"], bkv, row)

    print(f"rows={len(rows)} valid_best_cases={len(best_valid)} invalid_rows={len(invalid)} below_bkv_valid_rows={len(below)}")
    if below:
        print("\nBELOW_BKV_VALID")
        for row_id, ms, published_ms, bkv in below:
            print(f"{row_id}: exact={ms:.10g} rounded={published_ms:.10g} bkv={bkv:.10g}")
    if invalid:
        print("\nINVALID_ROWS")
        for row_id, errors in invalid[:50]:
            print(f"{row_id}: {'; '.join(errors[:6])}")
        if len(invalid) > 50:
            print(f"... {len(invalid) - 50} more invalid rows")
    print("\nBEST_VALID_BY_CASE")
    for key in sorted(best_valid):
        ms, bkv, row = best_valid[key]
        published_ms = round_half_up(ms, args.bkv_decimals)
        gap = "NA" if bkv is None else f"{published_ms - bkv:.10g}"
        if bkv is None:
            status = "NA"
        elif key in OPTIMAL_KEYS:
            status = "HIT" if published_ms <= bkv + args.hit_tolerance + 1e-9 else "MISS"
        else:
            status = "HIT" if published_ms <= bkv + args.hit_tolerance + 1e-9 else "MISS"
        opt_mark = "*" if key in OPTIMAL_KEYS else ""
        print(f"{status:4s} {key[0]}_{key[1]} m={key[2]} best={published_ms:.10g} bkv={bkv}{opt_mark} gap={gap} exact={ms:.10g} run={row.get('RunId','')}")
    return 1 if below else 0


if __name__ == "__main__":
    raise SystemExit(main())
