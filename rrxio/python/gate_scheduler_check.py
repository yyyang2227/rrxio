#!/usr/bin/env python3

"""
Gate checker for scheduler migration (legacy/event_stage1/event_stage2).
Checks:
1) scheduler diag schema completeness and required-cell non-empty
2) same-timestamp priority order (IMU > VISUAL > RADAR > POSE/VEL > RESET)
3) drop/reject traceability consistency
4) event_stage2 radar latency P95 improvement over legacy
5) optional accuracy/runtime guardrails (using metrics csv)
"""

import argparse
import csv
import json
import math
import os
import statistics
import sys
from collections import defaultdict


EVENT_PRIORITY = {
    "imu": 0,
    "image0": 1,
    "image1": 1,
    "radar_trigger": 2,
    "radar_scan": 2,
    "groundtruth_pose": 3,
    "groundtruth_odom": 3,
    "velocity": 3,
    "reset": 4,
    "reset_to_pose": 4,
}


def append_fail(failures, message):
    failures.append(message)


def finite(v):
    try:
        return math.isfinite(float(v))
    except Exception:
        return False


def f64(v):
    return float(v)


def i64(v):
    return int(float(v))


def percentile(values, q):
    if not values:
        return float("nan")
    vals = sorted(values)
    if len(vals) == 1:
        return float(vals[0])
    pos = (len(vals) - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return float(vals[lo])
    w = pos - lo
    return float(vals[lo] * (1.0 - w) + vals[hi] * w)


def median(values):
    vals = [float(v) for v in values if finite(v)]
    if not vals:
        return float("nan")
    return float(statistics.median(vals))


def load_required_schema_columns(schema_csv):
    required = []
    with open(schema_csv, "r", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row.get("required", "").strip().lower() == "true":
                field = row.get("field", "").strip()
                if field:
                    required.append(field)
    return required


def check_sched_diag_file(path, required_cols):
    info = {
        "path": path,
        "rows": 0,
        "missing_required_columns": [],
        "empty_required_cells": 0,
        "row_id_sequence_ok": True,
        "drop_total_monotonic_ok": True,
        "drop_type_consistent_ok": True,
        "priority_order_ok": True,
        "radar_latency_samples": 0,
        "radar_latency_p95_ms": float("nan"),
    }
    failures = []
    radar_latencies = []

    if not os.path.isfile(path):
        append_fail(failures, "sched diag file missing: %s" % path)
        return info, failures, radar_latencies

    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames or []
        for col in required_cols:
            if col not in fieldnames:
                info["missing_required_columns"].append(col)
        if info["missing_required_columns"]:
            append_fail(failures, "missing required columns in %s: %s" % (path, info["missing_required_columns"]))
            return info, failures, radar_latencies

        prev_row_id = -1
        prev_drop_total = -1
        ts_priority = defaultdict(list)

        for row in reader:
            info["rows"] += 1
            for col in required_cols:
                if row.get(col, "").strip() == "":
                    info["empty_required_cells"] += 1

            try:
                row_id = i64(row.get("row_id", "nan"))
                if row_id != prev_row_id + 1:
                    info["row_id_sequence_ok"] = False
                prev_row_id = row_id
            except Exception:
                info["row_id_sequence_ok"] = False

            try:
                drop_total = i64(row.get("drop_total", "nan"))
                if drop_total < prev_drop_total:
                    info["drop_total_monotonic_ok"] = False
                if prev_drop_total >= 0 and drop_total > prev_drop_total:
                    drop_type = row.get("drop_type", "").strip().lower()
                    if drop_type in ("", "none"):
                        info["drop_type_consistent_ok"] = False
                prev_drop_total = drop_total
            except Exception:
                info["drop_total_monotonic_ok"] = False
                info["drop_type_consistent_ok"] = False

            event_type = row.get("event_type", "").strip().lower()
            if event_type not in EVENT_PRIORITY:
                info["priority_order_ok"] = False
            try:
                ts_key = round(f64(row.get("event_timestamp", "nan")), 9)
                ts_priority[ts_key].append(EVENT_PRIORITY.get(event_type, 99))
            except Exception:
                info["priority_order_ok"] = False

            if event_type == "radar_scan" and finite(row.get("radar_enqueue_to_commit_ms", "nan")):
                v = f64(row["radar_enqueue_to_commit_ms"])
                if v >= 0.0:
                    radar_latencies.append(v)

            starved = row.get("radar_starved", "").strip()
            if starved not in ("0", "1"):
                info["drop_type_consistent_ok"] = False

        for _, priorities in ts_priority.items():
            if any(priorities[i] < priorities[i - 1] for i in range(1, len(priorities))):
                info["priority_order_ok"] = False
                break

    if info["rows"] == 0:
        append_fail(failures, "empty sched diag file: %s" % path)
    if info["empty_required_cells"] > 0:
        append_fail(failures, "required cells empty in %s: %d" % (path, info["empty_required_cells"]))
    if not info["row_id_sequence_ok"]:
        append_fail(failures, "row_id sequence mismatch in %s" % path)
    if not info["drop_total_monotonic_ok"]:
        append_fail(failures, "drop_total is not monotonic in %s" % path)
    if not info["drop_type_consistent_ok"]:
        append_fail(failures, "drop/reject semantic mismatch in %s" % path)
    if not info["priority_order_ok"]:
        append_fail(failures, "same-timestamp event priority mismatch in %s" % path)

    info["radar_latency_samples"] = len(radar_latencies)
    info["radar_latency_p95_ms"] = percentile(radar_latencies, 0.95) if radar_latencies else float("nan")
    return info, failures, radar_latencies


def main():
    parser = argparse.ArgumentParser(description="Scheduler migration gate checker")
    parser.add_argument("--results_root", required=True)
    parser.add_argument("--manifest", default="")
    parser.add_argument("--sched_diag_dir", default="")
    parser.add_argument("--schema", default="")
    parser.add_argument("--metrics", default="")
    parser.add_argument("--report_json", default="")
    parser.add_argument("--min_latency_improve", type=float, default=0.30)
    parser.add_argument("--max_ate_degrade", type=float, default=0.05)
    parser.add_argument("--max_rpe_degrade", type=float, default=0.05)
    parser.add_argument("--max_runtime_increase", type=float, default=0.15)
    parser.add_argument("--require_metrics", type=int, default=1)
    args = parser.parse_args()

    manifest = args.manifest if args.manifest else os.path.join(args.results_root, "run_manifest.csv")
    sched_diag_dir = (
        args.sched_diag_dir if args.sched_diag_dir else os.path.join(args.results_root, "dvc_scheduler_diag")
    )
    schema = args.schema if args.schema else os.path.join(
        os.path.dirname(os.path.dirname(__file__)), "publish_plan", "templates", "dvc_sched_diag_schema.csv"
    )
    metrics = args.metrics if args.metrics else os.path.join(args.results_root, "w6_metrics.csv")
    report_json = (
        args.report_json if args.report_json else os.path.join(args.results_root, "gate_scheduler_report.json")
    )

    report = {
        "gate": "scheduler",
        "results_root": args.results_root,
        "manifest": manifest,
        "sched_diag_dir": sched_diag_dir,
        "schema": schema,
        "metrics": metrics,
        "criteria": {
            "min_latency_improve": args.min_latency_improve,
            "max_ate_degrade": args.max_ate_degrade,
            "max_rpe_degrade": args.max_rpe_degrade,
            "max_runtime_increase": args.max_runtime_increase,
        },
        "checks": {},
        "failures": [],
        "pass": False,
    }
    failures = []

    if not os.path.isfile(schema):
        append_fail(failures, "schema not found: %s" % schema)
        required_cols = []
    else:
        required_cols = load_required_schema_columns(schema)
        if not required_cols:
            append_fail(failures, "no required columns in schema: %s" % schema)
    report["checks"]["required_columns"] = required_cols

    if not os.path.isfile(manifest):
        append_fail(failures, "manifest not found: %s" % manifest)
        manifest_rows = []
    else:
        with open(manifest, "r", encoding="utf-8") as f:
            manifest_rows = list(csv.DictReader(f))

    success_rows = [r for r in manifest_rows if r.get("status", "") == "SUCCESS"]
    if not success_rows:
        append_fail(failures, "no SUCCESS rows in manifest")

    by_mode = defaultdict(list)
    run_mode = {}
    for row in success_rows:
        mode = row.get("scheduler_mode", "").strip().lower()
        by_mode[mode].append(row)
        run_mode[row.get("run_id", "")] = mode

    report["checks"]["success_count_by_mode"] = {k: len(v) for k, v in by_mode.items()}
    for mode in ("legacy", "event_stage1", "event_stage2"):
        if len(by_mode.get(mode, [])) == 0:
            append_fail(failures, "no SUCCESS run for scheduler_mode=%s" % mode)

    per_file = []
    latencies_by_mode = defaultdict(list)

    for mode, rows in by_mode.items():
        for row in rows:
            sched_diag_file = row.get("sched_diag_file", "").strip()
            if not sched_diag_file:
                sched_diag_file = os.path.join(sched_diag_dir, "dvc_sched_diag_%s.csv" % row.get("run_id", ""))
            info, local_failures, radar_latencies = check_sched_diag_file(sched_diag_file, required_cols)
            per_file.append(info)
            latencies_by_mode[mode].extend(radar_latencies)
            for item in local_failures:
                append_fail(failures, item)

    report["checks"]["sched_diag_files_checked"] = len(per_file)
    report["checks"]["per_file"] = per_file
    report["checks"]["radar_latency_p95_ms"] = {
        mode: percentile(values, 0.95) if values else float("nan") for mode, values in latencies_by_mode.items()
    }

    p95_legacy = report["checks"]["radar_latency_p95_ms"].get("legacy", float("nan"))
    p95_stage2 = report["checks"]["radar_latency_p95_ms"].get("event_stage2", float("nan"))
    if not finite(p95_legacy) or not finite(p95_stage2):
        append_fail(failures, "invalid radar latency P95 for legacy/event_stage2")
    else:
        improve = (p95_legacy - p95_stage2) / max(p95_legacy, 1.0e-12)
        report["checks"]["latency_improve_ratio"] = improve
        if improve < args.min_latency_improve:
            append_fail(
                failures,
                "radar latency improvement too small: %.6f < %.6f" % (improve, args.min_latency_improve),
            )

    metrics_rows = []
    if os.path.isfile(metrics):
        with open(metrics, "r", encoding="utf-8") as f:
            metrics_rows = list(csv.DictReader(f))
    elif args.require_metrics:
        append_fail(failures, "metrics file missing: %s" % metrics)

    if metrics_rows:
        for row in metrics_rows:
            if not row.get("scheduler_mode", "").strip():
                row["scheduler_mode"] = run_mode.get(row.get("run_id", ""), "")
        met_by_mode = defaultdict(list)
        for row in metrics_rows:
            mode = row.get("scheduler_mode", "").strip().lower()
            if mode:
                met_by_mode[mode].append(row)

        m_legacy = met_by_mode.get("legacy", [])
        m_stage2 = met_by_mode.get("event_stage2", [])
        if not m_legacy or not m_stage2:
            append_fail(failures, "metrics missing legacy/event_stage2 partition")
        else:
            ate_legacy = median([r.get("ate_rmse", "nan") for r in m_legacy])
            ate_stage2 = median([r.get("ate_rmse", "nan") for r in m_stage2])
            rpe_legacy = median([r.get("rpe_rmse", "nan") for r in m_legacy])
            rpe_stage2 = median([r.get("rpe_rmse", "nan") for r in m_stage2])
            rt_legacy = median([r.get("runtime_s", "nan") for r in m_legacy])
            rt_stage2 = median([r.get("runtime_s", "nan") for r in m_stage2])

            report["checks"]["metrics_median"] = {
                "ate_legacy": ate_legacy,
                "ate_stage2": ate_stage2,
                "rpe_legacy": rpe_legacy,
                "rpe_stage2": rpe_stage2,
                "runtime_legacy": rt_legacy,
                "runtime_stage2": rt_stage2,
            }

            if finite(ate_legacy) and finite(ate_stage2):
                ate_degrade = (ate_stage2 - ate_legacy) / max(abs(ate_legacy), 1.0e-12)
                report["checks"]["metrics_median"]["ate_degrade"] = ate_degrade
                if ate_degrade > args.max_ate_degrade:
                    append_fail(failures, "ATE degrade too high: %.6f > %.6f" % (ate_degrade, args.max_ate_degrade))
            else:
                append_fail(failures, "invalid median ATE in metrics")

            if finite(rpe_legacy) and finite(rpe_stage2):
                rpe_degrade = (rpe_stage2 - rpe_legacy) / max(abs(rpe_legacy), 1.0e-12)
                report["checks"]["metrics_median"]["rpe_degrade"] = rpe_degrade
                if rpe_degrade > args.max_rpe_degrade:
                    append_fail(failures, "RPE degrade too high: %.6f > %.6f" % (rpe_degrade, args.max_rpe_degrade))
            else:
                append_fail(failures, "invalid median RPE in metrics")

            if finite(rt_legacy) and finite(rt_stage2):
                rt_increase = (rt_stage2 - rt_legacy) / max(abs(rt_legacy), 1.0e-12)
                report["checks"]["metrics_median"]["runtime_increase"] = rt_increase
                if rt_increase > args.max_runtime_increase:
                    append_fail(
                        failures,
                        "runtime increase too high: %.6f > %.6f" % (rt_increase, args.max_runtime_increase),
                    )
            else:
                append_fail(failures, "invalid median runtime in metrics")

    report["pass"] = len(failures) == 0
    report["failures"] = failures
    os.makedirs(os.path.dirname(report_json), exist_ok=True)
    with open(report_json, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)

    print("[gate_scheduler] report: %s" % report_json)
    if failures:
        print("[gate_scheduler] FAIL")
        for item in failures:
            print(" - %s" % item)
        sys.exit(1)
    print("[gate_scheduler] PASS")


if __name__ == "__main__":
    main()
