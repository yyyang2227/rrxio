#!/usr/bin/env python3

"""
Gate checker for scheduler migration (legacy/event_stage1/event_stage2).
Checks:
1) scheduler diag schema completeness and required-cell non-empty
2) same-timestamp priority order (IMU > VISUAL > RADAR > POSE/VEL > RESET)
3) drop/reject traceability consistency
4) strict radar latency guardrails (absolute median/p95/max thresholds for event_stage2)
5) estimation/runtime guardrails (ATE/RPE/runtime + NIS + committed updates)
6) repeatability guardrails (3-trial consistency)
7) timing ledger summary for bottleneck analysis
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

TIMING_COLUMNS = [
    "loader_backpressure_wait_ms",
    "event_queue_wait_ms",
    "worker_dispatch_ms",
    "update_publish_ms",
    "diag_write_ms",
]


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


def text(v):
    if v is None:
        return ""
    return str(v).strip()


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


def safe_rate(num, den):
    if den <= 0:
        return float("nan")
    return float(num) / float(den)


def cv(values):
    vals = [float(v) for v in values if finite(v)]
    if len(vals) < 2:
        return float("nan")
    m = statistics.mean(vals)
    if abs(m) < 1.0e-12:
        return 0.0 if max(abs(v) for v in vals) < 1.0e-12 else float("inf")
    return statistics.pstdev(vals) / abs(m)


def rel_range(values):
    vals = [float(v) for v in values if finite(v)]
    if len(vals) < 2:
        return float("nan")
    m = statistics.mean([abs(v) for v in vals])
    return (max(vals) - min(vals)) / max(m, 1.0e-12)


def load_required_schema_columns(schema_csv):
    required = []
    with open(schema_csv, "r", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if text(row.get("required", "")).lower() == "true":
                field = text(row.get("field", ""))
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
        "radar_latency_median_ms": float("nan"),
        "radar_latency_p95_ms": float("nan"),
        "radar_latency_max_ms": float("nan"),
        "last_drop_total": 0,
        "timing_sum": {k: 0.0 for k in TIMING_COLUMNS},
        "timing_count": {k: 0 for k in TIMING_COLUMNS},
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
                if text(row.get(col, "")) == "":
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
                    drop_type = text(row.get("drop_type", "")).lower()
                    if drop_type in ("", "none"):
                        info["drop_type_consistent_ok"] = False
                prev_drop_total = drop_total
                info["last_drop_total"] = drop_total
            except Exception:
                info["drop_total_monotonic_ok"] = False
                info["drop_type_consistent_ok"] = False

            event_type = text(row.get("event_type", "")).lower()
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

            starved = text(row.get("radar_starved", ""))
            if starved not in ("0", "1"):
                info["drop_type_consistent_ok"] = False

            for col in TIMING_COLUMNS:
                if finite(row.get(col, "nan")):
                    info["timing_sum"][col] += f64(row[col])
                    info["timing_count"][col] += 1

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
    info["radar_latency_median_ms"] = median(radar_latencies)
    info["radar_latency_p95_ms"] = percentile(radar_latencies, 0.95) if radar_latencies else float("nan")
    info["radar_latency_max_ms"] = max(radar_latencies) if radar_latencies else float("nan")
    return info, failures, radar_latencies


def main():
    parser = argparse.ArgumentParser(description="Scheduler migration gate checker")
    parser.add_argument("--results_root", required=True)
    parser.add_argument("--manifest", default="")
    parser.add_argument("--sched_diag_dir", default="")
    parser.add_argument("--schema", default="")
    parser.add_argument("--metrics", default="")
    parser.add_argument("--report_json", default="")
    parser.add_argument("--required_scheduler_modes", default="legacy,event_stage2")
    parser.add_argument("--max_latency_median_ms", type=float, default=30.0)
    parser.add_argument("--max_latency_p95_ms", type=float, default=50.0)
    parser.add_argument("--max_latency_max_ms", type=float, default=80.0)
    parser.add_argument("--max_ate_degrade", type=float, default=0.05)
    parser.add_argument("--max_rpe_degrade", type=float, default=0.05)
    parser.add_argument("--max_runtime_increase", type=float, default=0.15)
    parser.add_argument("--max_nis_abs_increase_pp", type=float, default=1.0)
    parser.add_argument("--max_committed_drop", type=float, default=0.05)
    parser.add_argument("--repeat_min_runs", type=int, default=3)
    parser.add_argument("--max_repeat_ate_cv", type=float, default=0.10)
    parser.add_argument("--max_repeat_rpe_cv", type=float, default=0.10)
    parser.add_argument("--max_repeat_update_rel_range", type=float, default=0.05)
    parser.add_argument("--max_repeat_drop_rel_range", type=float, default=0.10)
    parser.add_argument("--repeatability_modes", default="event_stage2")
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
            "required_scheduler_modes": args.required_scheduler_modes,
            "max_latency_median_ms": args.max_latency_median_ms,
            "max_latency_p95_ms": args.max_latency_p95_ms,
            "max_latency_max_ms": args.max_latency_max_ms,
            "max_ate_degrade": args.max_ate_degrade,
            "max_rpe_degrade": args.max_rpe_degrade,
            "max_runtime_increase": args.max_runtime_increase,
            "max_nis_abs_increase_pp": args.max_nis_abs_increase_pp,
            "max_committed_drop": args.max_committed_drop,
            "repeat_min_runs": args.repeat_min_runs,
            "max_repeat_ate_cv": args.max_repeat_ate_cv,
            "max_repeat_rpe_cv": args.max_repeat_rpe_cv,
            "max_repeat_update_rel_range": args.max_repeat_update_rel_range,
            "max_repeat_drop_rel_range": args.max_repeat_drop_rel_range,
            "repeatability_modes": args.repeatability_modes,
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
        mode = text(row.get("scheduler_mode", "")).lower()
        by_mode[mode].append(row)
        run_mode[row.get("run_id", "")] = mode

    report["checks"]["success_count_by_mode"] = {k: len(v) for k, v in by_mode.items()}
    required_modes = [m.strip().lower() for m in args.required_scheduler_modes.split(",") if m.strip()]
    if not required_modes:
        required_modes = ["legacy", "event_stage2"]
    report["checks"]["required_modes"] = required_modes
    for mode in required_modes:
        if len(by_mode.get(mode, [])) == 0:
            append_fail(failures, "no SUCCESS run for scheduler_mode=%s" % mode)

    per_file = []
    latencies_by_mode = defaultdict(list)
    drop_by_run = {}
    timing_sum_by_mode = defaultdict(lambda: {k: 0.0 for k in TIMING_COLUMNS})

    for mode, rows in by_mode.items():
        for row in rows:
            sched_diag_file = text(row.get("sched_diag_file", ""))
            if not sched_diag_file:
                sched_diag_file = os.path.join(sched_diag_dir, "dvc_sched_diag_%s.csv" % row.get("run_id", ""))
            info, local_failures, radar_latencies = check_sched_diag_file(sched_diag_file, required_cols)
            per_file.append(info)
            latencies_by_mode[mode].extend(radar_latencies)
            drop_by_run[row.get("run_id", "")] = info.get("last_drop_total", 0)
            for col in TIMING_COLUMNS:
                timing_sum_by_mode[mode][col] += info["timing_sum"].get(col, 0.0)
            for item in local_failures:
                append_fail(failures, item)

    report["checks"]["sched_diag_files_checked"] = len(per_file)
    report["checks"]["per_file"] = per_file

    latency_stats = {}
    for mode, values in latencies_by_mode.items():
        latency_stats[mode] = {
            "median": median(values),
            "p95": percentile(values, 0.95) if values else float("nan"),
            "max": max(values) if values else float("nan"),
            "samples": len(values),
        }
    report["checks"]["radar_latency_ms"] = latency_stats

    legacy_lat = latency_stats.get("legacy", {})
    stage2_lat = latency_stats.get("event_stage2", {})
    if not (finite(legacy_lat.get("median", "nan")) and finite(stage2_lat.get("median", "nan"))):
        append_fail(failures, "invalid radar latency median for legacy/event_stage2")
    if not (finite(legacy_lat.get("p95", "nan")) and finite(stage2_lat.get("p95", "nan"))):
        append_fail(failures, "invalid radar latency p95 for legacy/event_stage2")
    if not (finite(legacy_lat.get("max", "nan")) and finite(stage2_lat.get("max", "nan"))):
        append_fail(failures, "invalid radar latency max for legacy/event_stage2")

    if not failures:
        if f64(stage2_lat["median"]) > args.max_latency_median_ms:
            append_fail(
                failures,
                "event_stage2 radar latency median too high: %.6f ms > %.6f ms" % (stage2_lat["median"], args.max_latency_median_ms),
            )
        if f64(stage2_lat["p95"]) > args.max_latency_p95_ms:
            append_fail(
                failures,
                "event_stage2 radar latency p95 too high: %.6f ms > %.6f ms" % (stage2_lat["p95"], args.max_latency_p95_ms),
            )
        if f64(stage2_lat["max"]) > args.max_latency_max_ms:
            append_fail(
                failures,
                "event_stage2 radar latency max too high: %.6f ms > %.6f ms" % (stage2_lat["max"], args.max_latency_max_ms),
            )

    if finite(legacy_lat.get("median", "nan")) and finite(stage2_lat.get("median", "nan")):
        ratio_median = f64(stage2_lat["median"]) / max(f64(legacy_lat["median"]), 1.0e-12)
        ratio_p95 = f64(stage2_lat["p95"]) / max(f64(legacy_lat["p95"]), 1.0e-12)
        ratio_max = f64(stage2_lat["max"]) / max(f64(legacy_lat["max"]), 1.0e-12)
        report["checks"]["radar_latency_ratio"] = {"median": ratio_median, "p95": ratio_p95, "max": ratio_max}

    metrics_rows = []
    if os.path.isfile(metrics):
        with open(metrics, "r", encoding="utf-8") as f:
            metrics_rows = list(csv.DictReader(f))
    elif args.require_metrics:
        append_fail(failures, "metrics file missing: %s" % metrics)

    if metrics_rows:
        for row in metrics_rows:
            if not text(row.get("scheduler_mode", "")):
                row["scheduler_mode"] = run_mode.get(row.get("run_id", ""), "")

        met_by_mode = defaultdict(list)
        for row in metrics_rows:
            mode = text(row.get("scheduler_mode", "")).lower()
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

            valid_legacy = sum(int(r.get("nis_valid_count", "0") or 0) for r in m_legacy)
            exceed_legacy = sum(int(r.get("nis_exceed_count", "0") or 0) for r in m_legacy)
            valid_stage2 = sum(int(r.get("nis_valid_count", "0") or 0) for r in m_stage2)
            exceed_stage2 = sum(int(r.get("nis_exceed_count", "0") or 0) for r in m_stage2)
            nis_legacy = safe_rate(exceed_legacy, valid_legacy)
            nis_stage2 = safe_rate(exceed_stage2, valid_stage2)
            nis_abs_inc_pp = (nis_stage2 - nis_legacy) * 100.0 if finite(nis_legacy) and finite(nis_stage2) else float("nan")

            committed_legacy = sum(int(r.get("radar_update_committed_count", "0") or 0) for r in m_legacy)
            committed_stage2 = sum(int(r.get("radar_update_committed_count", "0") or 0) for r in m_stage2)
            committed_drop = 1.0 - safe_rate(committed_stage2, committed_legacy) if committed_legacy > 0 else float("nan")

            report["checks"]["nis_and_committed"] = {
                "nis_rate_legacy": nis_legacy,
                "nis_rate_stage2": nis_stage2,
                "nis_abs_increase_pp": nis_abs_inc_pp,
                "committed_legacy": committed_legacy,
                "committed_stage2": committed_stage2,
                "committed_drop_ratio": committed_drop,
            }

            if not finite(nis_abs_inc_pp):
                append_fail(failures, "invalid NIS rates for legacy/event_stage2")
            elif nis_abs_inc_pp > args.max_nis_abs_increase_pp:
                append_fail(failures, "NIS absolute increase too high: %.6f pp > %.6f pp" % (nis_abs_inc_pp, args.max_nis_abs_increase_pp))

            if not finite(committed_drop):
                append_fail(failures, "invalid committed update counts for legacy/event_stage2")
            elif committed_drop > args.max_committed_drop:
                append_fail(failures, "radar committed update drop too high: %.6f > %.6f" % (committed_drop, args.max_committed_drop))

            repeat_groups = defaultdict(list)
            repeatability_modes = {m.strip().lower() for m in args.repeatability_modes.split(",") if m.strip()}
            if not repeatability_modes:
                repeatability_modes = {"event_stage2"}
            report["checks"]["repeatability_modes"] = sorted(repeatability_modes)
            for row in metrics_rows:
                mode = text(row.get("scheduler_mode", "")).lower()
                if mode not in repeatability_modes:
                    continue
                key = (
                    row.get("dataset", ""),
                    row.get("modality", ""),
                    mode,
                    row.get("cov_mode", ""),
                )
                repeat_groups[key].append(row)

            repeat_report = []
            for key, group in repeat_groups.items():
                if len(group) < args.repeat_min_runs:
                    append_fail(
                        failures,
                        "repeatability runs too few for %s: %d < %d" % (key, len(group), args.repeat_min_runs),
                    )
                    continue
                ate_vals = [f64(r.get("ate_rmse", "nan")) for r in group if finite(r.get("ate_rmse", "nan"))]
                rpe_vals = [f64(r.get("rpe_rmse", "nan")) for r in group if finite(r.get("rpe_rmse", "nan"))]
                upd_vals = [f64(r.get("radar_update_committed_count", "nan")) for r in group if finite(r.get("radar_update_committed_count", "nan"))]
                drop_vals = [
                    f64(drop_by_run.get(r.get("run_id", ""), float("nan")))
                    for r in group
                    if finite(drop_by_run.get(r.get("run_id", ""), float("nan")))
                ]

                ate_cv = cv(ate_vals)
                rpe_cv = cv(rpe_vals)
                upd_rr = rel_range(upd_vals)
                drop_rr = rel_range(drop_vals)
                repeat_report.append(
                    {
                        "group": key,
                        "n": len(group),
                        "ate_cv": ate_cv,
                        "rpe_cv": rpe_cv,
                        "update_rel_range": upd_rr,
                        "drop_rel_range": drop_rr,
                    }
                )

                if finite(ate_cv) and ate_cv > args.max_repeat_ate_cv:
                    append_fail(failures, "repeatability ate cv too high for %s: %.6f > %.6f" % (key, ate_cv, args.max_repeat_ate_cv))
                if finite(rpe_cv) and rpe_cv > args.max_repeat_rpe_cv:
                    append_fail(failures, "repeatability rpe cv too high for %s: %.6f > %.6f" % (key, rpe_cv, args.max_repeat_rpe_cv))
                if finite(upd_rr) and upd_rr > args.max_repeat_update_rel_range:
                    append_fail(failures, "repeatability update rel-range too high for %s: %.6f > %.6f" % (key, upd_rr, args.max_repeat_update_rel_range))
                if finite(drop_rr) and drop_rr > args.max_repeat_drop_rel_range:
                    append_fail(failures, "repeatability drop rel-range too high for %s: %.6f > %.6f" % (key, drop_rr, args.max_repeat_drop_rel_range))

            report["checks"]["repeatability"] = repeat_report

    timing_share = {}
    for mode, sums in timing_sum_by_mode.items():
        total = sum(max(v, 0.0) for v in sums.values())
        if total <= 0.0:
            timing_share[mode] = {k: float("nan") for k in TIMING_COLUMNS}
        else:
            timing_share[mode] = {k: sums[k] / total for k in TIMING_COLUMNS}
    report["checks"]["timing_share_by_mode"] = timing_share

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
