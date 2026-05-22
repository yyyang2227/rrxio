#!/usr/bin/env python3

"""
Gate checker for W7-W8 directional covariance shaping (S_k).
Checks:
1) Required modes/runs exist (alpha_r and alpha_r_sk)
2) Jump proxy improvement: rpe_p95 median reduction >= 20%
3) ATE/RPE/runtime/NIS/committed-count guardrails
4) dvc_diag schema completeness and S_k numeric validity
5) radar_starved must remain zero
"""

import argparse
import csv
import json
import math
import os
import statistics
import sys
from collections import defaultdict


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


def median(values):
    vals = [float(v) for v in values if finite(v)]
    if not vals:
        return float("nan")
    return float(statistics.median(vals))


def safe_rate(numer, denom):
    if denom <= 0:
        return float("nan")
    return float(numer) / float(denom)


def load_csv_rows(path):
    with open(path, "r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def check_diag_file(path, require_sk_columns):
    info = {
        "path": path,
        "rows": 0,
        "missing_columns": [],
        "sk_rows": 0,
        "sk_invalid_rows": 0,
    }
    failures = []

    if not os.path.isfile(path):
        append_fail(failures, "diag file missing: %s" % path)
        return info, failures

    required_cols = [
        "timestamp",
        "cond",
        "inlier_ratio",
        "trace_R_used",
        "minEig_R_used",
        "use_radar_update",
        "cov_mode",
        "d_r",
        "alpha_r",
        "nis_vel",
        "nis_valid",
        "nis_exceed_95",
        "radar_update_committed",
    ]
    if require_sk_columns:
        required_cols.extend(
            [
                "s_k_valid",
                "lambda1_obs",
                "lambda2_obs",
                "lambda3_obs",
                "s1",
                "s2",
                "s3",
                "trace_R_after_alpha",
                "trace_R_after_sk",
            ]
        )

    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames or []
        for col in required_cols:
            if col not in fields:
                info["missing_columns"].append(col)
        if info["missing_columns"]:
            append_fail(failures, "diag missing columns in %s: %s" % (path, info["missing_columns"]))
            return info, failures

        for row in reader:
            info["rows"] += 1
            if text(row.get("s_k_valid", "")) != "1":
                continue
            info["sk_rows"] += 1
            min_eig = row.get("minEig_R_used", "nan")
            s1 = row.get("s1", "nan")
            s2 = row.get("s2", "nan")
            s3 = row.get("s3", "nan")
            if (not finite(min_eig)) or f64(min_eig) <= 0.0:
                info["sk_invalid_rows"] += 1
                continue
            if (not finite(s1)) or (not finite(s2)) or (not finite(s3)):
                info["sk_invalid_rows"] += 1
                continue
            if f64(s1) < 1.0 or f64(s2) < 1.0 or f64(s3) < 1.0:
                info["sk_invalid_rows"] += 1

    if info["rows"] == 0:
        append_fail(failures, "diag file has no rows: %s" % path)
    if require_sk_columns and info["sk_rows"] == 0:
        append_fail(failures, "diag file has no s_k_valid=1 rows: %s" % path)
    if info["sk_invalid_rows"] > 0:
        append_fail(failures, "diag file has invalid S_k rows in %s: %d" % (path, info["sk_invalid_rows"]))
    return info, failures


def check_sched_starved(path):
    if not os.path.isfile(path):
        return 0, ["sched diag file missing: %s" % path]
    starved = 0
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if "radar_starved" not in (reader.fieldnames or []):
            return 0, ["sched diag missing radar_starved column: %s" % path]
        for row in reader:
            if text(row.get("radar_starved", "")) == "1":
                starved += 1
    return starved, []


def rel_change(new_v, old_v):
    if (not finite(new_v)) or (not finite(old_v)) or abs(old_v) < 1.0e-12:
        return float("nan")
    return (float(new_v) - float(old_v)) / abs(float(old_v))


def main():
    parser = argparse.ArgumentParser(description="Gate checker for W8 S_k")
    parser.add_argument("--results_root", required=True)
    parser.add_argument("--manifest", default="")
    parser.add_argument("--metrics", default="")
    parser.add_argument("--report_json", default="")
    parser.add_argument("--stage_filter", default="", help="Only include rows with this exact stage tag")
    parser.add_argument("--mode_base", default="alpha_r")
    parser.add_argument("--mode_sk", default="alpha_r_sk")
    parser.add_argument("--min_rpe_p95_improve", type=float, default=0.20)
    parser.add_argument("--max_ate_degrade", type=float, default=0.05)
    parser.add_argument("--max_rpe_degrade", type=float, default=0.05)
    parser.add_argument("--max_runtime_increase", type=float, default=0.15)
    parser.add_argument("--max_nis_abs_increase_pp", type=float, default=1.0)
    parser.add_argument("--max_committed_drop", type=float, default=0.05)
    args = parser.parse_args()

    manifest = args.manifest if args.manifest else os.path.join(args.results_root, "run_manifest.csv")
    metrics = args.metrics if args.metrics else os.path.join(args.results_root, "w8_metrics.csv")
    report_json = args.report_json if args.report_json else os.path.join(args.results_root, "gate_w8_report.json")

    report = {
        "gate": "W8",
        "results_root": args.results_root,
        "manifest": manifest,
        "metrics": metrics,
        "criteria": {
            "stage_filter": args.stage_filter,
            "mode_base": args.mode_base,
            "mode_sk": args.mode_sk,
            "min_rpe_p95_improve": args.min_rpe_p95_improve,
            "max_ate_degrade": args.max_ate_degrade,
            "max_rpe_degrade": args.max_rpe_degrade,
            "max_runtime_increase": args.max_runtime_increase,
            "max_nis_abs_increase_pp": args.max_nis_abs_increase_pp,
            "max_committed_drop": args.max_committed_drop,
        },
        "checks": {},
        "failures": [],
        "pass": False,
    }
    failures = []

    if not os.path.isfile(manifest):
        append_fail(failures, "manifest not found: %s" % manifest)
        manifest_rows = []
    else:
        manifest_rows = load_csv_rows(manifest)
    if args.stage_filter:
        manifest_rows = [r for r in manifest_rows if text(r.get("stage", "")) == args.stage_filter]

    if not os.path.isfile(metrics):
        append_fail(failures, "metrics not found: %s" % metrics)
        metric_rows = []
    else:
        metric_rows = load_csv_rows(metrics)
    if args.stage_filter and metric_rows and "stage" in metric_rows[0]:
        metric_rows = [r for r in metric_rows if text(r.get("stage", "")) == args.stage_filter]

    success_manifest = [r for r in manifest_rows if r.get("status", "") == "SUCCESS"]
    by_mode_manifest = defaultdict(list)
    for row in success_manifest:
        by_mode_manifest[text(row.get("cov_mode", "")).lower()].append(row)

    mode_base = args.mode_base.strip().lower()
    mode_sk = args.mode_sk.strip().lower()
    if len(by_mode_manifest.get(mode_base, [])) == 0:
        append_fail(failures, "no SUCCESS run for cov_mode=%s" % mode_base)
    if len(by_mode_manifest.get(mode_sk, [])) == 0:
        append_fail(failures, "no SUCCESS run for cov_mode=%s" % mode_sk)

    by_mode_metrics = defaultdict(list)
    for row in metric_rows:
        mode = text(row.get("cov_mode", "")).lower()
        by_mode_metrics[mode].append(row)
    if len(by_mode_metrics.get(mode_base, [])) == 0:
        append_fail(failures, "no metrics rows for cov_mode=%s" % mode_base)
    if len(by_mode_metrics.get(mode_sk, [])) == 0:
        append_fail(failures, "no metrics rows for cov_mode=%s" % mode_sk)

    base_rows = by_mode_metrics.get(mode_base, [])
    sk_rows = by_mode_metrics.get(mode_sk, [])
    ate_base = median([r.get("ate_rmse", "nan") for r in base_rows])
    ate_sk = median([r.get("ate_rmse", "nan") for r in sk_rows])
    rpe_base = median([r.get("rpe_rmse", "nan") for r in base_rows])
    rpe_sk = median([r.get("rpe_rmse", "nan") for r in sk_rows])
    rpe_p95_base = median([r.get("rpe_p95", "nan") for r in base_rows])
    rpe_p95_sk = median([r.get("rpe_p95", "nan") for r in sk_rows])
    rt_base = median([r.get("runtime_s", "nan") for r in base_rows])
    rt_sk = median([r.get("runtime_s", "nan") for r in sk_rows])

    valid_base = sum(i64(r.get("nis_valid_count", "0")) for r in base_rows)
    exceed_base = sum(i64(r.get("nis_exceed_count", "0")) for r in base_rows)
    valid_sk = sum(i64(r.get("nis_valid_count", "0")) for r in sk_rows)
    exceed_sk = sum(i64(r.get("nis_exceed_count", "0")) for r in sk_rows)
    rho_base = safe_rate(exceed_base, valid_base)
    rho_sk = safe_rate(exceed_sk, valid_sk)

    committed_base = sum(i64(r.get("radar_update_committed_count", "0")) for r in base_rows)
    committed_sk = sum(i64(r.get("radar_update_committed_count", "0")) for r in sk_rows)

    rpe_p95_improve = safe_rate((rpe_p95_base - rpe_p95_sk), rpe_p95_base)
    ate_degrade = rel_change(ate_sk, ate_base)
    rpe_degrade = rel_change(rpe_sk, rpe_base)
    runtime_increase = rel_change(rt_sk, rt_base)
    nis_abs_inc_pp = (rho_sk - rho_base) * 100.0 if finite(rho_sk) and finite(rho_base) else float("nan")
    committed_drop = 1.0 - safe_rate(committed_sk, committed_base) if committed_base > 0 else float("nan")

    report["checks"]["aggregate"] = {
        "ate_base": ate_base,
        "ate_sk": ate_sk,
        "rpe_base": rpe_base,
        "rpe_sk": rpe_sk,
        "rpe_p95_base": rpe_p95_base,
        "rpe_p95_sk": rpe_p95_sk,
        "runtime_base": rt_base,
        "runtime_sk": rt_sk,
        "nis_rho_base": rho_base,
        "nis_rho_sk": rho_sk,
        "committed_base": committed_base,
        "committed_sk": committed_sk,
        "rpe_p95_improve": rpe_p95_improve,
        "ate_degrade": ate_degrade,
        "rpe_degrade": rpe_degrade,
        "runtime_increase": runtime_increase,
        "nis_abs_inc_pp": nis_abs_inc_pp,
        "committed_drop": committed_drop,
    }

    if (not finite(rpe_p95_improve)) or rpe_p95_improve < args.min_rpe_p95_improve:
        append_fail(
            failures,
            "rpe_p95 improvement failed: got=%s need >= %.6f"
            % (str(rpe_p95_improve), args.min_rpe_p95_improve),
        )
    if (not finite(ate_degrade)) or ate_degrade > args.max_ate_degrade:
        append_fail(failures, "ATE degrade failed: got=%s need <= %.6f" % (str(ate_degrade), args.max_ate_degrade))
    if (not finite(rpe_degrade)) or rpe_degrade > args.max_rpe_degrade:
        append_fail(failures, "RPE degrade failed: got=%s need <= %.6f" % (str(rpe_degrade), args.max_rpe_degrade))
    if (not finite(runtime_increase)) or runtime_increase > args.max_runtime_increase:
        append_fail(
            failures,
            "runtime increase failed: got=%s need <= %.6f" % (str(runtime_increase), args.max_runtime_increase),
        )
    if (not finite(nis_abs_inc_pp)) or nis_abs_inc_pp > args.max_nis_abs_increase_pp:
        append_fail(
            failures,
            "NIS absolute increase failed: got=%s pp need <= %.6f pp"
            % (str(nis_abs_inc_pp), args.max_nis_abs_increase_pp),
        )
    if (not finite(committed_drop)) or committed_drop > args.max_committed_drop:
        append_fail(
            failures,
            "committed drop failed: got=%s need <= %.6f" % (str(committed_drop), args.max_committed_drop),
        )

    diag_checks = []
    total_starved = 0
    total_sk_rows = 0
    for mode, rows in ((mode_base, by_mode_manifest.get(mode_base, [])), (mode_sk, by_mode_manifest.get(mode_sk, []))):
        require_sk_cols = (mode == mode_sk)
        for row in rows:
            diag_file = text(row.get("diag_file", ""))
            if not diag_file:
                continue
            diag_info, local_failures = check_diag_file(diag_file, require_sk_cols)
            diag_checks.append(diag_info)
            failures.extend(local_failures)
            total_sk_rows += int(diag_info.get("sk_rows", 0))

            sched_diag_file = text(row.get("sched_diag_file", ""))
            if sched_diag_file:
                starved_count, starved_failures = check_sched_starved(sched_diag_file)
                total_starved += starved_count
                failures.extend(starved_failures)

    report["checks"]["diag_files"] = diag_checks
    report["checks"]["total_radar_starved"] = total_starved
    report["checks"]["total_sk_rows"] = total_sk_rows
    if total_starved != 0:
        append_fail(failures, "radar_starved must be 0, got=%d" % total_starved)
    if total_sk_rows <= 0:
        append_fail(failures, "no s_k_valid rows observed in target mode")

    report["failures"] = failures
    report["pass"] = len(failures) == 0

    with open(report_json, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    print(json.dumps(report, indent=2, ensure_ascii=False))
    sys.exit(0 if report["pass"] else 1)


if __name__ == "__main__":
    main()
