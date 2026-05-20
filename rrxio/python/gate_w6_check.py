#!/usr/bin/env python3

"""
Gate-W6 checker for alpha_R contribution.
Strict pass criteria:
1) full mode coverage and repeat coverage
2) NIS exceedance improvement (alpha_r vs base):
   - relative drop >= 20%
   - absolute drop >= 1.5 percentage points
3) median ATE/RPE degradation <= 5%
4) median runtime increase <= 15%
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


def median(values):
    vals = [float(v) for v in values if finite(v)]
    if not vals:
        return float("nan")
    return float(statistics.median(vals))


def main():
    parser = argparse.ArgumentParser(description="Gate-W6 alpha_R checker")
    parser.add_argument("--results_root", required=True)
    parser.add_argument("--manifest", default="")
    parser.add_argument("--metrics", default="")
    parser.add_argument("--report_json", default="")
    parser.add_argument("--min_trials", type=int, default=3)
    parser.add_argument("--min_nis_relative_drop", type=float, default=0.20)
    parser.add_argument("--min_nis_abs_drop_pp", type=float, default=1.5)
    parser.add_argument("--max_ate_degrade", type=float, default=0.05)
    parser.add_argument("--max_rpe_degrade", type=float, default=0.05)
    parser.add_argument("--max_runtime_increase", type=float, default=0.15)
    args = parser.parse_args()

    manifest = args.manifest if args.manifest else os.path.join(args.results_root, "run_manifest.csv")
    metrics = args.metrics if args.metrics else os.path.join(args.results_root, "w6_metrics.csv")
    report_json = args.report_json if args.report_json else os.path.join(args.results_root, "gate_w6_report.json")

    failures = []
    report = {
        "gate": "W6",
        "results_root": args.results_root,
        "manifest": manifest,
        "metrics": metrics,
        "criteria": {
            "min_nis_relative_drop": args.min_nis_relative_drop,
            "min_nis_abs_drop_pp": args.min_nis_abs_drop_pp,
            "max_ate_degrade": args.max_ate_degrade,
            "max_rpe_degrade": args.max_rpe_degrade,
            "max_runtime_increase": args.max_runtime_increase,
        },
        "checks": {},
        "failures": [],
        "pass": False,
    }

    if not os.path.isfile(manifest):
        append_fail(failures, "manifest not found: %s" % manifest)
        manifest_rows = []
    else:
        with open(manifest, "r", encoding="utf-8") as f:
            manifest_rows = list(csv.DictReader(f))

    if not os.path.isfile(metrics):
        append_fail(failures, "metrics not found: %s" % metrics)
        metric_rows = []
    else:
        with open(metrics, "r", encoding="utf-8") as f:
            metric_rows = list(csv.DictReader(f))

    required_modes = ["base", "fixed", "alpha_r"]
    success_rows = [r for r in manifest_rows if r.get("status", "") == "SUCCESS"]
    mode_counts = defaultdict(int)
    coverage = defaultdict(int)
    for row in success_rows:
        mode = row.get("cov_mode", "").strip()
        mode_counts[mode] += 1
        key = (mode, row.get("dataset", ""), row.get("modality", ""))
        coverage[key] += 1

    report["checks"]["mode_success_counts"] = dict(mode_counts)
    for mode in required_modes:
        if mode_counts.get(mode, 0) == 0:
            append_fail(failures, "no SUCCESS runs for cov_mode=%s" % mode)

    for mode in required_modes:
        insufficient = [k for k, c in coverage.items() if k[0] == mode and c < args.min_trials]
        if insufficient:
            append_fail(failures, "insufficient repeats for mode=%s: %s" % (mode, insufficient))

    by_mode = defaultdict(list)
    for row in metric_rows:
        by_mode[row.get("cov_mode", "").strip()].append(row)

    for mode in required_modes:
        if mode not in by_mode or not by_mode[mode]:
            append_fail(failures, "metrics missing for mode=%s" % mode)

    def aggregate_nis(mode):
        valid_total = 0
        exceed_total = 0
        for row in by_mode.get(mode, []):
            try:
                valid_total += int(row.get("nis_valid_count", "0"))
                exceed_total += int(row.get("nis_exceed_count", "0"))
            except Exception:
                pass
        rho = float("nan")
        if valid_total > 0:
            rho = float(exceed_total) / float(valid_total)
        return rho, valid_total, exceed_total

    rho_base, valid_base, exceed_base = aggregate_nis("base")
    rho_alpha, valid_alpha, exceed_alpha = aggregate_nis("alpha_r")
    report["checks"]["nis"] = {
        "base": {"rho": rho_base, "valid_total": valid_base, "exceed_total": exceed_base},
        "alpha_r": {"rho": rho_alpha, "valid_total": valid_alpha, "exceed_total": exceed_alpha},
    }

    if not finite(rho_base) or not finite(rho_alpha) or valid_base <= 0 or valid_alpha <= 0:
        append_fail(failures, "invalid NIS aggregation for base/alpha_r")
        rel_drop = float("nan")
        abs_drop_pp = float("nan")
    else:
        if rho_base <= 0.0:
            append_fail(failures, "base NIS exceedance rate is non-positive; cannot evaluate relative improvement")
        rel_drop = (rho_base - rho_alpha) / max(rho_base, 1.0e-12)
        abs_drop_pp = (rho_base - rho_alpha) * 100.0
        report["checks"]["nis"]["relative_drop"] = rel_drop
        report["checks"]["nis"]["absolute_drop_pp"] = abs_drop_pp
        if rel_drop < args.min_nis_relative_drop:
            append_fail(failures, "relative NIS drop too small: %.6f < %.6f" % (rel_drop, args.min_nis_relative_drop))
        if abs_drop_pp < args.min_nis_abs_drop_pp:
            append_fail(failures, "absolute NIS drop too small: %.6f pp < %.6f pp" % (abs_drop_pp, args.min_nis_abs_drop_pp))

    ate_base = median([r.get("ate_rmse", "nan") for r in by_mode.get("base", [])])
    ate_alpha = median([r.get("ate_rmse", "nan") for r in by_mode.get("alpha_r", [])])
    rpe_base = median([r.get("rpe_rmse", "nan") for r in by_mode.get("base", [])])
    rpe_alpha = median([r.get("rpe_rmse", "nan") for r in by_mode.get("alpha_r", [])])
    rt_base = median([r.get("runtime_s", "nan") for r in by_mode.get("base", [])])
    rt_alpha = median([r.get("runtime_s", "nan") for r in by_mode.get("alpha_r", [])])

    report["checks"]["median"] = {
        "ate_base": ate_base,
        "ate_alpha": ate_alpha,
        "rpe_base": rpe_base,
        "rpe_alpha": rpe_alpha,
        "runtime_base": rt_base,
        "runtime_alpha": rt_alpha,
    }

    if not finite(ate_base) or not finite(ate_alpha):
        append_fail(failures, "invalid median ATE")
    if not finite(rpe_base) or not finite(rpe_alpha):
        append_fail(failures, "invalid median RPE")
    if not finite(rt_base) or not finite(rt_alpha):
        append_fail(failures, "invalid median runtime")

    if finite(ate_base) and finite(ate_alpha):
        ate_degrade = (ate_alpha - ate_base) / max(abs(ate_base), 1.0e-12)
        report["checks"]["median"]["ate_degrade"] = ate_degrade
        if ate_degrade > args.max_ate_degrade:
            append_fail(failures, "ATE degrade too high: %.6f > %.6f" % (ate_degrade, args.max_ate_degrade))

    if finite(rpe_base) and finite(rpe_alpha):
        rpe_degrade = (rpe_alpha - rpe_base) / max(abs(rpe_base), 1.0e-12)
        report["checks"]["median"]["rpe_degrade"] = rpe_degrade
        if rpe_degrade > args.max_rpe_degrade:
            append_fail(failures, "RPE degrade too high: %.6f > %.6f" % (rpe_degrade, args.max_rpe_degrade))

    if finite(rt_base) and finite(rt_alpha):
        rt_increase = (rt_alpha - rt_base) / max(abs(rt_base), 1.0e-12)
        report["checks"]["median"]["runtime_increase"] = rt_increase
        if rt_increase > args.max_runtime_increase:
            append_fail(failures, "Runtime increase too high: %.6f > %.6f" % (rt_increase, args.max_runtime_increase))

    report["pass"] = len(failures) == 0
    report["failures"] = failures
    with open(report_json, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)

    print("[gate_w6] report: %s" % report_json)
    if failures:
        print("[gate_w6] FAIL")
        for item in failures:
            print(" - %s" % item)
        sys.exit(1)

    print("[gate_w6] PASS")


if __name__ == "__main__":
    main()
