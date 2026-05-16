#!/usr/bin/env python3

"""
Gate-W2 checker for baseline freeze & reproducibility.
Pass criteria:
1) snapshot exists and is structurally complete
2) run_manifest has >= min_trials successful runs per dataset+modality
3) baseline metrics/summary exist and fluctuation within threshold
"""

import argparse
import csv
import glob
import json
import math
import os
import sys
from collections import defaultdict


def fail(msg, failures):
    failures.append(msg)


def list_snapshot_dirs(snapshot_root):
    if not os.path.isdir(snapshot_root):
        return []
    return sorted([d for d in glob.glob(os.path.join(snapshot_root, "*")) if os.path.isdir(d)])


def load_manifest(path):
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def is_finite(v):
    try:
        x = float(v)
        return math.isfinite(x)
    except Exception:
        return False


def main():
    parser = argparse.ArgumentParser(description="Gate-W2 baseline checker")
    parser.add_argument("--results_root", required=True)
    parser.add_argument("--snapshot_root", default="")
    parser.add_argument("--manifest", default="")
    parser.add_argument("--metrics", default="")
    parser.add_argument("--min_trials", type=int, default=3)
    parser.add_argument("--max_cv", type=float, default=0.25)
    parser.add_argument("--abs_std_tol", type=float, default=0.05)
    parser.add_argument("--report_json", default="")
    args = parser.parse_args()

    snapshot_root = args.snapshot_root if args.snapshot_root else os.path.join(args.results_root, "snapshots")
    manifest_path = args.manifest if args.manifest else os.path.join(args.results_root, "run_manifest.csv")
    metrics_path = args.metrics if args.metrics else os.path.join(args.results_root, "baseline_v1_metrics.csv")

    failures = []
    report = {
        "gate": "W2",
        "results_root": args.results_root,
        "snapshot_root": snapshot_root,
        "manifest": manifest_path,
        "metrics": metrics_path,
        "min_trials": args.min_trials,
        "checks": {},
    }

    # Check snapshot completeness
    snapshots = list_snapshot_dirs(snapshot_root)
    if not snapshots:
        fail("No snapshot directory found under %s" % snapshot_root, failures)
    else:
        latest = snapshots[-1]
        required = ["metadata.json", "file_sha256.csv", "README.txt"]
        missing = [x for x in required if not os.path.isfile(os.path.join(latest, x))]
        if missing:
            fail("Latest snapshot missing files: %s" % ",".join(missing), failures)
        report["checks"]["latest_snapshot"] = latest

    # Check manifest exists
    if not os.path.isfile(manifest_path):
        fail("Manifest not found: %s" % manifest_path, failures)
        rows = []
    else:
        rows = load_manifest(manifest_path)

    # Repeats per dataset+modality
    succ_count = defaultdict(int)
    for row in rows:
        if row.get("status") == "SUCCESS":
            succ_count[(row.get("dataset", ""), row.get("modality", ""))] += 1

    if not succ_count:
        fail("No SUCCESS rows in manifest", failures)
    else:
        insufficient = [k for k, v in succ_count.items() if v < args.min_trials]
        if insufficient:
            fail("Insufficient repeats (<%d) for: %s" % (args.min_trials, insufficient), failures)
        report["checks"]["success_runs"] = {"groups": len(succ_count), "counts": {"%s|%s" % k: v for k, v in succ_count.items()}}

    # Metrics and fluctuation
    if not os.path.isfile(metrics_path):
        fail("Metrics file not found: %s" % metrics_path, failures)
    else:
        by_group = defaultdict(list)
        with open(metrics_path, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                key = (row.get("dataset", ""), row.get("modality", ""), row.get("feature_num", ""))
                if is_finite(row.get("ate_rmse", "")):
                    by_group[key].append(float(row["ate_rmse"]))

        if not by_group:
            fail("No finite ATE metrics found", failures)
        else:
            noisy_groups = []
            for key, values in by_group.items():
                if len(values) < args.min_trials:
                    noisy_groups.append((key, "insufficient_runs", len(values)))
                    continue
                mean_v = sum(values) / len(values)
                var = sum((x - mean_v) ** 2 for x in values) / len(values)
                std_v = math.sqrt(var)
                if abs(mean_v) > 1.0e-9:
                    cv = std_v / abs(mean_v)
                    if cv > args.max_cv:
                        noisy_groups.append((key, "cv", cv))
                else:
                    if std_v > args.abs_std_tol:
                        noisy_groups.append((key, "std", std_v))

            if noisy_groups:
                fail("Metric fluctuation exceeds threshold: %s" % noisy_groups, failures)
            report["checks"]["metric_groups"] = len(by_group)

    report["pass"] = len(failures) == 0
    report["failures"] = failures

    report_json = args.report_json if args.report_json else os.path.join(args.results_root, "gate_w2_report.json")
    with open(report_json, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)

    print("[gate_w2] report: %s" % report_json)
    if failures:
        print("[gate_w2] FAIL")
        for item in failures:
            print(" - %s" % item)
        sys.exit(1)

    print("[gate_w2] PASS")


if __name__ == "__main__":
    main()
