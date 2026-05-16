#!/usr/bin/env python3

"""
Summarize baseline runs for W1-W2 gate checks.
Inputs: run_manifest.csv + per-run trajectory files.
Outputs:
- baseline_v1_metrics.csv (per-run metrics)
- baseline_v1_summary.md (grouped statistics)
"""

import argparse
import csv
import math
import os
from collections import defaultdict

import numpy as np


def load_traj(path):
    data = np.genfromtxt(path, delimiter=" ")
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 4:
        raise RuntimeError("Trajectory file has insufficient columns: %s" % path)
    return data


def associate_by_time(gt, est, max_dt=0.05):
    i, j = 0, 0
    pairs = []
    while i < gt.shape[0] and j < est.shape[0]:
        dt = est[j, 0] - gt[i, 0]
        if abs(dt) <= max_dt:
            pairs.append((i, j))
            i += 1
            j += 1
        elif dt > 0:
            i += 1
        else:
            j += 1
    if not pairs:
        return None, None
    gi = np.array([p[0] for p in pairs], dtype=np.int64)
    ej = np.array([p[1] for p in pairs], dtype=np.int64)
    return gt[gi, 1:4], est[ej, 1:4]


def umeyama_rigid(src, dst):
    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)
    src_c = src - mu_src
    dst_c = dst - mu_dst

    cov = (src_c.T @ dst_c) / max(1, src.shape[0])
    u, _, vt = np.linalg.svd(cov)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1
        r = vt.T @ u.T
    t = mu_dst - r @ mu_src
    return r, t


def compute_metrics(gt_path, est_path, rpe_delta=10):
    gt = load_traj(gt_path)
    est = load_traj(est_path)

    gt_p, est_p = associate_by_time(gt, est)
    if gt_p is None or gt_p.shape[0] < max(3, rpe_delta + 1):
        return float("nan"), float("nan"), 0

    r, t = umeyama_rigid(est_p, gt_p)
    est_aligned = (r @ est_p.T).T + t

    ate_err = np.linalg.norm(est_aligned - gt_p, axis=1)
    ate_rmse = float(np.sqrt(np.mean(ate_err ** 2)))

    rpe_err = np.linalg.norm((est_aligned[rpe_delta:] - est_aligned[:-rpe_delta]) - (gt_p[rpe_delta:] - gt_p[:-rpe_delta]), axis=1)
    rpe_rmse = float(np.sqrt(np.mean(rpe_err ** 2)))

    return ate_rmse, rpe_rmse, int(gt_p.shape[0])


def get_estimate_file(run_dir, run_id):
    trial_specific = os.path.join(run_dir, "stamped_traj_estimate_%s.txt" % run_id)
    default_file = os.path.join(run_dir, "stamped_traj_estimate.txt")
    if os.path.isfile(trial_specific):
        return trial_specific
    return default_file


def as_float(v):
    try:
        return float(v)
    except Exception:
        return float("nan")


def main():
    parser = argparse.ArgumentParser(description="Summarize W1-W2 baseline metrics")
    parser.add_argument("--results_root", required=True)
    parser.add_argument("--manifest", default="")
    parser.add_argument("--rpe_delta", type=int, default=10)
    args = parser.parse_args()

    manifest = args.manifest if args.manifest else os.path.join(args.results_root, "run_manifest.csv")
    if not os.path.isfile(manifest):
        raise RuntimeError("Manifest not found: %s" % manifest)

    metrics_csv = os.path.join(args.results_root, "baseline_v1_metrics.csv")
    summary_md = os.path.join(args.results_root, "baseline_v1_summary.md")

    rows = []
    with open(manifest, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get("status", "") != "SUCCESS":
                continue

            run_dir = row.get("export_directory", "")
            if not run_dir:
                continue
            gt_path = os.path.join(run_dir, "stamped_groundtruth.txt")
            est_path = get_estimate_file(run_dir, row.get("run_id", ""))
            if not (os.path.isfile(gt_path) and os.path.isfile(est_path)):
                continue

            ate_rmse, rpe_rmse, nmatch = compute_metrics(gt_path, est_path, rpe_delta=args.rpe_delta)
            rows.append({
                "run_id": row.get("run_id", ""),
                "dataset": row.get("dataset", ""),
                "modality": row.get("modality", ""),
                "feature_num": row.get("feature_num", ""),
                "status": row.get("status", ""),
                "runtime_s": as_float(row.get("runtime_s", "nan")),
                "ate_rmse": ate_rmse,
                "rpe_rmse": rpe_rmse,
                "rmse": ate_rmse,
                "n_match": nmatch,
            })

    with open(metrics_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "run_id", "dataset", "modality", "feature_num", "status", "runtime_s", "ate_rmse", "rpe_rmse", "rmse", "n_match"
        ])
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

    groups = defaultdict(list)
    for row in rows:
        key = (row["dataset"], row["modality"], row["feature_num"])
        groups[key].append(row)

    with open(summary_md, "w", encoding="utf-8") as f:
        f.write("# Baseline v1 Summary\n\n")
        f.write("| dataset | modality | feature | n_runs | ate_rmse_mean | ate_rmse_std | rpe_rmse_mean | rpe_rmse_std | runtime_mean_s |\n")
        f.write("|---|---|---:|---:|---:|---:|---:|---:|---:|\n")
        for key in sorted(groups.keys()):
            vals = groups[key]
            ate = np.array([v["ate_rmse"] for v in vals], dtype=np.float64)
            rpe = np.array([v["rpe_rmse"] for v in vals], dtype=np.float64)
            rt = np.array([v["runtime_s"] for v in vals], dtype=np.float64)

            def safe_mean(x):
                return float(np.nanmean(x)) if np.isfinite(x).any() else float("nan")

            def safe_std(x):
                return float(np.nanstd(x)) if np.isfinite(x).any() else float("nan")

            f.write("| %s | %s | %s | %d | %.6f | %.6f | %.6f | %.6f | %.3f |\n" % (
                key[0], key[1], key[2], len(vals),
                safe_mean(ate), safe_std(ate), safe_mean(rpe), safe_std(rpe), safe_mean(rt)
            ))

    print("[summary] wrote: %s" % metrics_csv)
    print("[summary] wrote: %s" % summary_md)


if __name__ == "__main__":
    main()
