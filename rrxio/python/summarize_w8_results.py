#!/usr/bin/env python3

"""
Summarize W7-W8 experiment runs.
Inputs:
- run_manifest.csv
- per-run trajectory files
- per-run dvc_diag_*.csv
- per-run dvc_sched_diag_*.csv
Outputs:
- w8_metrics.csv
- w8_summary.md
- w8_compare_metrics.png
- w8_compare_rpe_p95.png
- w8_compare_nis.png
"""

import argparse
import csv
import math
import os
from collections import defaultdict

import matplotlib.pyplot as plt
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


def percentile(values, q):
    vals = np.array(values, dtype=np.float64)
    vals = vals[np.isfinite(vals)]
    if vals.size == 0:
        return float("nan")
    vals.sort()
    if vals.size == 1:
        return float(vals[0])
    pos = (vals.size - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return float(vals[lo])
    w = pos - lo
    return float(vals[lo] * (1.0 - w) + vals[hi] * w)


def compute_metrics(gt_path, est_path, rpe_delta=10):
    gt = load_traj(gt_path)
    est = load_traj(est_path)
    gt_p, est_p = associate_by_time(gt, est)
    if gt_p is None or gt_p.shape[0] < max(3, rpe_delta + 1):
        return float("nan"), float("nan"), float("nan"), 0

    r, t = umeyama_rigid(est_p, gt_p)
    est_aligned = (r @ est_p.T).T + t

    ate_err = np.linalg.norm(est_aligned - gt_p, axis=1)
    ate_rmse = float(np.sqrt(np.mean(ate_err ** 2)))

    rel = (est_aligned[rpe_delta:] - est_aligned[:-rpe_delta]) - (gt_p[rpe_delta:] - gt_p[:-rpe_delta])
    rpe_err = np.linalg.norm(rel, axis=1)
    rpe_rmse = float(np.sqrt(np.mean(rpe_err ** 2)))
    rpe_p95 = percentile(rpe_err, 0.95)
    return ate_rmse, rpe_rmse, rpe_p95, int(gt_p.shape[0])


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


def safe_median(values):
    vals = np.array(values, dtype=np.float64)
    vals = vals[np.isfinite(vals)]
    if vals.size == 0:
        return float("nan")
    return float(np.median(vals))


def safe_rate(numer, denom):
    if denom <= 0:
        return float("nan")
    return float(numer) / float(denom)


def read_diag_stats(diag_file):
    if not os.path.isfile(diag_file):
        return 0, 0, 0, 0, 0
    rows = 0
    nis_valid_count = 0
    nis_exceed_count = 0
    committed_count = 0
    sk_applied_count = 0
    with open(diag_file, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows += 1
            if row.get("nis_valid", "").strip() == "1":
                nis_valid_count += 1
                if row.get("nis_exceed_95", "").strip() == "1":
                    nis_exceed_count += 1
            if row.get("radar_update_committed", "").strip() == "1":
                committed_count += 1
            if row.get("s_k_valid", "").strip() == "1":
                sk_applied_count += 1
    return rows, nis_valid_count, nis_exceed_count, committed_count, sk_applied_count


def read_sched_starved_count(path):
    if not os.path.isfile(path):
        return 0
    cnt = 0
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get("radar_starved", "").strip() == "1":
                cnt += 1
    return cnt


def write_compare_plots(results_root, by_mode):
    modes = sorted(by_mode.keys())
    if not modes:
        return "", "", ""

    ate_vals = []
    rpe_vals = []
    runtime_vals = []
    rpe_p95_vals = []
    nis_vals = []
    for mode in modes:
        mode_rows = by_mode[mode]
        ate_vals.append(safe_median([r["ate_rmse"] for r in mode_rows]))
        rpe_vals.append(safe_median([r["rpe_rmse"] for r in mode_rows]))
        runtime_vals.append(safe_median([r["runtime_s"] for r in mode_rows]))
        rpe_p95_vals.append(safe_median([r["rpe_p95"] for r in mode_rows]))
        valid_total = sum(int(r["nis_valid_count"]) for r in mode_rows)
        exceed_total = sum(int(r["nis_exceed_count"]) for r in mode_rows)
        nis_vals.append(safe_rate(exceed_total, valid_total))

    fig_metrics = os.path.join(results_root, "w8_compare_metrics.png")
    fig_rpe_p95 = os.path.join(results_root, "w8_compare_rpe_p95.png")
    fig_nis = os.path.join(results_root, "w8_compare_nis.png")

    x = np.arange(len(modes))
    w = 0.25
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.bar(x - w, ate_vals, width=w, label="ATE RMSE")
    ax.bar(x, rpe_vals, width=w, label="RPE RMSE")
    ax.bar(x + w, runtime_vals, width=w, label="Runtime(s)")
    ax.set_xticks(x)
    ax.set_xticklabels(modes)
    ax.set_title("W8 Mode Comparison (Median)")
    ax.grid(axis="y", alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(fig_metrics, dpi=180)
    plt.close(fig)

    fig2, ax2 = plt.subplots(figsize=(7, 4.2))
    ax2.bar(modes, rpe_p95_vals, color="#4e79a7")
    ax2.set_title("W8 Jump Proxy (RPE P95)")
    ax2.set_ylabel("RPE P95")
    ax2.grid(axis="y", alpha=0.25)
    fig2.tight_layout()
    fig2.savefig(fig_rpe_p95, dpi=180)
    plt.close(fig2)

    fig3, ax3 = plt.subplots(figsize=(7, 4.2))
    ax3.bar(modes, nis_vals, color="#d97a3a")
    ax3.set_title("W8 NIS Exceedance Rate")
    ax3.set_ylabel("rho_nis_exceed")
    ax3.grid(axis="y", alpha=0.25)
    fig3.tight_layout()
    fig3.savefig(fig_nis, dpi=180)
    plt.close(fig3)

    return fig_metrics, fig_rpe_p95, fig_nis


def main():
    parser = argparse.ArgumentParser(description="Summarize W8 S_k results")
    parser.add_argument("--results_root", required=True)
    parser.add_argument("--manifest", default="")
    parser.add_argument("--stage", default="W8")
    parser.add_argument("--stage_filter", default="", help="Only include rows with this exact manifest stage tag")
    parser.add_argument("--rpe_delta", type=int, default=10)
    args = parser.parse_args()

    manifest = args.manifest if args.manifest else os.path.join(args.results_root, "run_manifest.csv")
    if not os.path.isfile(manifest):
        raise RuntimeError("Manifest not found: %s" % manifest)

    metrics_csv = os.path.join(args.results_root, "w8_metrics.csv")
    summary_md = os.path.join(args.results_root, "w8_summary.md")

    rows = []
    with open(manifest, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get("status", "") != "SUCCESS":
                continue
            if args.stage_filter and row.get("stage", "") != args.stage_filter:
                continue

            run_id = row.get("run_id", "")
            run_dir = row.get("export_directory", "")
            if not run_dir:
                continue
            gt_path = os.path.join(run_dir, "stamped_groundtruth.txt")
            est_path = get_estimate_file(run_dir, run_id)
            ate_rmse = float("nan")
            rpe_rmse = float("nan")
            rpe_p95 = float("nan")
            nmatch = 0
            if os.path.isfile(gt_path) and os.path.isfile(est_path):
                ate_rmse, rpe_rmse, rpe_p95, nmatch = compute_metrics(gt_path, est_path, rpe_delta=args.rpe_delta)

            diag_file = row.get("diag_file", "")
            sched_diag_file = row.get("sched_diag_file", "")
            diag_rows, nis_valid_count, nis_exceed_count, committed_count, sk_applied_count = read_diag_stats(diag_file)
            starved_count = read_sched_starved_count(sched_diag_file)
            nis_rate = float("nan")
            if nis_valid_count > 0:
                nis_rate = float(nis_exceed_count) / float(nis_valid_count)

            rows.append(
                {
                    "run_id": run_id,
                    "dataset": row.get("dataset", ""),
                    "modality": row.get("modality", ""),
                    "feature_num": row.get("feature_num", ""),
                    "cov_mode": row.get("cov_mode", ""),
                    "scheduler_mode": row.get("scheduler_mode", ""),
                    "stage": row.get("stage", ""),
                    "config_tag": row.get("config_tag", ""),
                    "status": row.get("status", ""),
                    "runtime_s": as_float(row.get("runtime_s", "nan")),
                    "ate_rmse": ate_rmse,
                    "rpe_rmse": rpe_rmse,
                    "rpe_p95": rpe_p95,
                    "n_match": nmatch,
                    "diag_rows": diag_rows,
                    "nis_valid_count": nis_valid_count,
                    "nis_exceed_count": nis_exceed_count,
                    "nis_exceed_rate": nis_rate,
                    "radar_update_committed_count": committed_count,
                    "radar_starved_count": starved_count,
                    "s_k_applied_count": sk_applied_count,
                }
            )

    with open(metrics_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "run_id",
                "dataset",
                "modality",
                "feature_num",
                "cov_mode",
                "scheduler_mode",
                "stage",
                "config_tag",
                "status",
                "runtime_s",
                "ate_rmse",
                "rpe_rmse",
                "rpe_p95",
                "n_match",
                "diag_rows",
                "nis_valid_count",
                "nis_exceed_count",
                "nis_exceed_rate",
                "radar_update_committed_count",
                "radar_starved_count",
                "s_k_applied_count",
            ],
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

    by_mode = defaultdict(list)
    for row in rows:
        by_mode[row["cov_mode"]].append(row)

    with open(summary_md, "w", encoding="utf-8") as f:
        f.write("# W8 S_k Summary\n\n")
        f.write("stage: %s\n\n" % args.stage)
        f.write(
            "| cov_mode | n_runs | ate_median | rpe_median | rpe_p95_median | runtime_median_s | rho_nis_exceed | committed_total | starved_total | s_k_applied_total |\n"
        )
        f.write("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n")
        for mode in sorted(by_mode.keys()):
            mode_rows = by_mode[mode]
            ate_med = safe_median([r["ate_rmse"] for r in mode_rows])
            rpe_med = safe_median([r["rpe_rmse"] for r in mode_rows])
            rpe_p95_med = safe_median([r["rpe_p95"] for r in mode_rows])
            rt_med = safe_median([r["runtime_s"] for r in mode_rows])
            valid_total = sum(int(r["nis_valid_count"]) for r in mode_rows)
            exceed_total = sum(int(r["nis_exceed_count"]) for r in mode_rows)
            committed_total = sum(int(r["radar_update_committed_count"]) for r in mode_rows)
            starved_total = sum(int(r["radar_starved_count"]) for r in mode_rows)
            sk_total = sum(int(r["s_k_applied_count"]) for r in mode_rows)
            rho = float("nan")
            if valid_total > 0:
                rho = float(exceed_total) / float(valid_total)
            f.write(
                "| %s | %d | %.6f | %.6f | %.6f | %.3f | %.6f | %d | %d | %d |\n"
                % (mode, len(mode_rows), ate_med, rpe_med, rpe_p95_med, rt_med, rho, committed_total, starved_total, sk_total)
            )

    fig_metrics, fig_rpe_p95, fig_nis = write_compare_plots(args.results_root, by_mode)

    print("[w8_summary] wrote: %s" % metrics_csv)
    print("[w8_summary] wrote: %s" % summary_md)
    if fig_metrics:
        print("[w8_summary] wrote: %s" % fig_metrics)
    if fig_rpe_p95:
        print("[w8_summary] wrote: %s" % fig_rpe_p95)
    if fig_nis:
        print("[w8_summary] wrote: %s" % fig_nis)


if __name__ == "__main__":
    main()
