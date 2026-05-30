#!/usr/bin/env python3

"""
Summarize W9-W10 experiments.
Outputs:
- w10_metrics.csv
- w10_summary.md
- w10_focus_metrics.csv
"""

import argparse
import csv
import math
import os
import statistics
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


def safe_float(v):
    try:
        return float(v)
    except Exception:
        return float("nan")


def safe_int(v):
    try:
        return int(float(v))
    except Exception:
        return 0


def finite(v):
    try:
        return math.isfinite(float(v))
    except Exception:
        return False


def safe_median(values):
    vals = [float(v) for v in values if finite(v)]
    if not vals:
        return float("nan")
    return float(statistics.median(vals))


def safe_rate(numer, denom):
    if denom <= 0:
        return float("nan")
    return float(numer) / float(denom)


def read_diag_stats(diag_file):
    stats = {
        "rows": 0,
        "nis_valid_count": 0,
        "nis_exceed_count": 0,
        "committed_count": 0,
        "alpha_nis_sat_count": 0,
        "alpha_nis_total_count": 0,
        "quality_reject_count": 0,
        "use_radar_update_count": 0,
    }
    if not os.path.isfile(diag_file):
        return stats

    with open(diag_file, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            stats["rows"] += 1
            if row.get("nis_valid", "").strip() == "1":
                stats["nis_valid_count"] += 1
                if row.get("nis_exceed_95", "").strip() == "1":
                    stats["nis_exceed_count"] += 1
            if row.get("radar_update_committed", "").strip() == "1":
                stats["committed_count"] += 1
            if row.get("use_radar_update", "").strip() == "1":
                stats["use_radar_update_count"] += 1
            if row.get("quality_gate_pass", "").strip() == "0":
                stats["quality_reject_count"] += 1
            alpha_new = row.get("alpha_nis_new", "")
            if finite(alpha_new):
                stats["alpha_nis_total_count"] += 1
                if row.get("alpha_nis_sat", "").strip() == "1":
                    stats["alpha_nis_sat_count"] += 1
    return stats


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


def summarize_group(rows):
    out = {
        "n_runs": len(rows),
        "ate_median": safe_median([r["ate_rmse"] for r in rows]),
        "rpe_median": safe_median([r["rpe_rmse"] for r in rows]),
        "rpe_p95_median": safe_median([r["rpe_p95"] for r in rows]),
        "runtime_median_s": safe_median([r["runtime_s"] for r in rows]),
    }
    nis_valid_total = sum(r["nis_valid_count"] for r in rows)
    nis_exceed_total = sum(r["nis_exceed_count"] for r in rows)
    committed_total = sum(r["radar_update_committed_count"] for r in rows)
    alpha_sat_total = sum(r["alpha_nis_sat_count"] for r in rows)
    alpha_total = sum(r["alpha_nis_total_count"] for r in rows)
    starved_total = sum(r["radar_starved_count"] for r in rows)
    quality_reject_total = sum(r["quality_reject_count"] for r in rows)

    out["nis_valid_total"] = nis_valid_total
    out["nis_exceed_total"] = nis_exceed_total
    out["nis_exceed_rate"] = safe_rate(nis_exceed_total, nis_valid_total)
    out["radar_update_committed_total"] = committed_total
    out["alpha_nis_sat_total"] = alpha_sat_total
    out["alpha_nis_total"] = alpha_total
    out["alpha_nis_sat_rate"] = safe_rate(alpha_sat_total, alpha_total)
    out["radar_starved_total"] = starved_total
    out["quality_reject_total"] = quality_reject_total
    return out


def main():
    parser = argparse.ArgumentParser(description="Summarize W10 experiments")
    parser.add_argument("--results_root", required=True)
    parser.add_argument("--manifest", default="")
    parser.add_argument("--output_csv", default="")
    parser.add_argument("--output_summary", default="")
    parser.add_argument("--output_focus_csv", default="")
    parser.add_argument("--stage_filter", default="")
    parser.add_argument("--focus_datasets", default="mocap_dark,mocap_dark_fast,indoor_floor,outdoor_street")
    args = parser.parse_args()

    manifest = args.manifest if args.manifest else os.path.join(args.results_root, "run_manifest.csv")
    output_csv = args.output_csv if args.output_csv else os.path.join(args.results_root, "w10_metrics.csv")
    output_summary = args.output_summary if args.output_summary else os.path.join(args.results_root, "w10_summary.md")
    output_focus_csv = args.output_focus_csv if args.output_focus_csv else os.path.join(args.results_root, "w10_focus_metrics.csv")

    focus_datasets = {x.strip() for x in args.focus_datasets.split(",") if x.strip()}

    with open(manifest, "r", encoding="utf-8") as f:
        manifest_rows = list(csv.DictReader(f))

    metric_rows = []
    for row in manifest_rows:
        if row.get("status", "") != "SUCCESS":
            continue
        if args.stage_filter and row.get("stage", "") != args.stage_filter:
            continue

        run_id = row.get("run_id", "")
        run_dir = row.get("export_directory", "")
        gt_path = os.path.join(run_dir, "stamped_groundtruth.txt")
        est_path = get_estimate_file(run_dir, run_id)

        ate_rmse, rpe_rmse, rpe_p95, n_assoc = (float("nan"), float("nan"), float("nan"), 0)
        if os.path.isfile(gt_path) and os.path.isfile(est_path):
            try:
                ate_rmse, rpe_rmse, rpe_p95, n_assoc = compute_metrics(gt_path, est_path)
            except Exception:
                pass

        diag_file = row.get("diag_file", "")
        sched_diag_file = row.get("sched_diag_file", "")
        diag_stats = read_diag_stats(diag_file)
        starved_count = read_sched_starved_count(sched_diag_file)

        metric_rows.append(
            {
                "run_id": run_id,
                "dataset": row.get("dataset", ""),
                "modality": row.get("modality", ""),
                "cov_mode": row.get("cov_mode", ""),
                "scheduler_mode": row.get("scheduler_mode", ""),
                "stage": row.get("stage", ""),
                "runtime_s": safe_float(row.get("runtime_s", "nan")),
                "ate_rmse": ate_rmse,
                "rpe_rmse": rpe_rmse,
                "rpe_p95": rpe_p95,
                "assoc_count": n_assoc,
                "diag_rows": diag_stats["rows"],
                "nis_valid_count": diag_stats["nis_valid_count"],
                "nis_exceed_count": diag_stats["nis_exceed_count"],
                "radar_update_committed_count": diag_stats["committed_count"],
                "alpha_nis_sat_count": diag_stats["alpha_nis_sat_count"],
                "alpha_nis_total_count": diag_stats["alpha_nis_total_count"],
                "quality_reject_count": diag_stats["quality_reject_count"],
                "use_radar_update_count": diag_stats["use_radar_update_count"],
                "radar_starved_count": starved_count,
                "diag_file": diag_file,
                "sched_diag_file": sched_diag_file,
            }
        )

    fieldnames = [
        "run_id",
        "dataset",
        "modality",
        "cov_mode",
        "scheduler_mode",
        "stage",
        "runtime_s",
        "ate_rmse",
        "rpe_rmse",
        "rpe_p95",
        "assoc_count",
        "diag_rows",
        "nis_valid_count",
        "nis_exceed_count",
        "radar_update_committed_count",
        "alpha_nis_sat_count",
        "alpha_nis_total_count",
        "quality_reject_count",
        "use_radar_update_count",
        "radar_starved_count",
        "diag_file",
        "sched_diag_file",
    ]
    with open(output_csv, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in metric_rows:
            writer.writerow(r)

    by_mode = defaultdict(list)
    for r in metric_rows:
        by_mode[r["cov_mode"]].append(r)

    focus_by_mode = defaultdict(list)
    for r in metric_rows:
        if r["dataset"] in focus_datasets:
            focus_by_mode[r["cov_mode"]].append(r)

    focus_fieldnames = [
        "cov_mode",
        "n_runs_focus",
        "ate_median_focus",
        "rpe_median_focus",
        "rpe_p95_median_focus",
        "runtime_median_focus_s",
        "nis_exceed_rate_focus",
        "alpha_nis_sat_rate_focus",
        "committed_total_focus",
        "starved_total_focus",
        "quality_reject_total_focus",
    ]
    focus_rows = []
    for mode in sorted(by_mode.keys()):
        summary_focus = summarize_group(focus_by_mode.get(mode, [])) if focus_by_mode.get(mode, []) else None
        if summary_focus is None:
            focus_rows.append(
                {
                    "cov_mode": mode,
                    "n_runs_focus": 0,
                    "ate_median_focus": float("nan"),
                    "rpe_median_focus": float("nan"),
                    "rpe_p95_median_focus": float("nan"),
                    "runtime_median_focus_s": float("nan"),
                    "nis_exceed_rate_focus": float("nan"),
                    "alpha_nis_sat_rate_focus": float("nan"),
                    "committed_total_focus": 0,
                    "starved_total_focus": 0,
                    "quality_reject_total_focus": 0,
                }
            )
            continue

        focus_rows.append(
            {
                "cov_mode": mode,
                "n_runs_focus": summary_focus["n_runs"],
                "ate_median_focus": summary_focus["ate_median"],
                "rpe_median_focus": summary_focus["rpe_median"],
                "rpe_p95_median_focus": summary_focus["rpe_p95_median"],
                "runtime_median_focus_s": summary_focus["runtime_median_s"],
                "nis_exceed_rate_focus": summary_focus["nis_exceed_rate"],
                "alpha_nis_sat_rate_focus": summary_focus["alpha_nis_sat_rate"],
                "committed_total_focus": summary_focus["radar_update_committed_total"],
                "starved_total_focus": summary_focus["radar_starved_total"],
                "quality_reject_total_focus": summary_focus["quality_reject_total"],
            }
        )

    with open(output_focus_csv, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=focus_fieldnames)
        writer.writeheader()
        for r in focus_rows:
            writer.writerow(r)

    with open(output_summary, "w", encoding="utf-8") as f:
        f.write("# W10 Summary\n\n")
        f.write("## Overall by cov_mode\n\n")
        f.write("| cov_mode | n_runs | ate_median | rpe_median | rpe_p95_median | runtime_median_s | nis_exceed_rate | alpha_nis_sat_rate | committed_total | starved_total | quality_reject_total |\n")
        f.write("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n")
        for mode in sorted(by_mode.keys()):
            summary = summarize_group(by_mode[mode])
            f.write(
                "| {mode} | {n_runs} | {ate:.6f} | {rpe:.6f} | {rpe95:.6f} | {rt:.6f} | {nis:.6f} | {sat:.6f} | {committed} | {starved} | {reject} |\n".format(
                    mode=mode,
                    n_runs=summary["n_runs"],
                    ate=summary["ate_median"],
                    rpe=summary["rpe_median"],
                    rpe95=summary["rpe_p95_median"],
                    rt=summary["runtime_median_s"],
                    nis=summary["nis_exceed_rate"],
                    sat=summary["alpha_nis_sat_rate"],
                    committed=summary["radar_update_committed_total"],
                    starved=summary["radar_starved_total"],
                    reject=summary["quality_reject_total"],
                )
            )

        f.write("\n## Focus by cov_mode\n\n")
        f.write("Focus datasets: %s\n\n" % ",".join(sorted(focus_datasets)))
        f.write("| cov_mode | n_runs_focus | ate_median_focus | rpe_median_focus | rpe_p95_median_focus | runtime_median_focus_s | nis_exceed_rate_focus | alpha_nis_sat_rate_focus | committed_total_focus | starved_total_focus | quality_reject_total_focus |\n")
        f.write("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n")
        for r in focus_rows:
            f.write(
                "| {cov_mode} | {n_runs_focus} | {ate_median_focus:.6f} | {rpe_median_focus:.6f} | {rpe_p95_median_focus:.6f} | {runtime_median_focus_s:.6f} | {nis_exceed_rate_focus:.6f} | {alpha_nis_sat_rate_focus:.6f} | {committed_total_focus} | {starved_total_focus} | {quality_reject_total_focus} |\n".format(
                    **r
                )
            )

    print("[summarize_w10_results] wrote:")
    print("  - %s" % output_csv)
    print("  - %s" % output_summary)
    print("  - %s" % output_focus_csv)


if __name__ == "__main__":
    main()
