#!/usr/bin/env python3

"""
Analyze local RPE spikes for W10 visual-only runs and align them with DVC diagnostics.
"""

import argparse
import csv
import math
import os
import statistics
from collections import defaultdict

import numpy as np


DIAG_FIELDS = [
    "nis_vel", "d_r", "q_r", "d_v", "q_v", "alpha_r", "zeta_rv", "alpha_nis_new",
    "quality_soft_scale", "trace_R_used", "minEig_R_used", "cond", "inlier_ratio", "n_targets",
    "zero_velocity_detected",
    "candidate_nis", "pre_update_nis_scale", "nis_band_scale", "nis_band_state", "nis_rolling_rate",
    "nis_rolling_mean", "zero_velocity_rolling_rate", "nis_shrink_allowed",
    "residual_mean", "residual_rmse", "residual_mad", "residual_p95", "snr_mean",
    "noise_db_mean", "doppler_bias_abs", "inlier_azimuth_span", "inlier_elevation_span",
]
SCHED_FIELDS = [
    "radar_enqueue_to_commit_ms", "event_latency_ms", "watermark_wait_ms", "queue_depth",
    "loader_backpressure_wait_ms", "event_queue_wait_ms", "worker_dispatch_ms", "update_publish_ms",
    "diag_write_ms", "radar_starved",
]


def finite(v):
    try:
        return math.isfinite(float(v))
    except Exception:
        return False


def f64(v):
    try:
        return float(v)
    except Exception:
        return float("nan")


def text(v):
    return "" if v is None else str(v).strip()


def percentile(values, q):
    vals = np.array([v for v in values if math.isfinite(float(v))], dtype=np.float64)
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


def safe_median(values):
    vals = [float(v) for v in values if finite(v)]
    return float(statistics.median(vals)) if vals else float("nan")


def load_traj(path):
    data = np.genfromtxt(path, delimiter=" ")
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 4:
        raise RuntimeError("Trajectory file has insufficient columns: %s" % path)
    return data


def associate_indices(gt, est, max_dt):
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
    return pairs


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


def get_estimate_file(run_dir, run_id):
    trial_specific = os.path.join(run_dir, "stamped_traj_estimate_%s.txt" % run_id)
    if os.path.isfile(trial_specific):
        return trial_specific
    return os.path.join(run_dir, "stamped_traj_estimate.txt")


def load_time_rows(path, time_col):
    if not path or not os.path.isfile(path):
        return []
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = f64(row.get(time_col, "nan"))
            if math.isfinite(t):
                rows.append((t, row))
    rows.sort(key=lambda item: item[0])
    return rows


def nearest_row(rows, timestamp, max_dt):
    if not rows:
        return None, float("nan")
    lo, hi = 0, len(rows) - 1
    best = None
    best_dt = float("inf")
    while lo <= hi:
        mid = (lo + hi) // 2
        t = rows[mid][0]
        dt = abs(t - timestamp)
        if dt < best_dt:
            best = rows[mid][1]
            best_dt = dt
        if t < timestamp:
            lo = mid + 1
        else:
            hi = mid - 1
    if best is None or best_dt > max_dt:
        return None, float("nan")
    return best, best_dt


def local_rpe_events(gt_path, est_path, rpe_delta, assoc_max_dt):
    gt = load_traj(gt_path)
    est = load_traj(est_path)
    pairs = associate_indices(gt, est, assoc_max_dt)
    if len(pairs) < max(3, rpe_delta + 1):
        return [], float("nan")

    gi = np.array([p[0] for p in pairs], dtype=np.int64)
    ej = np.array([p[1] for p in pairs], dtype=np.int64)
    gt_p = gt[gi, 1:4]
    est_p = est[ej, 1:4]
    r, t = umeyama_rigid(est_p, gt_p)
    est_aligned = (r @ est_p.T).T + t
    rel = (est_aligned[rpe_delta:] - est_aligned[:-rpe_delta]) - (gt_p[rpe_delta:] - gt_p[:-rpe_delta])
    rpe = np.linalg.norm(rel, axis=1)
    p95 = percentile(rpe, 0.95)
    events = []
    for i, err in enumerate(rpe):
        end_idx = i + rpe_delta
        events.append(
            {
                "event_timestamp": float(gt[gi[end_idx], 0]),
                "window_start_timestamp": float(gt[gi[i], 0]),
                "window_end_timestamp": float(gt[gi[end_idx], 0]),
                "rpe_local": float(err),
                "rpe_run_p95": p95,
            }
        )
    return events, p95


def parse_modalities(raw):
    out = {item.strip().lower() for item in str(raw).split(",") if item.strip()}
    if not out:
        raise RuntimeError("No modality selected.")
    bad = out.difference({"visual", "thermal"})
    if bad:
        raise RuntimeError("Unsupported modalities: %s" % ",".join(sorted(bad)))
    return out


def infer_spike_note(row):
    nis = f64(row.get("nis_vel", "nan"))
    candidate = f64(row.get("candidate_nis", "nan"))
    residual = f64(row.get("residual_p95", "nan"))
    latency = f64(row.get("radar_enqueue_to_commit_ms", "nan"))
    trace_r = f64(row.get("trace_R_used", "nan"))
    notes = []
    if finite(nis):
        if nis > 7.8147279:
            notes.append("high_post_update_nis")
        elif nis < 1.0:
            notes.append("low_post_update_nis")
    if finite(candidate) and candidate > 7.8147279:
        notes.append("high_candidate_nis")
    if finite(residual) and residual > 0.5:
        notes.append("large_reve_residual")
    if finite(latency) and latency > 50.0:
        notes.append("radar_latency_tail")
    if finite(trace_r) and trace_r > 1.0:
        notes.append("large_covariance")
    return ";".join(notes) if notes else "no_single_dominant_signal"


def main():
    parser = argparse.ArgumentParser(description="Analyze W10 local RPE spikes and align diagnostics")
    parser.add_argument("--results_root", required=True)
    parser.add_argument("--manifest", default="")
    parser.add_argument("--cov_mode", default="alpha_r_sk_nis_rv")
    parser.add_argument("--scheduler_mode", default="event_stage2")
    parser.add_argument("--modalities", default="visual")
    parser.add_argument("--top_k_per_run", type=int, default=20)
    parser.add_argument("--rpe_delta", type=int, default=10)
    parser.add_argument("--assoc_max_dt", type=float, default=0.05)
    parser.add_argument("--diag_max_dt", type=float, default=0.08)
    parser.add_argument("--sched_max_dt", type=float, default=0.08)
    parser.add_argument("--output_events", default="")
    parser.add_argument("--output_by_dataset", default="")
    parser.add_argument("--output_summary", default="")
    args = parser.parse_args()

    manifest = args.manifest or os.path.join(args.results_root, "run_manifest.csv")
    output_events = args.output_events or os.path.join(args.results_root, "w10_spike_events.csv")
    output_by_dataset = args.output_by_dataset or os.path.join(args.results_root, "w10_spike_by_dataset.csv")
    output_summary = args.output_summary or os.path.join(args.results_root, "w10_spike_summary.md")
    modalities = parse_modalities(args.modalities)

    with open(manifest, "r", encoding="utf-8") as f:
        manifest_rows = list(csv.DictReader(f))

    spike_rows = []
    for run in manifest_rows:
        if text(run.get("status", "")) != "SUCCESS":
            continue
        if text(run.get("cov_mode", "")) != args.cov_mode:
            continue
        if args.scheduler_mode and text(run.get("scheduler_mode", "")) != args.scheduler_mode:
            continue
        if text(run.get("modality", "")).lower() not in modalities:
            continue

        run_id = text(run.get("run_id", ""))
        run_dir = text(run.get("export_directory", ""))
        gt_path = os.path.join(run_dir, "stamped_groundtruth.txt")
        est_path = get_estimate_file(run_dir, run_id)
        if not os.path.isfile(gt_path) or not os.path.isfile(est_path):
            continue

        events, _ = local_rpe_events(gt_path, est_path, args.rpe_delta, args.assoc_max_dt)
        if not events:
            continue
        events.sort(key=lambda item: item["rpe_local"], reverse=True)
        events = events[: max(1, args.top_k_per_run)]

        diag_rows = load_time_rows(text(run.get("diag_file", "")), "timestamp")
        sched_rows = load_time_rows(text(run.get("sched_diag_file", "")), "event_timestamp")

        for rank, event in enumerate(events, start=1):
            t = event["event_timestamp"]
            diag, diag_dt = nearest_row(diag_rows, t, args.diag_max_dt)
            sched, sched_dt = nearest_row(sched_rows, t, args.sched_max_dt)
            out = {
                "run_id": run_id,
                "dataset": text(run.get("dataset", "")),
                "modality": text(run.get("modality", "")),
                "cov_mode": text(run.get("cov_mode", "")),
                "scheduler_mode": text(run.get("scheduler_mode", "")),
                "trial_note": text(run.get("note", "")),
                "event_timestamp": "%.9f" % t,
                "window_start_timestamp": "%.9f" % event["window_start_timestamp"],
                "window_end_timestamp": "%.9f" % event["window_end_timestamp"],
                "rpe_local": event["rpe_local"],
                "rpe_run_p95": event["rpe_run_p95"],
                "rpe_rank_in_run": rank,
                "diag_delta_s": diag_dt,
                "sched_delta_s": sched_dt,
            }
            for field in DIAG_FIELDS:
                out[field] = diag.get(field, "nan") if diag else "nan"
            for field in SCHED_FIELDS:
                out[field] = sched.get(field, "nan") if sched else "nan"
            if "radar_latency_ms" not in out:
                out["radar_latency_ms"] = out.get("radar_enqueue_to_commit_ms", "nan")
            out["spike_note"] = infer_spike_note(out)
            spike_rows.append(out)

    fieldnames = [
        "run_id", "dataset", "modality", "cov_mode", "scheduler_mode", "trial_note", "event_timestamp",
        "window_start_timestamp", "window_end_timestamp", "rpe_local", "rpe_run_p95", "rpe_rank_in_run",
        "diag_delta_s", "sched_delta_s",
    ] + DIAG_FIELDS + SCHED_FIELDS + ["radar_latency_ms", "spike_note"]
    with open(output_events, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in spike_rows:
            writer.writerow(row)

    grouped = defaultdict(list)
    for row in spike_rows:
        grouped[(row["dataset"], row["modality"])].append(row)

    by_dataset_fields = [
        "dataset", "modality", "spike_rows", "rpe_local_median", "rpe_local_p95", "nis_vel_median",
        "candidate_nis_median", "trace_R_used_median", "residual_p95_median", "radar_latency_ms_median",
        "radar_latency_ms_p95", "dominant_notes",
    ]
    by_dataset_rows = []
    for (dataset, modality), rows in sorted(grouped.items()):
        notes = defaultdict(int)
        for row in rows:
            for note in text(row.get("spike_note", "")).split(";"):
                if note:
                    notes[note] += 1
        dominant = ";".join("%s:%d" % item for item in sorted(notes.items(), key=lambda kv: (-kv[1], kv[0]))[:5])
        by_dataset_rows.append(
            {
                "dataset": dataset,
                "modality": modality,
                "spike_rows": len(rows),
                "rpe_local_median": safe_median([r["rpe_local"] for r in rows]),
                "rpe_local_p95": percentile([f64(r["rpe_local"]) for r in rows], 0.95),
                "nis_vel_median": safe_median([r.get("nis_vel", "nan") for r in rows]),
                "candidate_nis_median": safe_median([r.get("candidate_nis", "nan") for r in rows]),
                "trace_R_used_median": safe_median([r.get("trace_R_used", "nan") for r in rows]),
                "residual_p95_median": safe_median([r.get("residual_p95", "nan") for r in rows]),
                "radar_latency_ms_median": safe_median([r.get("radar_latency_ms", "nan") for r in rows]),
                "radar_latency_ms_p95": percentile([f64(r.get("radar_latency_ms", "nan")) for r in rows], 0.95),
                "dominant_notes": dominant,
            }
        )

    with open(output_by_dataset, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=by_dataset_fields)
        writer.writeheader()
        for row in by_dataset_rows:
            writer.writerow(row)

    with open(output_summary, "w", encoding="utf-8") as f:
        f.write("# W10 Spike Analysis\n\n")
        f.write("results_root: `%s`\n\n" % args.results_root)
        f.write("cov_mode: `%s`, scheduler_mode: `%s`, modalities: `%s`\n\n" % (args.cov_mode, args.scheduler_mode, args.modalities))
        f.write("events_csv: `%s`\n\n" % output_events)
        f.write("by_dataset_csv: `%s`\n\n" % output_by_dataset)
        f.write("| dataset | modality | spike_rows | rpe_local_median | rpe_local_p95 | nis_median | candidate_nis_median | residual_p95_median | radar_latency_p95_ms | dominant_notes |\n")
        f.write("|---|---:|---:|---:|---:|---:|---:|---:|---:|---|\n")
        for row in by_dataset_rows:
            f.write(
                "| {dataset} | {modality} | {spike_rows} | {rpe_local_median:.6f} | {rpe_local_p95:.6f} | {nis_vel_median:.6f} | {candidate_nis_median:.6f} | {residual_p95_median:.6f} | {radar_latency_ms_p95:.6f} | {dominant_notes} |\n".format(
                    **row
                )
            )

    print("wrote %s spike rows" % len(spike_rows))
    print(output_events)
    print(output_by_dataset)
    print(output_summary)


if __name__ == "__main__":
    main()
