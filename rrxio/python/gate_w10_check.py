#!/usr/bin/env python3

"""
Gate checker for W9-W10 Contribution-3.
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


def rel_change(new_v, old_v):
    if (not finite(new_v)) or (not finite(old_v)) or abs(old_v) < 1.0e-12:
        return float("nan")
    return (float(new_v) - float(old_v)) / abs(float(old_v))


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


def load_csv_rows(path):
    with open(path, "r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def parse_name_set(raw):
    return {item.strip() for item in str(raw).split(",") if item.strip()}


def parse_modalities(raw):
    modalities = {item.strip().lower() for item in str(raw).split(",") if item.strip()}
    if not modalities:
        raise RuntimeError("No modality provided.")
    unsupported = modalities.difference({"visual", "thermal"})
    if unsupported:
        raise RuntimeError("Unsupported modalities: %s" % ",".join(sorted(unsupported)))
    return modalities


def filter_modalities(rows, modalities):
    return [r for r in rows if text(r.get("modality", "")).lower() in modalities]


def find_disallowed_modalities(rows, modalities):
    return sorted(
        {text(r.get("modality", "")).lower() for r in rows if text(r.get("modality", "")).lower() not in modalities}
    )


def check_diag_file(path, require_full_columns):
    info = {
        "path": path,
        "rows": 0,
        "missing_columns": [],
        "alpha_nis_total": 0,
        "alpha_nis_sat": 0,
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
    if require_full_columns:
        required_cols.extend(
            [
                "d_v",
                "q_r",
                "q_v",
                "zeta_rv",
                "alpha_nis_prev",
                "alpha_nis_new",
                "alpha_nis_sat",
                "quality_gate_pass",
                "quality_gate_reason",
                "quality_soft_scale",
                "quality_hard_reject",
                "quality_gate_stage",
                "radar_update_reject_reason",
                "visual_feature_valid_count",
                "visual_stale_s",
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
            if require_full_columns and finite(row.get("alpha_nis_new", "nan")):
                info["alpha_nis_total"] += 1
                if text(row.get("alpha_nis_sat", "")) == "1":
                    info["alpha_nis_sat"] += 1

    if info["rows"] == 0:
        append_fail(failures, "diag file has no rows: %s" % path)
    return info, failures


def read_sched_starved_count(path):
    if not os.path.isfile(path):
        return 0, ["sched diag file missing: %s" % path]
    cnt = 0
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if "radar_starved" not in (reader.fieldnames or []):
            return 0, ["sched diag missing radar_starved column: %s" % path]
        for row in reader:
            if text(row.get("radar_starved", "")) == "1":
                cnt += 1
    return cnt, []


def summarize_mode(rows):
    ate = median([r.get("ate_rmse", "nan") for r in rows])
    rpe = median([r.get("rpe_rmse", "nan") for r in rows])
    rpe_p95 = median([r.get("rpe_p95", "nan") for r in rows])
    runtime = median([r.get("runtime_s", "nan") for r in rows])
    valid = sum(i64(r.get("nis_valid_count", "0")) for r in rows)
    exceed = sum(i64(r.get("nis_exceed_count", "0")) for r in rows)
    committed = sum(i64(r.get("radar_update_committed_count", "0")) for r in rows)
    alpha_sat = sum(i64(r.get("alpha_nis_sat_count", "0")) for r in rows)
    alpha_total = sum(i64(r.get("alpha_nis_total_count", "0")) for r in rows)
    quality_reject = sum(i64(r.get("quality_reject_count", "0")) for r in rows)
    hard_reject = sum(i64(r.get("hard_reject_count", "0")) for r in rows)
    diag_rows = sum(i64(r.get("diag_rows", "0")) for r in rows)
    max_consecutive_quality_reject = 0
    for r in rows:
        max_consecutive_quality_reject = max(
            max_consecutive_quality_reject, i64(r.get("quality_reject_max_consecutive", "0"))
        )
    return {
        "n": len(rows),
        "ate": ate,
        "rpe": rpe,
        "rpe_p95": rpe_p95,
        "runtime": runtime,
        "nis_valid": valid,
        "nis_exceed": exceed,
        "nis_rate": safe_rate(exceed, valid),
        "committed": committed,
        "alpha_sat": alpha_sat,
        "alpha_total": alpha_total,
        "alpha_sat_rate": safe_rate(alpha_sat, alpha_total),
        "quality_reject_total": quality_reject,
        "hard_reject_total": hard_reject,
        "diag_rows_total": diag_rows,
        "quality_reject_rate": safe_rate(quality_reject, diag_rows),
        "hard_reject_rate": safe_rate(hard_reject, diag_rows),
        "max_consecutive_quality_reject": max_consecutive_quality_reject,
    }


def main():
    parser = argparse.ArgumentParser(description="Gate checker for W10 contribution")
    parser.add_argument("--results_root", required=True)
    parser.add_argument("--manifest", default="")
    parser.add_argument("--metrics", default="")
    parser.add_argument("--base_manifest", default="", help="Optional original-baseline manifest for traceability checks")
    parser.add_argument("--base_metrics", default="", help="Optional original-baseline metrics CSV used as comparison base")
    parser.add_argument("--base_label", default="original", help="Label used in report when --base_metrics is provided")
    parser.add_argument("--report_json", default="")
    parser.add_argument("--stage_filter", default="")
    parser.add_argument("--mode_base", default="alpha_r_sk")
    parser.add_argument("--mode_full", default="alpha_r_sk_nis_rv")
    parser.add_argument(
        "--focus_datasets",
        default="mocap_easy,mocap_medium,mocap_difficult,mocap_dark,mocap_dark_fast,gym,indoor_floor,outdoor_campus,outdoor_street",
    )
    parser.add_argument("--modalities", default="visual", help="Comma-separated modalities to gate: visual,thermal")
    parser.add_argument("--nis_gate_mode", default="band", choices=["band", "relative_drop"])
    parser.add_argument("--nis_exceed_rate_min", type=float, default=0.01)
    parser.add_argument("--nis_exceed_rate_max", type=float, default=0.08)
    parser.add_argument("--nis_exceed_rate_target", type=float, default=0.05)
    parser.add_argument("--nis_group_gate_enable", type=int, default=1)
    parser.add_argument("--min_rpe_p95_improve_focus", type=float, default=0.10)
    parser.add_argument("--min_nis_relative_drop_focus", type=float, default=0.15)
    parser.add_argument("--max_alpha_nis_sat_rate_focus", type=float, default=0.10)
    parser.add_argument("--max_ate_degrade", type=float, default=0.05)
    parser.add_argument("--max_rpe_degrade", type=float, default=0.05)
    parser.add_argument("--max_runtime_increase", type=float, default=0.15)
    parser.add_argument("--max_nis_abs_increase_pp", type=float, default=1.0)
    parser.add_argument("--max_committed_drop", type=float, default=0.05)
    parser.add_argument("--repeat_min_runs", type=int, default=3)
    parser.add_argument("--require_focus_datasets", type=int, default=1)
    parser.add_argument("--max_repeat_ate_cv", type=float, default=0.10)
    parser.add_argument("--max_repeat_rpe_cv", type=float, default=0.10)
    parser.add_argument("--max_repeat_update_rel_range", type=float, default=0.05)
    parser.add_argument("--max_repeat_reject_rel_range", type=float, default=0.10)
    parser.add_argument("--health_gate_enable", type=int, default=1)
    parser.add_argument("--strict_gate_enable", type=int, default=1)
    parser.add_argument("--max_quality_reject_rate_focus", type=float, default=0.08)
    parser.add_argument("--max_hard_reject_rate_focus", type=float, default=0.03)
    parser.add_argument("--max_consecutive_quality_reject_focus", type=int, default=50)
    args = parser.parse_args()

    manifest = args.manifest if args.manifest else os.path.join(args.results_root, "run_manifest.csv")
    metrics = args.metrics if args.metrics else os.path.join(args.results_root, "w10_metrics.csv")
    report_json = args.report_json if args.report_json else os.path.join(args.results_root, "gate_w10_report.json")

    report = {
        "gate": "W10",
        "results_root": args.results_root,
        "manifest": manifest,
        "metrics": metrics,
        "base_manifest": args.base_manifest,
        "base_metrics": args.base_metrics,
        "criteria": {
            "stage_filter": args.stage_filter,
            "mode_base": args.mode_base,
            "mode_full": args.mode_full,
            "base_label": args.base_label,
            "focus_datasets": args.focus_datasets,
            "modalities": args.modalities,
            "nis_gate_mode": args.nis_gate_mode,
            "nis_exceed_rate_min": args.nis_exceed_rate_min,
            "nis_exceed_rate_max": args.nis_exceed_rate_max,
            "nis_exceed_rate_target": args.nis_exceed_rate_target,
            "nis_group_gate_enable": int(args.nis_group_gate_enable) != 0,
            "min_rpe_p95_improve_focus": args.min_rpe_p95_improve_focus,
            "min_nis_relative_drop_focus": args.min_nis_relative_drop_focus,
            "max_alpha_nis_sat_rate_focus": args.max_alpha_nis_sat_rate_focus,
            "max_ate_degrade": args.max_ate_degrade,
            "max_rpe_degrade": args.max_rpe_degrade,
            "max_runtime_increase": args.max_runtime_increase,
            "max_nis_abs_increase_pp": args.max_nis_abs_increase_pp,
            "max_committed_drop": args.max_committed_drop,
            "repeat_min_runs": args.repeat_min_runs,
            "require_focus_datasets": int(args.require_focus_datasets) != 0,
            "max_repeat_ate_cv": args.max_repeat_ate_cv,
            "max_repeat_rpe_cv": args.max_repeat_rpe_cv,
            "max_repeat_update_rel_range": args.max_repeat_update_rel_range,
            "max_repeat_reject_rel_range": args.max_repeat_reject_rel_range,
            "health_gate_enable": int(args.health_gate_enable) != 0,
            "strict_gate_enable": int(args.strict_gate_enable) != 0,
            "max_quality_reject_rate_focus": args.max_quality_reject_rate_focus,
            "max_hard_reject_rate_focus": args.max_hard_reject_rate_focus,
            "max_consecutive_quality_reject_focus": args.max_consecutive_quality_reject_focus,
        },
        "checks": {},
        "health_failures": [],
        "strict_failures": [],
        "health_pass": False,
        "strict_pass": False,
        "failures": [],
        "pass": False,
    }

    failures = []
    health_gate_enable = int(args.health_gate_enable) != 0
    strict_gate_enable = int(args.strict_gate_enable) != 0
    modalities = parse_modalities(args.modalities)

    if not os.path.isfile(manifest):
        append_fail(failures, "manifest not found: %s" % manifest)
        manifest_rows = []
    else:
        manifest_rows = load_csv_rows(manifest)

    if not os.path.isfile(metrics):
        append_fail(failures, "metrics not found: %s" % metrics)
        metric_rows = []
    else:
        metric_rows = load_csv_rows(metrics)

    if args.base_metrics:
        if not os.path.isfile(args.base_metrics):
            append_fail(failures, "base metrics not found: %s" % args.base_metrics)
            base_metric_rows_external = []
        else:
            base_metric_rows_external = load_csv_rows(args.base_metrics)
    else:
        base_metric_rows_external = []

    if args.base_manifest:
        if not os.path.isfile(args.base_manifest):
            append_fail(failures, "base manifest not found: %s" % args.base_manifest)
            base_manifest_rows_external = []
        else:
            base_manifest_rows_external = load_csv_rows(args.base_manifest)
    else:
        base_manifest_rows_external = []

    if args.stage_filter:
        manifest_rows = [r for r in manifest_rows if text(r.get("stage", "")) == args.stage_filter]
        metric_rows = [r for r in metric_rows if text(r.get("stage", "")) == args.stage_filter]

    manifest_bad_modalities = find_disallowed_modalities(manifest_rows, modalities)
    metric_bad_modalities = find_disallowed_modalities(metric_rows, modalities)
    base_metric_bad_modalities = find_disallowed_modalities(base_metric_rows_external, modalities)
    base_manifest_bad_modalities = find_disallowed_modalities(base_manifest_rows_external, modalities)
    if manifest_bad_modalities:
        append_fail(
            failures,
            "manifest contains modalities outside %s: %s"
            % (",".join(sorted(modalities)), ",".join(manifest_bad_modalities)),
        )
    if metric_bad_modalities:
        append_fail(
            failures,
            "metrics contains modalities outside %s: %s"
            % (",".join(sorted(modalities)), ",".join(metric_bad_modalities)),
        )
    if base_metric_bad_modalities:
        append_fail(
            failures,
            "base metrics contains modalities outside %s: %s"
            % (",".join(sorted(modalities)), ",".join(base_metric_bad_modalities)),
        )
    if base_manifest_bad_modalities:
        append_fail(
            failures,
            "base manifest contains modalities outside %s: %s"
            % (",".join(sorted(modalities)), ",".join(base_manifest_bad_modalities)),
        )
    manifest_rows = filter_modalities(manifest_rows, modalities)
    metric_rows = filter_modalities(metric_rows, modalities)
    base_metric_rows_external = filter_modalities(base_metric_rows_external, modalities)
    base_manifest_rows_external = filter_modalities(base_manifest_rows_external, modalities)
    report["checks"]["modalities"] = {
        "selected": sorted(modalities),
        "manifest_disallowed": manifest_bad_modalities,
        "metrics_disallowed": metric_bad_modalities,
        "base_manifest_disallowed": base_manifest_bad_modalities,
        "base_metrics_disallowed": base_metric_bad_modalities,
    }

    success_manifest = [r for r in manifest_rows if r.get("status", "") == "SUCCESS"]
    by_mode_manifest = defaultdict(list)
    for row in success_manifest:
        by_mode_manifest[text(row.get("cov_mode", "")).lower()].append(row)

    mode_base = args.mode_base.strip().lower()
    mode_full = args.mode_full.strip().lower()

    using_external_base = bool(args.base_metrics)
    if (not using_external_base) and len(by_mode_manifest.get(mode_base, [])) == 0:
        append_fail(failures, "no SUCCESS run for cov_mode=%s" % mode_base)
    if len(by_mode_manifest.get(mode_full, [])) == 0:
        append_fail(failures, "no SUCCESS run for cov_mode=%s" % mode_full)

    by_mode_metrics = defaultdict(list)
    for row in metric_rows:
        by_mode_metrics[text(row.get("cov_mode", "")).lower()].append(row)

    base_rows = base_metric_rows_external if using_external_base else by_mode_metrics.get(mode_base, [])
    full_rows = by_mode_metrics.get(mode_full, [])
    if not base_rows:
        if using_external_base:
            append_fail(failures, "no metric rows in base metrics: %s" % args.base_metrics)
        else:
            append_fail(failures, "no metric rows for cov_mode=%s" % mode_base)
    if not full_rows:
        append_fail(failures, "no metric rows for cov_mode=%s" % mode_full)

    agg_base = summarize_mode(base_rows)
    agg_full = summarize_mode(full_rows)
    ate_degrade = rel_change(agg_full["ate"], agg_base["ate"])
    rpe_degrade = rel_change(agg_full["rpe"], agg_base["rpe"])
    runtime_increase = rel_change(agg_full["runtime"], agg_base["runtime"])
    nis_abs_inc_pp = (agg_full["nis_rate"] - agg_base["nis_rate"]) * 100.0 if finite(agg_base["nis_rate"]) and finite(agg_full["nis_rate"]) else float("nan")
    committed_drop = 1.0 - safe_rate(agg_full["committed"], agg_base["committed"]) if agg_base["committed"] > 0 else float("nan")

    report["checks"]["aggregate"] = {
        "base": agg_base,
        "full": agg_full,
        "ate_degrade": ate_degrade,
        "rpe_degrade": rpe_degrade,
        "runtime_increase": runtime_increase,
        "nis_abs_inc_pp": nis_abs_inc_pp,
        "committed_drop": committed_drop,
        "base_source": args.base_label if using_external_base else mode_base,
    }

    if strict_gate_enable:
        if (not finite(ate_degrade)) or ate_degrade > args.max_ate_degrade:
            append_fail(failures, "ATE degrade failed: got=%s need <= %.6f" % (str(ate_degrade), args.max_ate_degrade))
        if (not finite(rpe_degrade)) or rpe_degrade > args.max_rpe_degrade:
            append_fail(failures, "RPE degrade failed: got=%s need <= %.6f" % (str(rpe_degrade), args.max_rpe_degrade))
        if (not finite(runtime_increase)) or runtime_increase > args.max_runtime_increase:
            append_fail(failures, "runtime increase failed: got=%s need <= %.6f" % (str(runtime_increase), args.max_runtime_increase))
        if args.nis_gate_mode == "relative_drop":
            if (not finite(nis_abs_inc_pp)) or nis_abs_inc_pp > args.max_nis_abs_increase_pp:
                append_fail(failures, "NIS absolute increase failed: got=%s need <= %.6f pp" % (str(nis_abs_inc_pp), args.max_nis_abs_increase_pp))
        if (not finite(committed_drop)) or committed_drop > args.max_committed_drop:
            append_fail(failures, "committed drop failed: got=%s need <= %.6f" % (str(committed_drop), args.max_committed_drop))

    focus_set = parse_name_set(args.focus_datasets)
    base_focus = [r for r in base_rows if text(r.get("dataset", "")) in focus_set]
    full_focus = [r for r in full_rows if text(r.get("dataset", "")) in focus_set]
    if strict_gate_enable and int(args.require_focus_datasets) != 0:
        for dataset in sorted(focus_set):
            base_count = sum(1 for r in base_rows if text(r.get("dataset", "")) == dataset)
            full_count = sum(1 for r in full_rows if text(r.get("dataset", "")) == dataset)
            if base_count < args.repeat_min_runs:
                append_fail(
                    failures,
                    "focus dataset base runs too few for %s: %d < %d"
                    % (dataset, base_count, args.repeat_min_runs),
                )
            if full_count < args.repeat_min_runs:
                append_fail(
                    failures,
                    "focus dataset full runs too few for %s: %d < %d"
                    % (dataset, full_count, args.repeat_min_runs),
                )
    focus_base_agg = summarize_mode(base_focus)
    focus_full_agg = summarize_mode(full_focus)

    rpe_p95_improve_focus = safe_rate((focus_base_agg["rpe_p95"] - focus_full_agg["rpe_p95"]), focus_base_agg["rpe_p95"])
    nis_relative_drop_focus = safe_rate((focus_base_agg["nis_rate"] - focus_full_agg["nis_rate"]), focus_base_agg["nis_rate"])
    nis_consistency_error_focus = (
        abs(focus_full_agg["nis_rate"] - args.nis_exceed_rate_target)
        if finite(focus_full_agg["nis_rate"])
        else float("nan")
    )
    alpha_nis_sat_rate_focus = focus_full_agg["alpha_sat_rate"]

    report["checks"]["focus"] = {
        "base": focus_base_agg,
        "full": focus_full_agg,
        "rpe_p95_improve_focus": rpe_p95_improve_focus,
        "nis_relative_drop_focus": nis_relative_drop_focus,
        "nis_consistency_error_focus": nis_consistency_error_focus,
        "nis_gate_mode": args.nis_gate_mode,
        "alpha_nis_sat_rate_focus": alpha_nis_sat_rate_focus,
        "quality_reject_rate_focus": focus_full_agg["quality_reject_rate"],
        "hard_reject_rate_focus": focus_full_agg["hard_reject_rate"],
        "max_consecutive_quality_reject_focus": focus_full_agg["max_consecutive_quality_reject"],
    }

    nis_group_checks = []
    if args.nis_gate_mode == "band":
        grouped_full_focus = defaultdict(list)
        for row in full_focus:
            key = "%s/%s" % (text(row.get("dataset", "")), text(row.get("modality", "")))
            grouped_full_focus[key].append(row)
        for key in sorted(grouped_full_focus.keys()):
            agg = summarize_mode(grouped_full_focus[key])
            nis_rate = agg["nis_rate"]
            group_pass = finite(nis_rate) and args.nis_exceed_rate_min <= nis_rate <= args.nis_exceed_rate_max
            nis_group_checks.append(
                {
                    "group": key,
                    "nis_rate": nis_rate,
                    "target_error": abs(nis_rate - args.nis_exceed_rate_target) if finite(nis_rate) else float("nan"),
                    "pass": group_pass,
                }
            )
            if strict_gate_enable and int(args.nis_group_gate_enable) != 0 and not group_pass:
                append_fail(
                    failures,
                    "focus NIS group band failed for %s: got=%s need in [%.6f, %.6f]"
                    % (key, str(nis_rate), args.nis_exceed_rate_min, args.nis_exceed_rate_max),
                )
    report["checks"]["nis_by_group"] = nis_group_checks

    health_failures = []
    if health_gate_enable:
        if (not finite(committed_drop)) or committed_drop > args.max_committed_drop:
            append_fail(
                health_failures,
                "health committed drop failed: got=%s need <= %.6f" % (str(committed_drop), args.max_committed_drop),
            )
        if (not finite(focus_full_agg["quality_reject_rate"])) or (
            focus_full_agg["quality_reject_rate"] > args.max_quality_reject_rate_focus
        ):
            append_fail(
                health_failures,
                "health quality reject rate failed: got=%s need <= %.6f"
                % (str(focus_full_agg["quality_reject_rate"]), args.max_quality_reject_rate_focus),
            )
        if (not finite(focus_full_agg["hard_reject_rate"])) or (
            focus_full_agg["hard_reject_rate"] > args.max_hard_reject_rate_focus
        ):
            append_fail(
                health_failures,
                "health hard reject rate failed: got=%s need <= %.6f"
                % (str(focus_full_agg["hard_reject_rate"]), args.max_hard_reject_rate_focus),
            )
        if focus_full_agg["max_consecutive_quality_reject"] > args.max_consecutive_quality_reject_focus:
            append_fail(
                health_failures,
                "health max consecutive quality reject failed: got=%d need <= %d"
                % (
                    focus_full_agg["max_consecutive_quality_reject"],
                    args.max_consecutive_quality_reject_focus,
                ),
            )

    if strict_gate_enable:
        if (not finite(rpe_p95_improve_focus)) or rpe_p95_improve_focus < args.min_rpe_p95_improve_focus:
            append_fail(
                failures,
                "focus rpe_p95 improve failed: got=%s need >= %.6f"
                % (str(rpe_p95_improve_focus), args.min_rpe_p95_improve_focus),
            )
        if args.nis_gate_mode == "relative_drop":
            if (not finite(nis_relative_drop_focus)) or nis_relative_drop_focus < args.min_nis_relative_drop_focus:
                append_fail(
                    failures,
                    "focus NIS relative drop failed: got=%s need >= %.6f"
                    % (str(nis_relative_drop_focus), args.min_nis_relative_drop_focus),
                )
        else:
            nis_rate = focus_full_agg["nis_rate"]
            if (not finite(nis_rate)) or nis_rate < args.nis_exceed_rate_min or nis_rate > args.nis_exceed_rate_max:
                append_fail(
                    failures,
                    "focus NIS consistency band failed: got=%s need in [%.6f, %.6f] around target %.6f"
                    % (str(nis_rate), args.nis_exceed_rate_min, args.nis_exceed_rate_max, args.nis_exceed_rate_target),
                )
        if (not finite(alpha_nis_sat_rate_focus)) or alpha_nis_sat_rate_focus > args.max_alpha_nis_sat_rate_focus:
            append_fail(
                failures,
                "focus alpha_nis saturation failed: got=%s need <= %.6f"
                % (str(alpha_nis_sat_rate_focus), args.max_alpha_nis_sat_rate_focus),
            )

    diag_checks = []
    total_starved = 0
    for mode, rows in ((mode_base, by_mode_manifest.get(mode_base, [])), (mode_full, by_mode_manifest.get(mode_full, []))):
        require_full = (mode == mode_full)
        for row in rows:
            diag_file = text(row.get("diag_file", ""))
            if diag_file:
                diag_info, local_failures = check_diag_file(diag_file, require_full)
                diag_checks.append(diag_info)
                failures.extend(local_failures)
            sched_file = text(row.get("sched_diag_file", ""))
            if sched_file:
                starved, local_failures = read_sched_starved_count(sched_file)
                total_starved += starved
                failures.extend(local_failures)

    report["checks"]["diag_files"] = diag_checks
    report["checks"]["total_radar_starved"] = total_starved
    if total_starved != 0:
        if health_gate_enable:
            append_fail(health_failures, "health radar_starved must be 0, got=%d" % total_starved)
        if strict_gate_enable:
            append_fail(failures, "radar_starved must be 0, got=%d" % total_starved)

    repeat_groups = defaultdict(list)
    for row in full_rows:
        key = "{}/{}/{}".format(text(row.get("dataset", "")), text(row.get("modality", "")), text(row.get("scheduler_mode", "")))
        repeat_groups[key].append(row)

    repeat_report = []
    for key, group in sorted(repeat_groups.items()):
        if len(group) < args.repeat_min_runs:
            if strict_gate_enable:
                append_fail(failures, "repeatability runs too few for %s: %d < %d" % (key, len(group), args.repeat_min_runs))
            continue

        ate_vals = [f64(r.get("ate_rmse", "nan")) for r in group if finite(r.get("ate_rmse", "nan"))]
        rpe_vals = [f64(r.get("rpe_rmse", "nan")) for r in group if finite(r.get("rpe_rmse", "nan"))]
        upd_vals = [f64(r.get("radar_update_committed_count", "nan")) for r in group if finite(r.get("radar_update_committed_count", "nan"))]
        rej_vals = [f64(r.get("quality_reject_count", "nan")) for r in group if finite(r.get("quality_reject_count", "nan"))]

        ate_cv = cv(ate_vals)
        rpe_cv = cv(rpe_vals)
        upd_rr = rel_range(upd_vals)
        rej_rr = rel_range(rej_vals)

        repeat_report.append(
            {
                "group": key,
                "n": len(group),
                "ate_cv": ate_cv,
                "rpe_cv": rpe_cv,
                "update_rel_range": upd_rr,
                "reject_rel_range": rej_rr,
            }
        )

        if strict_gate_enable and finite(ate_cv) and ate_cv > args.max_repeat_ate_cv:
            append_fail(failures, "repeatability ate cv too high for %s: %.6f > %.6f" % (key, ate_cv, args.max_repeat_ate_cv))
        if strict_gate_enable and finite(rpe_cv) and rpe_cv > args.max_repeat_rpe_cv:
            append_fail(failures, "repeatability rpe cv too high for %s: %.6f > %.6f" % (key, rpe_cv, args.max_repeat_rpe_cv))
        if strict_gate_enable and finite(upd_rr) and upd_rr > args.max_repeat_update_rel_range:
            append_fail(failures, "repeatability update rel-range too high for %s: %.6f > %.6f" % (key, upd_rr, args.max_repeat_update_rel_range))
        if strict_gate_enable and finite(rej_rr) and rej_rr > args.max_repeat_reject_rel_range:
            append_fail(failures, "repeatability reject rel-range too high for %s: %.6f > %.6f" % (key, rej_rr, args.max_repeat_reject_rel_range))

    report["checks"]["repeatability"] = repeat_report

    report["health_failures"] = health_failures
    report["strict_failures"] = failures
    report["health_pass"] = (not health_gate_enable) or len(health_failures) == 0
    report["strict_pass"] = (not strict_gate_enable) or len(failures) == 0
    report["failures"] = failures
    report["pass"] = report["strict_pass"]

    with open(report_json, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    print(json.dumps(report, indent=2, ensure_ascii=False))
    sys.exit(0 if report["pass"] else 1)


if __name__ == "__main__":
    main()
