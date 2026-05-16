#!/usr/bin/env python3

"""
Gate-W4 checker for minimal diagnostic closure.
Pass criteria:
1) dvc_diag schema required columns are present
2) required columns are not empty row-wise
3) record count is consistent with radar callback counter
4) runtime diagnostics are parseable when present
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


def as_int(value):
    return int(float(value))


def as_float(value):
    return float(value)


def is_finite_number(value):
    try:
        return math.isfinite(float(value))
    except Exception:
        return False


def load_required_schema_columns(schema_csv):
    required = []
    with open(schema_csv, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get("required", "").strip().lower() == "true":
                field = row.get("field", "").strip()
                if field:
                    required.append(field)
    return required


def check_diag_file(path, required_cols, max_callback_lag):
    info = {
        "path": path,
        "rows": 0,
        "missing_required_columns": [],
        "empty_required_cells": 0,
        "invalid_use_radar_update": 0,
        "invalid_callback_sequence": False,
        "invalid_row_id_sequence": False,
        "runtime_reve_ms_mean": None,
        "runtime_backend_ms_mean": None,
    }
    failures = []

    if not os.path.isfile(path):
        append_fail(failures, "diag file missing: %s" % path)
        return info, failures

    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames or []

        for col in required_cols:
            if col not in fieldnames:
                info["missing_required_columns"].append(col)
        if info["missing_required_columns"]:
            append_fail(
                failures,
                "missing required columns in %s: %s" % (path, info["missing_required_columns"]),
            )
            return info, failures

        callback_values = []
        row_id_values = []
        runtime_reve_values = []
        runtime_backend_values = []

        for row in reader:
            info["rows"] += 1
            for col in required_cols:
                if row.get(col, "").strip() == "":
                    info["empty_required_cells"] += 1

            u = row.get("use_radar_update", "")
            if u not in ("0", "1"):
                info["invalid_use_radar_update"] += 1

            if "radar_scan_callback_count" in fieldnames:
                try:
                    callback_values.append(as_int(row.get("radar_scan_callback_count", "nan")))
                except Exception:
                    info["invalid_callback_sequence"] = True

            if "row_id" in fieldnames:
                try:
                    row_id_values.append(as_int(row.get("row_id", "nan")))
                except Exception:
                    info["invalid_row_id_sequence"] = True

            if "runtime_reve_ms" in fieldnames and is_finite_number(row.get("runtime_reve_ms", "nan")):
                runtime_reve_values.append(as_float(row["runtime_reve_ms"]))

            if "runtime_backend_ms" in fieldnames and is_finite_number(row.get("runtime_backend_ms", "nan")):
                runtime_backend_values.append(as_float(row["runtime_backend_ms"]))

        if info["rows"] == 0:
            append_fail(failures, "empty diag file: %s" % path)
            return info, failures

        if info["empty_required_cells"] > 0:
            append_fail(
                failures,
                "required cells empty in %s: %d" % (path, info["empty_required_cells"]),
            )

        if info["invalid_use_radar_update"] > 0:
            append_fail(
                failures,
                "invalid use_radar_update values in %s: %d" % (path, info["invalid_use_radar_update"]),
            )

        if callback_values:
            if any(callback_values[i] < callback_values[i - 1] for i in range(1, len(callback_values))):
                info["invalid_callback_sequence"] = True
            last_callback = callback_values[-1]
            callback_gap = last_callback - info["rows"]
            if callback_gap < 0 or callback_gap > max_callback_lag:
                info["invalid_callback_sequence"] = True
            if info["invalid_callback_sequence"]:
                append_fail(
                    failures,
                    "callback count mismatch in %s (rows=%d, last_callback=%d, gap=%d, allowed<=%d)"
                    % (path, info["rows"], last_callback, callback_gap, max_callback_lag),
                )

        if row_id_values:
            expected = list(range(info["rows"]))
            if row_id_values != expected:
                info["invalid_row_id_sequence"] = True
            if info["invalid_row_id_sequence"]:
                append_fail(failures, "row_id sequence mismatch in %s" % path)

        if runtime_reve_values:
            info["runtime_reve_ms_mean"] = statistics.mean(runtime_reve_values)
        if runtime_backend_values:
            info["runtime_backend_ms_mean"] = statistics.mean(runtime_backend_values)

    return info, failures


def main():
    parser = argparse.ArgumentParser(description="Gate-W4 diagnostics checker")
    parser.add_argument("--results_root", required=True)
    parser.add_argument("--manifest", default="")
    parser.add_argument("--diag_dir", default="")
    parser.add_argument("--schema", default="")
    parser.add_argument("--report_json", default="")
    parser.add_argument("--max_callback_lag", type=int, default=1)
    args = parser.parse_args()

    manifest = args.manifest if args.manifest else os.path.join(args.results_root, "run_manifest.csv")
    diag_dir = args.diag_dir if args.diag_dir else os.path.join(args.results_root, "dvc_w4_diag")
    schema = args.schema if args.schema else os.path.join(
        os.path.dirname(os.path.dirname(__file__)), "publish_plan", "templates", "dvc_diag_schema.csv"
    )
    report_json = args.report_json if args.report_json else os.path.join(args.results_root, "gate_w4_report.json")

    failures = []
    report = {
        "gate": "W4",
        "results_root": args.results_root,
        "manifest": manifest,
        "diag_dir": diag_dir,
        "schema": schema,
        "pass": False,
        "checks": {},
        "failures": [],
    }

    if not os.path.isfile(schema):
        append_fail(failures, "schema not found: %s" % schema)
        required_cols = []
    else:
        required_cols = load_required_schema_columns(schema)
        report["checks"]["required_columns"] = required_cols
        if not required_cols:
            append_fail(failures, "no required columns in schema: %s" % schema)

    if not os.path.isfile(manifest):
        append_fail(failures, "manifest not found: %s" % manifest)
        manifest_rows = []
    else:
        with open(manifest, "r", encoding="utf-8") as f:
            manifest_rows = list(csv.DictReader(f))

    success_rows = [r for r in manifest_rows if r.get("status", "") == "SUCCESS"]
    if not success_rows:
        append_fail(failures, "no SUCCESS rows in manifest")

    diag_files = []
    for row in success_rows:
        diag_file = row.get("diag_file", "").strip()
        if not diag_file:
            diag_file = os.path.join(diag_dir, "dvc_diag_%s.csv" % row.get("run_id", ""))
        diag_files.append(diag_file)

    report["checks"]["expected_diag_files"] = len(diag_files)

    seen = set()
    unique_diag_files = []
    for path in diag_files:
        if path and path not in seen:
            unique_diag_files.append(path)
            seen.add(path)

    per_file_info = []
    aggregate = defaultdict(int)
    aggregate_runtime_reve = []
    aggregate_runtime_backend = []

    for diag_file in unique_diag_files:
        info, local_failures = check_diag_file(diag_file, required_cols, args.max_callback_lag)
        per_file_info.append(info)
        for m in local_failures:
            append_fail(failures, m)

        aggregate["rows"] += info["rows"]
        aggregate["empty_required_cells"] += info["empty_required_cells"]
        aggregate["invalid_use_radar_update"] += info["invalid_use_radar_update"]

        if info["runtime_reve_ms_mean"] is not None:
            aggregate_runtime_reve.append(info["runtime_reve_ms_mean"])
        if info["runtime_backend_ms_mean"] is not None:
            aggregate_runtime_backend.append(info["runtime_backend_ms_mean"])

    report["checks"]["diag_files_checked"] = len(unique_diag_files)
    report["checks"]["aggregate"] = dict(aggregate)
    report["checks"]["per_file"] = per_file_info

    if aggregate_runtime_reve:
        report["checks"]["runtime_reve_ms_mean"] = statistics.mean(aggregate_runtime_reve)
    if aggregate_runtime_backend:
        report["checks"]["runtime_backend_ms_mean"] = statistics.mean(aggregate_runtime_backend)

    report["pass"] = len(failures) == 0
    report["failures"] = failures

    os.makedirs(os.path.dirname(report_json), exist_ok=True)
    with open(report_json, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)

    print("[gate_w4] report: %s" % report_json)
    if failures:
        print("[gate_w4] FAIL")
        for item in failures:
            print(" - %s" % item)
        sys.exit(1)

    print("[gate_w4] PASS")


if __name__ == "__main__":
    main()
