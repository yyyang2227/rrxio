#!/usr/bin/env python3

"""
Freeze baseline experiment inputs for DVC-RRxIO publication path.
Creates a timestamped snapshot of critical launch/config/code files,
plus SHA256 manifest and git revision metadata.
"""

import argparse
import csv
import datetime as dt
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

SNAPSHOT_FILES = [
    "rrxio/launch/rrxio_evaluate_rosbag.launch",
    "rrxio/launch/rrxio_visual_iros_demo.launch",
    "rrxio/launch/rrxio_thermal_iros_demo.launch",
    "rrxio/launch/configs/rrxio_iros_datasets_visual.info",
    "rrxio/launch/configs/rrxio_iros_datasets_thermal.info",
    "rrxio/launch/configs/default_params_radar_ego_velocity_estimation.yaml",
    "rrxio/include/rrxio/RRxIONode.hpp",
    "rrxio/include/rrxio/VelocityUpdate.hpp",
    "thirdparty/reve/radar_ego_velocity_estimator/src/radar_ego_velocity_estimator.cpp",
]


def run_cmd(cmd, cwd):
    try:
        out = subprocess.check_output(cmd, cwd=str(cwd), stderr=subprocess.STDOUT)
        return out.decode("utf-8").strip()
    except Exception as exc:
        return f"<unavailable: {exc}>"


def sha256_of_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def find_repo_root(start):
    cur = Path(start).resolve()
    while cur != cur.parent:
        if (cur / ".git").exists():
            return cur
        cur = cur.parent
    raise RuntimeError("Cannot find repository root containing .git")


def main():
    parser = argparse.ArgumentParser(description="Freeze baseline snapshot")
    parser.add_argument("--output_dir", required=True, help="Output directory for snapshots")
    parser.add_argument("--tag", default="baseline", help="Snapshot tag")
    parser.add_argument("--note", default="", help="Optional note")
    args = parser.parse_args()

    repo_root = find_repo_root(__file__)
    now = dt.datetime.now().strftime("%Y%m%d_%H%M%S")

    snapshot_dir = Path(args.output_dir).expanduser().resolve() / f"{now}_{args.tag}"
    files_dir = snapshot_dir / "files"
    files_dir.mkdir(parents=True, exist_ok=True)

    manifest_rows = []
    missing = []

    for rel in SNAPSHOT_FILES:
        src = repo_root / rel
        if not src.exists():
            missing.append(rel)
            continue
        dst = files_dir / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        manifest_rows.append((rel, sha256_of_file(src), src.stat().st_size))

    sha_path = snapshot_dir / "file_sha256.csv"
    with open(sha_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["relative_path", "sha256", "bytes"])
        writer.writerows(manifest_rows)

    metadata = {
        "created_at": now,
        "tag": args.tag,
        "note": args.note,
        "repo_root": str(repo_root),
        "git_rev_root": run_cmd(["git", "rev-parse", "HEAD"], repo_root),
        "git_status_root": run_cmd(["git", "status", "--short"], repo_root),
        "git_rev_reve": run_cmd(["git", "rev-parse", "HEAD"], repo_root / "thirdparty/reve"),
        "snapshot_file_count": len(manifest_rows),
        "missing_files": missing,
    }

    with open(snapshot_dir / "metadata.json", "w", encoding="utf-8") as f:
        json.dump(metadata, f, ensure_ascii=False, indent=2)

    readme = snapshot_dir / "README.txt"
    with open(readme, "w", encoding="utf-8") as f:
        f.write("DVC-RRxIO baseline snapshot\n")
        f.write(f"created_at: {now}\n")
        f.write(f"tag: {args.tag}\n")
        f.write(f"repo_root: {repo_root}\n")
        f.write("\nFiles:\n")
        for rel, sha, size in manifest_rows:
            f.write(f"- {rel} | sha256={sha} | bytes={size}\n")
        if missing:
            f.write("\nMissing files:\n")
            for rel in missing:
                f.write(f"- {rel}\n")

    print(f"[snapshot] created: {snapshot_dir}")
    print(f"[snapshot] files: {len(manifest_rows)}")
    if missing:
        print("[snapshot] missing files:")
        for rel in missing:
            print(f"  - {rel}")


if __name__ == "__main__":
    main()
