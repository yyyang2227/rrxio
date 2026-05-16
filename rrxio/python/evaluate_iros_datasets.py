#!/usr/bin/env python3

"""
This file is part of RRxIO - Robust Radar Visual/Thermal Odometry
@author Christopher Doer <christopher.doer@kit.edu>

Extended for W1-W2 strict gate execution:
- manifest logging per run
- deterministic run_id
- configurable repeat trials
- DVC diagnostic output wiring
"""

from __future__ import with_statement

import argparse
import csv
import datetime as dt
import os
import subprocess
import time

import rospkg

import evaluate_ground_truth


def _git_rev(path):
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=path, stderr=subprocess.STDOUT).decode("utf-8").strip()
    except Exception:
        return "unknown"


def _ensure_dir(path):
    if not os.path.isdir(path):
        os.makedirs(path)


def _append_manifest_row(manifest_csv, row):
    header = [
        "run_id", "date", "week", "stage", "dataset", "modality", "launch_file", "config_info", "camera_config",
        "bag_start", "bag_duration", "timeshift_cam_imu", "topic_radar_trigger", "topic_radar_scan", "feature_num",
        "git_rev_root", "git_rev_reve", "status", "owner", "note", "runtime_s", "export_directory", "diag_file"
    ]
    is_new = not os.path.isfile(manifest_csv)
    with open(manifest_csv, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=header)
        if is_new:
            writer.writeheader()
        writer.writerow(row)


def run_feature(args, n_feature, git_rev_root, git_rev_reve):
    result_base_directory = os.path.join(args.results_root, "N_" + str(n_feature))
    _ensure_dir(result_base_directory)

    n_rovio_features = " N:=" + str(n_feature)

    camera_config_visual = os.path.join(args.rosbag_dir, "rovio_visual.yaml")
    camera_config_thermal = os.path.join(args.rosbag_dir, "rovio_thermal.yaml")

    base_configs = [
        {
            "launch_file": "rrxio_evaluate_rosbag.launch",
            "name": str(n_feature) + "_rrxio_visual",
            "config": "rrxio_iros_datasets_visual.info",
            "use_vel": True,
            "modality": "visual",
            "timeshift_cam_imu": "0.002",
            "camera_config": camera_config_visual,
            "topic_cam": "/sensor_platform/camera_visual/img",
        },
        {
            "launch_file": "rrxio_evaluate_rosbag.launch",
            "name": str(n_feature) + "_rrxio_thermal",
            "config": "rrxio_iros_datasets_thermal.info",
            "use_vel": True,
            "modality": "thermal",
            "timeshift_cam_imu": "-0.004",
            "camera_config": camera_config_thermal,
            "topic_cam": "/sensor_platform/camera_thermal/img",
        },
    ]

    configs = [
        {"name": "base", "changes": {}},
    ]

    datasets = [
        {"name": "mocap_easy", "start_time": 0, "ground_truth_type": evaluate_ground_truth.VICON},
        {"name": "mocap_medium", "start_time": 0, "ground_truth_type": evaluate_ground_truth.VICON},
        {"name": "mocap_difficult", "start_time": 0, "ground_truth_type": evaluate_ground_truth.VICON},
        {"name": "mocap_dark", "start_time": 0, "ground_truth_type": evaluate_ground_truth.VICON},
        {"name": "mocap_dark_fast", "start_time": 0, "ground_truth_type": evaluate_ground_truth.VICON},
        {"name": "gym", "start_time": 0, "ground_truth_type": evaluate_ground_truth.PSEUDO_GT},
        {"name": "indoor_floor", "start_time": 0, "ground_truth_type": evaluate_ground_truth.PSEUDO_GT},
        {"name": "outdoor_campus", "start_time": 0, "ground_truth_type": evaluate_ground_truth.PSEUDO_GT},
        {"name": "outdoor_street", "start_time": 0, "ground_truth_type": evaluate_ground_truth.PSEUDO_GT},
    ]

    rospack = rospkg.RosPack()
    default_params = "shutdown_when_done:=True enable_rviz:=False"
    topic_radar_trigger = "/sensor_platform/radar/trigger"
    topic_radar_scan = "/sensor_platform/radar/scan"

    runs = len(base_configs) * len(configs) * len(datasets)
    ctr = 0
    start_time = time.time()

    manifest_csv = os.path.join(args.results_root, "run_manifest.csv")

    for base_config in base_configs:
        for dataset in datasets:
            euroc_name = dataset["name"]
            for config in configs:
                ctr += 1
                print("############################################################")
                rem_min = (runs - ctr) * (time.time() - start_time) / max(1, ctr) / 60.0
                print("%s: Progress %d / %d remaining time %0.2fmin" % (str(n_feature), ctr, runs, rem_min))
                print("############################################################")

                config_name = base_config["name"] + "_" + config["name"]
                export_directory_run = os.path.join(
                    result_base_directory,
                    base_config["modality"],
                    config_name,
                    "nuc_" + config_name + "_" + euroc_name,
                )
                _ensure_dir(export_directory_run)

                src_config = os.path.join(rospack.get_path("rrxio"), "launch", "configs", base_config["config"])
                dst_config = os.path.join(export_directory_run, config_name + ".info")
                with open(src_config, "r", encoding="utf-8") as f:
                    config_str = "".join(f.readlines())
                for old, new in config["changes"].items():
                    config_str = config_str.replace(old, new)
                with open(dst_config, "w", encoding="utf-8") as f:
                    f.write(config_str)

                # DVC diagnostic outputs by run
                diag_output_dir = os.path.join(args.results_root, "dvc_w4_diag")
                _ensure_dir(diag_output_dir)

                for k in range(args.n_trials):
                    run_id = "%s_%s_%s_%s_t%d" % (
                        dt.datetime.now().strftime("%Y%m%d_%H%M%S"),
                        str(n_feature),
                        base_config["modality"],
                        euroc_name,
                        k,
                    )

                    cmd = (
                        "roslaunch rrxio {launch_file} "
                        "bag_start:={bag_start} bag_duration:={bag_duration} "
                        "{default_params} "
                        "rosbag:={rosbag} ground_truth_csv:={gt_csv} ground_truth_type:={gt_type} "
                        "export_directory:={export_dir} config:={config_file} rosbag_dir:={rosbag_dir} "
                        "camera_config:={camera_config} topic_cam:={topic_cam} timeshift_cam_imu:={timeshift} "
                        "topic_radar_trigger:={topic_radar_trigger} topic_radar_scan:={topic_radar_scan} "
                        "dvc_diag_enabled:=true dvc_diag_output_dir:={diag_output_dir} dvc_run_id:={run_id} "
                        "id:={idv} {n_features}"
                    ).format(
                        launch_file=base_config["launch_file"],
                        bag_start=dataset["start_time"],
                        bag_duration=args.bag_duration,
                        default_params=default_params,
                        rosbag=euroc_name,
                        gt_csv=euroc_name + "_gt.csv",
                        gt_type=dataset["ground_truth_type"],
                        export_dir=export_directory_run,
                        config_file=dst_config,
                        rosbag_dir=args.rosbag_dir,
                        camera_config=base_config["camera_config"],
                        topic_cam=base_config["topic_cam"],
                        timeshift=base_config["timeshift_cam_imu"],
                        topic_radar_trigger=topic_radar_trigger,
                        topic_radar_scan=topic_radar_scan,
                        diag_output_dir=diag_output_dir,
                        run_id=run_id,
                        idv=str(n_feature),
                        n_features=n_rovio_features,
                    )

                    print(cmd)
                    t0 = time.time()
                    rc = os.system(cmd + (" >/dev/null 2>&1" if args.suppress_console_output else ""))
                    runtime_s = time.time() - t0

                    status = "SUCCESS" if rc == 0 else "FAILED"
                    diag_file = os.path.join(diag_output_dir, "dvc_diag_" + run_id + ".csv")
                    _append_manifest_row(
                        manifest_csv,
                        {
                            "run_id": run_id,
                            "date": dt.datetime.now().strftime("%Y-%m-%d"),
                            "week": args.week,
                            "stage": "baseline",
                            "dataset": euroc_name,
                            "modality": base_config["modality"],
                            "launch_file": base_config["launch_file"],
                            "config_info": dst_config,
                            "camera_config": base_config["camera_config"],
                            "bag_start": dataset["start_time"],
                            "bag_duration": args.bag_duration,
                            "timeshift_cam_imu": base_config["timeshift_cam_imu"],
                            "topic_radar_trigger": topic_radar_trigger,
                            "topic_radar_scan": topic_radar_scan,
                            "feature_num": n_feature,
                            "git_rev_root": git_rev_root,
                            "git_rev_reve": git_rev_reve,
                            "status": status,
                            "owner": args.owner,
                            "note": "trial_%d" % k,
                            "runtime_s": "%.6f" % runtime_s,
                            "export_directory": export_directory_run,
                            "diag_file": diag_file,
                        },
                    )

                    if args.n_trials > 1 and os.path.exists(os.path.join(export_directory_run, "stamped_traj_estimate.txt")):
                        os.system(
                            "mv " + os.path.join(export_directory_run, "stamped_traj_estimate.txt") + " "
                            + os.path.join(export_directory_run, "stamped_traj_estimate_%s.txt" % run_id)
                        )

                with open(os.path.join(export_directory_run, "eval_cfg.yaml"), "w", encoding="utf-8") as config_file_eval:
                    config_file_eval.write("align_type: posyaw\nalign_num_frames: -1")

    print(str(n_feature) + ": Starting evaluation...")

    evaluation_dir = os.path.join(result_base_directory, "evaluation_full_align")
    _ensure_dir(evaluation_dir)

    ws = "  "
    s = "Datasets:\n"
    for euroc_dataset in datasets:
        if euroc_dataset["ground_truth_type"] != evaluate_ground_truth.FINAL_POSE:
            name = euroc_dataset["name"]
            s += ws + name + ":\n"
            s += ws + ws + "label: " + name.replace("_", "") + "\n"

    s += "Algorithms:\n"
    algos = []
    for base_config in base_configs:
        for config in configs:
            algos.append(base_config["name"] + "_" + config["name"])

    for algo in algos:
        s += ws + algo + ":\n"
        s += ws + ws + "fn: traj_est\n"
        s += ws + ws + "label: " + algo.replace("_", "") + "\n"

    s += "RelDistances: [12,24,36,48,60]"
    eval_config_path = os.path.join(result_base_directory, "evaluation_config.yaml")
    with open(eval_config_path, "w", encoding="utf-8") as eval_config:
        eval_config.write(s)

    cmd = (
        "rosrun rpg_trajectory_evaluation analyze_trajectories.py {cfg} "
        "--output_dir={out_dir} --results_dir={results_dir} --platform nuc "
        "--odometry_error_per_dataset --overall_odometry_error --plot_trajectories "
        "--rmse_table --rmse_boxplot --png --no_sort_names"
    ).format(cfg=eval_config_path, out_dir=evaluation_dir, results_dir=result_base_directory)

    if args.n_trials > 1:
        cmd += " --mul_trials " + str(args.n_trials)

    print(cmd)
    os.system(cmd + (" >/dev/null 2>&1" if args.suppress_console_output else ""))


def main():
    parser = argparse.ArgumentParser(description="Evaluate RRxIO datasets with strict manifest logging")
    parser.add_argument("rosbag_dir", help="Path to radar_thermal_visual_inertial_datasets_iros_2021 root")
    parser.add_argument("--results_root", default="", help="Root output dir. Default: <rosbag_dir>/results/dvc_rrxio_publish/baseline_v1")
    parser.add_argument("--n_trials", type=int, default=3, help="Repeat trials per dataset/config")
    parser.add_argument("--bag_duration", type=float, default=10000.0, help="Bag duration parameter")
    parser.add_argument("--week", default="W1", help="Week tag for manifest")
    parser.add_argument("--owner", default="unassigned", help="Owner tag for manifest")
    parser.add_argument("--suppress_console_output", action="store_true", help="Suppress roslaunch/analyze console output")
    parser.add_argument("--features", default="25,15,10", help="Comma-separated feature counts")
    args = parser.parse_args()

    args.rosbag_dir = args.rosbag_dir if args.rosbag_dir.endswith("/") else args.rosbag_dir + "/"

    if args.results_root == "":
        args.results_root = os.path.join(args.rosbag_dir, "results", "dvc_rrxio_publish", "baseline_v1")
    _ensure_dir(args.results_root)

    features = [f.strip() for f in args.features.split(",") if f.strip()]
    if not features:
        raise RuntimeError("No feature count provided.")

    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    git_rev_root = _git_rev(repo_root)
    git_rev_reve = _git_rev(os.path.join(repo_root, "thirdparty", "reve"))

    start = time.time()
    for n_feature in features:
        run_feature(args, n_feature, git_rev_root, git_rev_reve)
    print("Done in %0.2f minutes" % ((time.time() - start) / 60.0))


if __name__ == "__main__":
    main()
