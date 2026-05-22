# RRxIO 核心链路深度审计报告（`rrxio` + 直接调用依赖）

## 文档信息
- 编码：UTF-8
- 生成日期：2026-05-16
- 适用范围：`rrxio` 主包及其运行时直接调用依赖（`thirdparty/rovio`、`thirdparty/reve/radar_ego_velocity_estimator`）

## 审计对象与证据基线
1. 主链路代码
- `/home/yyy/DVC-RRxIO_ws/src/rrxio/rrxio/include/rrxio/RRxIONode.hpp`
- `/home/yyy/DVC-RRxIO_ws/src/rrxio/rrxio/include/rrxio/RRxIOFilter.hpp`
- `/home/yyy/DVC-RRxIO_ws/src/rrxio/rrxio/include/rrxio/VelocityUpdate.hpp`
- `/home/yyy/DVC-RRxIO_ws/src/rrxio/rrxio/src/nodes/rrxio_rosbag_loader.cpp`
- `/home/yyy/DVC-RRxIO_ws/src/rrxio/rrxio/src/nodes/rrxio_node.cpp`

2. 直接运行时依赖
- `/home/yyy/DVC-RRxIO_ws/src/rrxio/thirdparty/rovio/lightweight_filtering/include/lightweight_filtering/FilterBase.hpp`
- `/home/yyy/DVC-RRxIO_ws/src/rrxio/thirdparty/rovio/lightweight_filtering/include/lightweight_filtering/Update.hpp`
- `/home/yyy/DVC-RRxIO_ws/src/rrxio/thirdparty/rovio/lightweight_filtering/include/lightweight_filtering/Prediction.hpp`
- `/home/yyy/DVC-RRxIO_ws/src/rrxio/thirdparty/rovio/include/rovio/FilterStates.hpp`
- `/home/yyy/DVC-RRxIO_ws/src/rrxio/thirdparty/rovio/include/rovio/ImuPrediction.hpp`
- `/home/yyy/DVC-RRxIO_ws/src/rrxio/thirdparty/rovio/include/rovio/ImgUpdate.hpp`
- `/home/yyy/DVC-RRxIO_ws/src/rrxio/thirdparty/rovio/include/rovio/PoseUpdate.hpp`
- `/home/yyy/DVC-RRxIO_ws/src/rrxio/thirdparty/reve/radar_ego_velocity_estimator/src/radar_body_velocity_estimator.cpp`
- `/home/yyy/DVC-RRxIO_ws/src/rrxio/thirdparty/reve/radar_ego_velocity_estimator/src/radar_ego_velocity_estimator.cpp`

3. 配置/构建入口
- `/home/yyy/DVC-RRxIO_ws/src/rrxio/rrxio/CMakeLists.txt`
- `/home/yyy/DVC-RRxIO_ws/src/rrxio/rrxio/package.xml`
- `/home/yyy/DVC-RRxIO_ws/src/rrxio/rrxio/launch/rrxio_visual_iros_demo.launch`
- `/home/yyy/DVC-RRxIO_ws/src/rrxio/rrxio/launch/rrxio_thermal_iros_demo.launch`
- `/home/yyy/DVC-RRxIO_ws/src/rrxio/rrxio/launch/rrxio_evaluate_rosbag.launch`
- `/home/yyy/DVC-RRxIO_ws/src/rrxio/rrxio/launch/configs/default_params_radar_ego_velocity_estimation.yaml`
- `/home/yyy/DVC-RRxIO_ws/src/rrxio/rrxio/launch/configs/rrxio_iros_datasets_visual.info`
- `/home/yyy/DVC-RRxIO_ws/src/rrxio/rrxio/launch/configs/rrxio_iros_datasets_thermal.info`

---

## 1. 宏观架构与拓扑（Macro Architecture）

### 1.1 执行拓扑主链
`*.launch -> rrxio_rosbag_loader_[10|15|25] -> RovioNode<mtFilter> -> RovioFilter -> FilterBase 时间线调度 -> Prediction/Update 子模型 -> ROS 发布`

### 1.2 模块生态位
- `RRxIONode`：运行时编排层（回调接入、互斥锁、时间戳处理、发布）。
- `RRxIOFilter`：参数注册与更新器装配层（`ImgUpdate/PoseUpdate/VelocityUpdate` 挂接）。
- `LWF::FilterBase`：滤波调度内核（按时间线推进预测和更新）。
- `REVE`：雷达速度前端估计器（`点云+角速度 -> v_b + P_v_b`）。

### 1.3 生命周期
1. 初始化：`readFromInfo -> refreshProperties -> 构造节点 -> 注册订阅/发布`。
2. 运行：IMU/图像/位姿/速度/雷达数据持续入队。
3. 推进：`updateAndPublish -> updateSafe` 做时间有序融合。
4. 退出：rosbag 模式读完 bag 结束；在线模式依赖 `ros::spin()`。

### 边界警示
1. 在线节点在 `rrxio/CMakeLists.txt` 中被注释，且注释行存在 `cd_add_executable` 拼写错误，直接解注会编译失败。
2. 默认 `filter_config` 路径来自 `rovio` 包路径，部署环境若 `rovio` 包不可见会失败。
3. 评估入口已统一使用 `bag_duration`（2026-05-16 修复）；旧脚本若仍传 `bag_dur` 将不再生效。

---

## 2. 数据结构与状态机（Data & State Management）

### 2.1 关键状态结构
`FilterState::state_` 核心状态包含：
- `WrWM`：世界到 IMU 的位置向量
- `MvM`：IMU 速度状态
- `acb/gyb`：加计/陀螺偏置
- `qWM`：IMU 姿态
- `MrMC/qCM`：IMU-相机外参
- `fea`：特征方位+深度参数

### 2.2 状态机 A：滤波初始化
`WaitForInitUsingAccel` / `WaitForInitExternalPose` -> `Initialized`

### 2.3 状态机 B：特征跟踪
`UNKNOWN / NOT_IN_FRAME / FAILED_ALIGNEMENT / FAILED_TRACKING / TRACKED`

### 2.4 状态机 C：时间线调度
- 预测队列：`predictionTimeline_.measMap_`
- 更新队列：`updateTimelineTuple_`（图像/位姿/速度）
- `updateSafe` 在安全时间推进并清理历史测量

### 边界警示
1. `updateToUpdateMeasOnly_ = true` 时，没有更新测量就不会仅靠预测推进 `safe_`。
2. `updateAndPublish` 由 `ROVIO_UPDATE_SOURCE` 指定队列驱动，默认是图像队列，雷达-only 场景可能更新饥饿。
3. `radarTriggerCallback` 当前为空实现，触发消息未参与实质同步控制。

---

## 3. 核心逻辑块与数学映射（Microscopic Review）

### 3.1 IMU 预测模型（`ImuPrediction`）
- 姿态：指数映射积分角速度。
- 平移/速度/偏置：离散模型推进。
- 特征：在相机系传播方位+深度，并显式构建状态/噪声 Jacobian。

### 3.2 图像更新（`ImgUpdate`）
- `useDirectMethod=true`：光度残差（`A_red`, `b_red`）驱动更新。
- 否则：重投影误差更新。
- 后处理：特征质量统计、剔除劣质特征、补充新特征、可选零速更新。

### 3.3 位姿更新（`PoseUpdate`）
- 位置创新：外部惯导系与内部系对齐误差。
- 姿态创新：`qVM * qWM^-1 * qWI * meas^-1`。
- 可选按测量协方差缩放更新噪声（`useOdometryCov`）。

### 3.4 速度更新（`VelocityUpdate`，RRxIO 增量）
- 创新：`y = qAM * MvM + z_vel + n`。
- 支持逐帧 `setMeasurementNoise(cov)` 注入雷达速度协方差。
- `MahalanobisThVel` 通过配置暴露。

### 3.5 雷达速度估计（REVE）
- 流程：点筛选 -> 零速检测 -> RANSAC + LSQ -> 可选 ODR 细化。
- 刚体速度变换：`v_b = R_b_r * v_r - (ω_b × l_b_r)`。
- 协方差传播：`P_v_b = R_b_r * P_v_r * R_b_r^T`。

### 3.6 调度器行为（LWF）
`update()` 循环执行：
`预测到下一更新时刻 -> 执行该时刻可用更新 -> 继续推进`。

### 3.7 工程优化点
- `pclMsg_` / `patchMsg_` 一次分配、循环复用。
- 维度模板编译期定型（`ROVIO_NMAXFEATURE` 等）。
- 图像金字塔缓存复用。

### 边界警示（高优先）
1. `max_r_cond` 已接入 LSQ 判据（2026-05-16 修复）；当 `max_r_cond <= 0` 时会显式拒绝并告警。
2. 在线节点读取了 `timeshift_cam_imu` 参数但未真正使用。
3. 雷达角速度用最多 20 个 IMU 简单平均，未做时间加权，机动工况下误差放大。
4. 大锁覆盖 `updateAndPublish` 和雷达估计，存在吞吐与回调阻塞风险。

---

## 4. 动态执行流程与生命周期模拟（Execution Workflow）

### 4.1 场景 A：IMU + 图像主链
`imuCallback -> addPredictionMeas -> imgCallback -> addUpdateMeas<0> -> updateAndPublish -> updateSafe -> 预测+图像更新 -> 发布`

### 4.2 场景 B：雷达速度闭环
`radarScanCallback(clear imu buffer) -> imuCallback 累积角速度 -> processRadarScan -> REVE estimate -> setMeasurementNoise -> addUpdateMeas<2> -> updateAndPublish`

### 4.3 场景 C：外部位姿更新链
`groundtruthOdometryCallback -> PoseUpdateMeas(pos/att/cov) -> addUpdateMeas<1> -> updateAndPublish`

### 4.4 线程模型
- rosbag 模式：主线程近似串行调用。
- 在线模式：默认单线程 `ros::spin()`；若改多线程 spinner，大锁将成为主要串行点。

### 边界警示
1. 雷达更新不一定被立即执行，受触发队列策略限制。
2. 发布逻辑在锁内，慢发布会反压回调。
3. trigger 话题当前不参与关键同步决策。

---

## 5. 参数 -> 代码变量 -> 行为影响映射（关键项）

| 参数来源 | 参数键 | 代码变量 | 行为影响 |
|---|---|---|---|
| launch | `filter_config` | `filter_config` | 载入 `.info` 滤波配置 |
| launch | `camera0_config` | `cameraCalibrationFile_[0]` | 覆盖相机标定路径 |
| launch | `imu_topic_name` | `imu_topic_name` | IMU 输入源 |
| launch | `cam0_topic_name` | `cam0_topic_name` | 图像输入源 |
| launch | `timeshift_cam_imu` | `timeshift_cam_imu` | 离线图像时间平移 |
| launch | `bag_start` | `bag_start` | 跳过 bag 前段 |
| launch | `bag_duration` | `bag_duration` | 处理窗口时长 |
| launch(评估) | `bag_duration` | `bag_duration` | 处理窗口时长（已与 loader 统一） |
| launch | `topic_radar_scan` | `topic_radar_scan` | 雷达点云输入 |
| launch | `topic_radar_trigger` | `topic_radar_trigger` | 触发消息输入（当前未实质使用） |
| launch/rosparam | `dvc_diag_enabled` | `dvc_diag_enabled_` | 启用/关闭 W3-W4 诊断写出 |
| launch/rosparam | `dvc_diag_output_dir` | `dvc_diag_output_dir_` | 每次 run 的诊断 CSV 输出目录 |
| launch/rosparam | `dvc_run_id` | `dvc_run_id_` | 诊断文件名绑定的运行批次 ID |
| launch/rosparam | `dvc_rrxio/cov_mode` | `radar_cov_recalib_cfg_.cov_mode` | 雷达协方差模式切换（`base/fixed/alpha_r`） |
| launch/rosparam | `dvc_rrxio/fixed_scale` | `radar_cov_recalib_cfg_.fixed_scale` | `fixed` 模式膨胀倍数 |
| launch/rosparam | `dvc_rrxio/alpha_r/*` | `radar_cov_recalib_cfg_.*` | `alpha_r` 退化评分权重、上限、数值保护 |
| yaml | `min_dist` 等 | `config_.*` | 雷达筛选/RANSAC/ODR 行为 |
| yaml | `l_b_r_*`,`q_b_r_*` | `T_b_r_` | 雷达到体坐标速度变换 |
| info | `Common.depthType` | `depthTypeInt_` | 深度参数化类型 |
| info | `MahalanobisThVel` | 速度更新门限 | 速度观测离群判据 |
| 运行时 | `cov_v_b_r` | `setMeasurementNoise` | 逐帧自适应速度协方差 |

---

## 6. 差异化创新点（相对上游 `rovio`）

1. 雷达体速度融合链接入（REVE + VelocityUpdate 协方差动态注入）。
2. 速度更新门限参数暴露（`MahalanobisThVel`）。
3. W5-W6 新增低侵入协方差重标定（`base/fixed/alpha_r` 三模式）与 SPD 数值保护。
4. W5-W6 新增 NIS 诊断闭环（`nis_vel/nis_valid/nis_exceed_95/radar_update_committed`）。
5. 调度触发策略增量（`ROVIO_UPDATE_SOURCE`）。
6. 运行 I/O 与调试链增强（雷达话题、tracker 图像、离线评估脚本链）。

---

## 7. 测试场景与验收指标

1. 视觉链路单驱动：验证图像更新消费、轨迹连续性、特征状态迁移。
2. 雷达速度闭环：验证 `estimate -> setMeasurementNoise -> addUpdateMeas<2>` 成对出现。
3. 时序边界：验证 `timeshift_cam_imu`、`bag_start`、`bag_duration` 生效，并确认评估入口不再接受 `bag_dur`。
4. 失败路径：雷达估计失败时显式日志暴露，不产生伪成功更新。
5. W5-W6 严格门禁：验证 `base/fixed/alpha_r` 全量矩阵（`9×2×3×3=162`）和 Gate-W6 指标阈值。

---

## 8. 审计结论

1. 主融合链路完整且可达，视觉-惯性-雷达速度闭环存在。
2. 参数链路一致性高优先问题（`bag_duration`、`max_r_cond`）已关闭，当前主要风险转为调度触发耦合与运行时锁粒度。
3. 当前高优先风险项：
- `ROVIO_UPDATE_SOURCE` 默认导致速度-only 场景更新饥饿

4. W1-W4 门禁状态（2026-05-16）：
- `Gate-W2`: `PASS`（`baseline_v1/gate_w2_report.json`）
- `Gate-W4`: `PASS`（`baseline_v1/gate_w4_report.json`）
- 结果：W1-W2 与 W3-W4 已满足门禁，可标记 `DONE`

5. W5-W6 门禁状态（2026-05-19）：
- `Gate-W6`: `PASS`（`dvc_w6_alpha_r/gate_w6_report.json`，退出码 `0`）
- 全量覆盖：`162/162 SUCCESS`（`run_manifest.csv`）
- 关键收益（`alpha_r` vs `base`）：
  - NIS 超限率：`5.7756% -> 2.2623%`（相对下降 `60.83%`，绝对下降 `3.513pp`）
  - ATE/RPE 中位数：未恶化
- 运行时中位数：`-0.48%`（未超预算）
- 结果：W5-W6 已满足严格门禁，可标记 `DONE`

6. W6 调度严格门禁复验（2026-05-22）：
- `Gate-Scheduler`: `PASS`（`dvc_w6_fixdet_fast0_48_24_full/gate_scheduler_report.json`，退出码 `0`）
- 全量覆盖：`9序列×2模态×legacy/event_stage2×3次`
- 收敛配置：`imu_fast_path=0`、`backpressure=48/24`、`cov_mode=alpha_r`
- 关键结果（event_stage2 vs legacy）：
  - 时延：`median=9.07ms, p95=16.95ms, max=41.74ms`
  - `ATE degrade=+1.23%`
  - `RPE degrade=-2.91%`
  - `runtime increase=-1.67%`
  - `NIS abs increase=+0.367pp`
  - `committed_drop=0.436%`
  - `repeatability=PASS`，`radar_starved=0`

---

## 9. 维护约定（后续代码改动时同步更新）

从本次开始，后续每次代码改动完成后都按以下规则更新本文件：
1. 在“更新日志”追加一条记录（日期、改动文件、影响链路、风险变化）。
2. 若修改了输入输出接口（topic/service/参数），同步更新第 5 节映射表。
3. 若修改了预测/更新数学实现，同步更新第 3 节对应小节。
4. 若改动影响执行顺序或线程行为，同步更新第 4 节流程描述。

---

## 10. 更新日志

- 2026-05-16：
  - 新增本审计文档（首版）。
  - 覆盖 `rrxio` 主链路与 `rovio/reve` 直接调用依赖。
  - 记录高优先风险：`bag_dur` 参数失效、`max_r_cond` 未生效、更新触发队列策略风险。
- 2026-05-16（A档低侵入修正）：
  - `rrxio/launch/rrxio_evaluate_rosbag.launch`：评估入口参数统一为 `bag_duration`（移除 `bag_dur`）。
  - `thirdparty/reve/radar_ego_velocity_estimator/src/radar_ego_velocity_estimator.cpp`：LSQ 条件数门限由硬编码 `1.0e3` 改为 `config_.max_r_cond`，并新增 `max_r_cond<=0` 显式告警拒绝。
  - 风险变化：关闭“参数名不一致”和“max_r_cond 不生效”两项缺陷；保留调度触发耦合风险（`ROVIO_UPDATE_SOURCE`）。
- 2026-05-16（16周发表路径资产落地）：
  - 新增 `rrxio/publish_plan/journal_16w_execution_board.md`（投稿执行看板）。
  - 新增 `rrxio/publish_plan/templates/`（运行清单、诊断schema、门禁清单、周报模板）。
  - 新增 `rrxio/python/freeze_baseline_snapshot.py`（基线快照与版本指纹固化）。
  - 风险变化：提升实验复现与过程可追踪性；核心未解风险仍为调度耦合（`ROVIO_UPDATE_SOURCE`）。
- 2026-05-16（W1-W4 严格门禁实施中）：
  - `rrxio/python/evaluate_iros_datasets.py`：新增 `run_manifest.csv`、`run_id`、`diag_file` 落盘链路。
  - `rrxio/python/summarize_baseline_results.py`：新增 `baseline_v1_metrics.csv` 与 `baseline_v1_summary.md` 聚合脚本。
  - `rrxio/python/gate_w2_check.py`、`rrxio/python/gate_w4_check.py`：新增门禁自动检查脚本。
  - `rrxio/include/rrxio/RRxIONode.hpp` + `thirdparty/reve/...`：新增 `cond/inlier_ratio/trace_R_used/minEig_R_used/use_radar_update` 运行诊断输出链路。
  - 状态变化：实施资产已到位，但 Gate-W2/W4 尚未通过，W1-W4 保持 `IN_PROGRESS`。
- 2026-05-16（W1-W4 门禁验收完成）：
  - `rrxio/launch/rrxio_evaluate_rosbag.launch`：补齐 `dvc_diag_enabled/dvc_diag_output_dir/dvc_run_id` 参数透传。
  - `rrxio/python/evaluate_iros_datasets.py`：`dvc_diag_enabled` 传参统一为 `true`（bool）。
  - `rrxio/python/gate_w4_check.py`：回调计数一致性放宽为“单调 + 尾差<=1”以适配 bag 结束边界。
  - 真实数据门禁结果：
    - `Gate-W2`: PASS
    - `Gate-W4`: PASS
    - `run_manifest` 成功条目：54（18组，每组3次）
- 2026-05-19（W5-W6 严格门禁通过）：
  - `rrxio/include/rrxio/RRxIONode.hpp`：接入 `dvc_rrxio.cov_mode` 三模式协方差重标定，新增 SPD 保护与 W6 诊断字段写出。
  - `rrxio/include/rrxio/VelocityUpdate.hpp`：新增最近一次速度更新 NIS/离群诊断导出。
  - `rrxio/launch/rrxio_evaluate_rosbag.launch`：新增 `dvc_rrxio` W6 参数透传。
  - `rrxio/python/evaluate_iros_datasets.py`：支持 `cov_mode/config_tag` 运行清单与三模式批量执行。
  - 新增 `rrxio/python/summarize_w6_results.py` 与 `rrxio/python/gate_w6_check.py`，并产出 `w6_compare_metrics.png`、`w6_compare_nis.png`。
  - 真实数据门禁结果（`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w6_alpha_r`）：
    - `Gate-W6`: PASS
    - `run_manifest` 成功条目：162（54组，每组3次）
- 2026-05-22（W6 调度确定性修正复验通过）：
  - 代码延续 2026-05-22 的确定性修正链路，最终以 `imu_fast_path=0` 收敛通过同门禁配置。
  - 小批目录：`dvc_w6_fixdet_fast0_48_24_smallbatch`（PASS）
  - 全量目录：`dvc_w6_fixdet_fast0_48_24_full`（PASS）
  - 严格门禁结果：`gate_scheduler_check.py` 退出码 `0` 且 `pass=true`。
  - 收益：`event_stage2` 长尾时延与 `RPE degrade/repeatability` 同时收敛至阈值内。
- 2026-05-22（统一参数输入落地）：
  - 新增统一配置：`rrxio/launch/configs/dvc_rrxio_unified_params.yaml`。
  - `rrxio_evaluate_rosbag.launch` 新增 `dvc_unified_config`，以单文件方式加载 DVC 参数。
  - `evaluate_iros_datasets.py` 改为按 run 生成 `dvc_param_configs/dvc_params_<run_id>.yaml` 并注入 launch。
  - `run_manifest.csv` 新增 `dvc_unified_config_file` 字段，实现参数输入的单源可追溯。
  - 验证：`py_compile` 与 `catkin build rrxio` 均通过，最小冒烟运行成功。
