# DVC-RRxIO 方案评估与落地修订报告（基于当前文档先行）

## 文档信息
- 评估对象：`rrxio/DVC-RRxIO_技术方案设计文档.md`
- 评估目标：工程落地稳定性优先（先可实现、可复现、可调试，再追求创新增益）
- 评估范围：`rrxio -> rovio(lightweight_filtering) -> reve` 运行主链与直接配置入口
- 证据原则：所有结论必须可回溯到现有代码与配置行号
- 编码：UTF-8
- 日期：2026-05-16

---

## 0. 证据基线与入口拓扑（依赖优先）

### 0.1 运行入口
- ROS 启动入口：
  - `rrxio/launch/rrxio_visual_iros_demo.launch`
  - `rrxio/launch/rrxio_thermal_iros_demo.launch`
  - `rrxio/launch/rrxio_evaluate_rosbag.launch`
- 可执行入口（离线）：`rrxio/src/nodes/rrxio_rosbag_loader.cpp`
- 节点核心：`rrxio/include/rrxio/RRxIONode.hpp`

### 0.2 滤波/更新主链
`launch -> rrxio_rosbag_loader_* -> RovioNode -> RovioFilter -> FilterBase(updateSafe) -> {ImuPrediction, ImgUpdate, PoseUpdate, VelocityUpdate}`

### 0.3 雷达速度链
`radar_scan -> RadarBodyVelocityEstimator::estimate -> setMeasurementNoise(cov) -> addUpdateMeas<2> -> updateSafe`

### 0.4 关键证据文件（最小集）
- `rrxio/include/rrxio/RRxIONode.hpp`
- `rrxio/include/rrxio/RRxIOFilter.hpp`
- `rrxio/include/rrxio/VelocityUpdate.hpp`
- `thirdparty/rovio/lightweight_filtering/include/lightweight_filtering/FilterBase.hpp`
- `thirdparty/reve/radar_ego_velocity_estimator/src/radar_body_velocity_estimator.cpp`
- `thirdparty/reve/radar_ego_velocity_estimator/src/radar_ego_velocity_estimator.cpp`
- `rrxio/launch/rrxio_evaluate_rosbag.launch`
- `rrxio/launch/configs/default_params_radar_ego_velocity_estimation.yaml`

### 《边界警示》
- 本报告以当前仓库实现为事实基线，不把文档中的未来设计自动视为“已实现”。
- 上游 `rovio/reve` 变更可能导致行号漂移，后续维护以“符号名 + 行号”双锚校验。

---

## 1. 宏观架构与拓扑（Macro Architecture）

### 1.1 模块生态位
- `RRxIONode`：运行时编排层，负责 ROS I/O、时间戳流入、互斥保护、测量入队、触发 `updateAndPublish()`。
- `RovioFilter`：状态与更新器装配层，注册 `ImgUpdate/PoseUpdate/VelocityUpdate` 到统一 LWF 调度框架。
- `FilterBase`：时间线调度层，维护 prediction/update timeline，执行 `updateSafe()` 时间推进。
- `REVE`：雷达自速度与协方差估计层，输出 `v_b_r` 与 `P_v_b` 供速度更新使用。

### 1.2 初始化与生命周期
- 初始化状态机在 `RovioNode::FilterInitializationState` 定义，状态：
  - `WaitForInitUsingAccel`
  - `WaitForInitExternalPose`
  - `Initialized`
- 生命周期路径：
  `构造节点 -> 加载参数/订阅发布 -> 首次 IMU 初始化滤波器 -> 回调驱动入队 -> updateSafe -> 发布`

### 1.3 构建时拓扑约束
- `rrxio/CMakeLists.txt` 将 `ROVIO_UPDATE_SOURCE` 固定为 `0`（图像队列）用于 `rrxio_rosbag_loader_*`。
- 该宏直接影响 `updateAndPublish()` 中用于 `getLastTime()` 的时间线选择。

### 《边界警示》
- 当前离线可执行路径默认把“图像时间线”作为安全更新时间上界，雷达更新无法独立驱动整体时间推进，设计新增调度策略时必须显式处理该耦合。
- `RovioNode` 所有核心回调均共享 `m_filter_` 互斥锁，任何重计算模块并入回调都会放大阻塞与尾延迟风险。

---

## 2. 数据结构与状态机（Data & State Management）

### 2.1 关键数据结构
- 滤波状态：`FilterState`（位姿、速度、偏置、外参、特征与协方差）。
- 雷达输入缓存：
  - `most_recent_imus_`（最近 IMU 窗）
  - `most_recent_radar_scan_`（待处理 scan）
- 更新测量：
  - 图像：`addUpdateMeas<0>`
  - 外部位姿：`addUpdateMeas<1>`
  - 速度（含雷达）：`addUpdateMeas<2>`
- 雷达观测噪声注入：`VelocityUpdate::setMeasurementNoise(const Eigen::MatrixXd&)`。

### 2.2 状态转移
- IMU 回调：
  - 已初始化：`addPredictionMeas -> updateAndPublish`
  - 未初始化：`resetWithAccelerometer` 或 `resetWithPose`，随后标记 `Initialized`
- 图像回调：聚合多相机金字塔后入队 `Update0` 并触发更新。
- 雷达回调：scan 先缓存，待后续 IMU 时间窗口满足条件后再 `processRadarScan()`。

### 2.3 调度状态机（FilterBase）
`add*Meas -> timeline -> getSafeTime -> updateSafe -> update -> doAvailableUpdates -> clean`

### 《边界警示》
- `setMeasurementNoise` 直接改写更新器内部噪声矩阵，若未来引入多源并发速度观测，需避免“后写覆盖前写”的竞态语义。
- 雷达 scan 触发处理依赖 IMU 时间推进，不是独立触发；若 IMU 中断，雷达链路会滞留。

---

## 3. 核心逻辑块/公式与现有代码映射（Microscopic Review）

### 3.1 现有代码对 DVC-RRxIO 三项创新的承载能力

#### Contribution 1：退化感知协方差重标定（`alpha_R`）
- 可承载点：
  - 已有 `setMeasurementNoise(cov)` 注入口，可替换 REVE 原始协方差。
  - REVE 输出 `P_v_b` 已完成雷达到机体系变换。
- 缺口：
  - 退化评分 `d_R` 尚无实现与日志出口。
  - `max_r_cond` 参数链路已闭环（硬编码已移除），当前缺口转为“缺少 cond/inlier_ratio 对外诊断可追溯”。

#### Contribution 2：方向性各向异性塑形（`S_k`）
- 可承载点：
  - 速度更新噪声接口接受完整矩阵，支持非对角协方差。
- 缺口：
  - 当前 REVE 未输出 `U^T U` 或方向可观测性指标，需要在 estimator 侧新增统计量或在上层从 inlier 几何重算。

#### Contribution 3：视觉质量耦合 + NIS 反馈
- 可承载点：
  - 图像更新与速度更新都在同一 FilterBase 时间轴，可在节点层获得跨模态时序。
- 缺口：
  - 尚无统一诊断结构承载 `d_V / zeta_RV / alpha_NIS / gamma`。
  - 目前门控主要依赖各更新器内建 Mahalanobis 阈值，缺少显式“雷达是否参与更新”的调度状态输出。

### 3.2 关键调用块逐段解析
- 雷达噪声注入顺序：
  `processRadarScan()` 内先 `setMeasurementNoise(cov_v_b_r)`，再写入 `velocityUpdateMeas_.vel()`，再 `addUpdateMeas<2>()`，顺序正确。
- 更新时间线选择：
  `updateAndPublish()` 使用 `std::get<ROVIO_UPDATE_SOURCE>(updateTimelineTuple_)` 获取 last time 后执行 `updateSafe(&lastTime)`。
- 雷达触发回调：
  `radarTriggerCallback` 当前主体为空，实际处理由 IMU 回调内“scan 时间 + 20ms 窗口”触发。

### 3.3 数学实现一致性判断（当前状态）
- 机体速度协方差传播：`P_v_b = R * P_v_r * R^T` 已实现，符合刚体旋转协方差变换。
- LSQ 稳定性门控：`max_r_cond` 已参与判定，并对 `max_r_cond<=0` 给出显式拒绝与告警。

### 《边界警示》
- 若直接把 DVC 协方差重标定塞进 `imuCallback` 大锁区，会显著增加回调临界区时间，可能造成图像/IMU堆积。
- 仅在上层后处理协方差而不补充一致性诊断，会出现“看似稳定但不可解释”的研究风险。

---

## 4. 动态执行流程与线程生命周期（Execution Workflow）

### 4.1 场景 A：IMU + 图像（基线）
`imuCallback(addPredictionMeas) -> imgCallback(addUpdateMeas<0>) -> updateAndPublish -> updateSafe -> 发布里程计/协方差`

### 4.2 场景 B：雷达速度闭环
`radarScanCallback(缓存scan, 清IMU窗) -> imuCallback(等待>20ms) -> processRadarScan -> setMeasurementNoise -> addUpdateMeas<2> -> updateAndPublish`

### 4.3 场景 C：外部位姿辅助
`groundtruthCallback/groundtruthOdometryCallback -> addUpdateMeas<1> -> updateAndPublish`

### 4.4 线程与锁模型
- 在线 ROS 运行时，回调可能并发调度；当前实现通过 `m_filter_` 全局互斥串行化核心状态访问。
- rosbag loader 路径下消息按读取顺序喂入，竞争较少但逻辑仍遵循同一锁模型。

### 《边界警示》
- 全局互斥虽然避免竞态，但对重计算极其敏感；DVC 新增模块必须控制在常数/线性低开销，必要时拆分“锁外计算 + 锁内提交”。
- `radarTriggerCallback` 未使用会导致“触发topic存在但不生效”的可维护性歧义。

---

## 5. 问题闭环修正卡（问题 -> 证据 -> 风险级别 -> 修正方案 -> 验收标准）

### 5.1 断点 #1：`bag_dur` vs `bag_duration` 参数键不一致
- 状态：`RESOLVED`（2026-05-16，A档修复）
- 问题：评测 launch 传入 `bag_dur`，loader 读取 `bag_duration`，导致时长控制可能失效。
- 证据：
  - `rrxio/launch/rrxio_evaluate_rosbag.launch` 已统一定义并传参 `bag_duration`
  - `rrxio/src/nodes/rrxio_rosbag_loader.cpp` 读取 `bag_duration`
- 风险级别：**High**（实验复现边界失真）
- A 档（优先，低侵入）：launch 统一改为 `bag_duration`。
  - 改动入口：`rrxio/launch/rrxio_evaluate_rosbag.launch`
  - 最小范围：仅参数名对齐，不改代码逻辑
  - 潜在回归：旧脚本若仍传 `bag_dur` 会失效
  - 回滚：恢复旧参数名
- B 档（增强兼容）：loader 同时接受 `bag_duration` 与 `bag_dur`（后者仅兼容告警）。
  - 改动入口：`rrxio_rosbag_loader.cpp` 参数读取段
  - 最小范围：读取别名 + 警告日志
  - 潜在回归：参数优先级需明确定义
  - 回滚：删除别名读取
- 验收标准：设置 3 组不同时长，实际处理终止时间与期望偏差 < 1 帧周期。

### 5.2 断点 #2：`max_r_cond` 未实质生效（硬编码 1e3）
- 状态：`RESOLVED`（2026-05-16，A档修复）
- 问题：REVE 已读取 `max_r_cond`，但 LSQ 判定仍写死 `1.0e3`。
- 证据：
  - `radar_body_velocity_estimator.cpp` 读取 `max_r_cond`
  - `radar_ego_velocity_estimator.cpp` 已改为 `if (fabs(cond) < config_.max_r_cond)`，并新增非法阈值保护
- 风险级别：**High**（参数调优不可达，论文结论可重复性下降）
- A 档（优先）：把 LSQ 条件数门限替换为 `config_.max_r_cond`。
  - 改动入口：`RadarEgoVelocityEstimator::solve3DLsq`
  - 最小范围：1 处阈值替换 + 非法阈值保护
  - 潜在回归：极端参数导致过严拒绝
  - 回滚：恢复常量
- B 档（增强）：输出 `cond` 与是否拒绝的诊断 topic/csv。
  - 改动入口：REVE ROS wrapper + 日志器
  - 最小范围：新增诊断字段，不改主算法
  - 潜在回归：日志频率过高影响实时性
  - 回滚：关闭诊断发布
- 验收标准：调参改变 `max_r_cond` 后，成功率/拒绝率曲线随阈值单调变化。

### 5.3 断点 #3：雷达触发调度与 `ROVIO_UPDATE_SOURCE` 耦合
- 问题：宏固定为 0（图像队列触发），雷达速度更新入队后仍受图像时间线主导。
- 证据：
  - `rrxio/CMakeLists.txt`：`ROVIO_UPDATE_SOURCE=0`
  - `RRxIONode::updateAndPublish()`：`std::get<ROVIO_UPDATE_SOURCE>(updateTimelineTuple_)`
- 风险级别：**Medium-High**（视觉间歇时雷达更新时效性受限）
- A 档（优先）：维持宏不动，仅在文档与诊断里显式声明“图像主时钟”并记录雷达等待时间。
  - 改动入口：文档/日志字段
  - 最小范围：零算法入侵
  - 潜在回归：无
  - 回滚：移除日志
- B 档（增强）：新增可配置调度源（图像/速度）并在无图像窗口允许速度队列推进。
  - 改动入口：`updateAndPublish()` + CMake/参数化接口
  - 最小范围：调度选择逻辑 + 运行参数
  - 潜在回归：与图像更新同步假设冲突，需要额外一致性测试
  - 回滚：默认回退到 source=0
- 验收标准：图像降频或间歇丢帧场景下，雷达更新延迟统计显著下降且不引入状态跳变。

### 5.4 断点 #4：`setMeasurementNoise` 与 `addUpdateMeas<2>` 时序一致性
- 问题：当前时序正确，但缺少“噪声矩阵来源标签/时间戳一致性”审计字段。
- 证据：`processRadarScan()` 中顺序为 `setMeasurementNoise -> set vel -> addUpdateMeas<2>`。
- 风险级别：**Medium**（后续加入 DVC 重标定后难排查“错配噪声”）
- A 档（优先）：增加每次速度更新的诊断记录：`t_meas, trace(R), minEig(R), source`。
  - 改动入口：`processRadarScan()` 周边日志
  - 最小范围：只增诊断
  - 潜在回归：日志开销
  - 回滚：关闭日志
- B 档（增强）：把速度测量与协方差封装为同一结构体入队，消除隐式共享状态。
  - 改动入口：`VelocityUpdateMeas` 扩展与 update 接口
  - 最小范围：接口级改动，影响较大
  - 潜在回归：破坏现有 update 模板兼容
  - 回滚：恢复全局噪声注入方式
- 验收标准：随机扰动压测下，不出现“速度值来自帧A、协方差来自帧B”的错配日志。

### 5.5 断点 #5（新增）：`radarTriggerCallback` 语义空转
- 问题：订阅了 trigger topic，但回调主体几乎为空，实际触发由 IMU 时间窗代理。
- 风险级别：**Medium**（接口语义误导）
- A 档（优先）：文档显式声明 trigger 当前不参与主逻辑，避免误配。
- B 档（增强）：将 trigger 作为雷达批次完成标记，替代固定 20ms 等待常量。
- 验收标准：触发机制在无歧义文档与日志中可追踪。

### 5.6 W1-W4 严格门禁执行状态（当前）
- 状态：`RESOLVED`（2026-05-16）
- 已完成实现：
  - W1-W2：`run_manifest.csv`、快照脚本、基线指标汇总脚本、Gate-W2 检查脚本。
  - W3-W4：REVE `cond/inlier_ratio` 诊断透传、节点侧 `dvc_diag_<run_id>.csv` 写出、Gate-W4 检查脚本。
- 验收证据：
  - 数据目录：`/home/yyy/datasets/irs_rtvi_datasets_2021`
  - 结果目录：`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/baseline_v1`
  - 门禁报告：`gate_w2_report.json` 与 `gate_w4_report.json` 均为 `pass=true`
  - 运行配置：`features=25`、`n_trials=3`、`bag_duration=60`
- 结论：
  - W1-W2 与 W3-W4 可标记为 `DONE`，允许推进 W5+。

### 5.7 W5-W6 严格门禁执行状态（当前）
- 状态：`RESOLVED`（2026-05-19）
- 已完成实现：
  - 节点侧接入 `dvc_rrxio.cov_mode={base,fixed,alpha_r}` 协方差模式切换。
  - 接入中等增强 `alpha_R`（`cond + inlier_ratio + n_targets`）与 SPD 保护后再注入 `setMeasurementNoise`。
  - 速度更新器输出 NIS 诊断（`nis_vel/is_outlier/seq`），并落盘到 `dvc_diag_<run_id>.csv`。
  - 评估脚本支持三模式批跑并写入 `run_manifest.csv(cov_mode,config_tag)`。
  - 新增 W6 汇总与门禁脚本：`summarize_w6_results.py`、`gate_w6_check.py`。
- 验收证据：
  - 数据目录：`/home/yyy/datasets/irs_rtvi_datasets_2021`
  - 结果目录：`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w6_alpha_r`
  - 核心文件：`run_manifest.csv`、`w6_metrics.csv`、`w6_summary.md`、`w6_compare_metrics.png`、`w6_compare_nis.png`、`gate_w6_report.json`
  - 全量覆盖：`9 序列 × 2 模态 × 3 次重复 × 3 模式 = 162`，且 `SUCCESS=162`
  - 严格门禁：`gate_w6_check.py` 退出码 `0` 且 `pass=true`
  - 关键指标（`alpha_r` vs `base`）：
    - NIS 超限率：`5.7756% -> 2.2623%`（相对下降 `60.83%`，绝对下降 `3.513pp`）
    - ATE 中位数：`0.113182 -> 0.101191`（未恶化）
    - RPE 中位数：`0.074655 -> 0.074027`（未恶化）
    - 运行时中位数：`10.111s -> 10.063s`（增量 `-0.48%`）
- 结论：
  - W5-W6 可标记为 `DONE`，可进入 W7-W8（`S_k`）阶段。

### 5.8 W6 调度确定性修正复验（2026-05-22）
- 状态：`RESOLVED`（同门禁配置下通过）
- 背景：
  - 代码级确定性修正回合后，历史失败项曾收敛到单项 `RPE degrade`。
  - 通过参数-调度联合复验（保持不改状态维度、不改 REVE 主算法）完成闭环。
- 最终收敛配置：
  - `cov_mode=alpha_r`
  - `scheduler_mode=event_stage2`
  - `dvc_scheduler_backpressure_high/low = 48/24`
  - `dvc_scheduler_imu_fast_path = 0`
- 验收证据：
  - 小批目录：`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w6_fixdet_fast0_48_24_smallbatch`
  - 全量目录：`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w6_fixdet_fast0_48_24_full`
  - 门禁报告：`gate_scheduler_report.json`（`pass=true`，退出码 `0`）
  - 全量关键指标（event_stage2 vs legacy）：
    - 时延：`median=9.07ms, p95=16.95ms, max=41.74ms`
    - `ATE degrade=+1.23%`
    - `RPE degrade=-2.91%`
    - `runtime increase=-1.67%`
    - `NIS abs increase=+0.367pp`
    - `committed_drop=0.436%`
    - `repeatability=PASS`，`radar_starved=0`
- 结论：
  - W6 在“精度/一致性/时延/重复性”四维门禁下完成收敛，后续可进入 W7-W8。

### 5.9 W7-W8（`S_k`）严格门禁状态（2026-05-23）
- 状态：`BLOCKED`（未通过严格 Gate-W8）
- 已实施内容：
  - P0：REVE 导出观测几何后，统一到体坐标系再计算特征值供 `S_k` 使用。
  - P1：`alpha_r_sk` 路径新增轻量激活门（`lambda3_obs/d_r/n_targets`）与显式跳过诊断。
  - 节点侧实现 `R_used = S_k * (alpha_R * R_reve + sigma_min2 I) * S_k^T`，并统一 SPD 投影。
  - 新增诊断列：`s_k_valid,s_k_applied,s_k_skip_reason,sk_gate_lambda3_pass,sk_gate_d_r_pass,sk_gate_ntargets_pass,lambda1_obs,lambda2_obs,lambda3_obs,s1,s2,s3,trace_R_after_alpha,trace_R_after_sk`。
  - 新增门禁与汇总脚本：`gate_w8_check.py`、`summarize_w8_results.py`。
- 证据目录：
  - S0：`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w8_p0p2_s0_smallbatch`
  - S1：`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w8_p0p2_s1_smallbatch`
  - S2：`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w8_p0p2_s2_smallbatch`
- 严格门禁结果（P2 三档）：
  - S0：`rpe_p95_improve=-1.64%`（FAIL，仅该项失败）
  - S1：`rpe_p95_improve=-1.30%` + `ATE degrade=+5.27%`（FAIL）
  - S2：`rpe_p95_improve=-1.84%`（FAIL，仅该项失败）
  - 共性：`total_sk_rows=2256`、`radar_starved=0`，但 `outdoor_street` 组 `s_k_applied_count=0`。
- 结论：
  - 当前 `S_k` 实现在本数据子集上可维持大多数稳定性副项，但未产生目标级 `rpe_p95` 收益（严格阈值 `>=20%`）。
  - 主要拖累组为 `indoor_floor/visual`，该组在三档参数下均为负向改善。
  - 按规则保持 `IN_PROGRESS/BLOCKED`，不得标记 `DONE`。
  - 例外条款：允许在“W8 冻结基线”前提下推进 W9-W10 受控试验，但不得回写为 W7-W8 完成。

### 5.10 W7-W8 Gate重构收敛回合（2026-05-24）
- 状态：`BLOCKED`（严格 Gate-W8 仍未通过）
- 本回合执行顺序：
  - 阶段A：`d_r × obs_trace` 网格预筛（`4×4`）
  - 阶段B1：固定 Gate 后扫描 `c_obs ∈ {0.35,0.50,0.65}`
  - 阶段B2：固定 `c_obs=0.35` 扫描 `tau_obs ∈ {6,8,10}`
  - 阶段B3：固定 `c_obs=0.35,tau_obs=8` 扫描 `s_max ∈ {1.8,2.0,2.2}`
- 关键证据文件：
  - `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w8_stageA_prescan_results.csv`
  - `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w8_stageB1_c_scan_smallbatch_results.csv`
  - `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w8_stageB2_tau_scan_smallbatch_results.csv`
  - `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w8_stageB3_smax_scan_smallbatch_results.csv`
- 最优已验证组合（小批）：
  - `d_r=0.70, obs_trace=55, c_obs=0.35, tau_obs=8, s_max in [1.8,2.2]`
  - `coverage_pass=true`
  - `rpe_p95_improve=+9.495%`
- 失败点：
  - 严格阈值要求 `rpe_p95_improve>=20%`，当前最优仍不足（`+9.495%`）。
  - 其余副项（`ATE/RPE/runtime/NIS/committed_drop`）均在门限内，`radar_starved=0`。
- 结论：
  - 当前回合表明主约束已从“稳定性副项”转为“主效应幅度不足”。
  - W7-W8 必须继续保持 `IN_PROGRESS/BLOCKED`，不得标记 `DONE`。

### 5.11 W9-W10（Contribution-3）平衡严格档 v1 状态（2026-05-29）
- 状态：`BLOCKED`（strict Gate-W10 未通过）
- W8 冻结基线：
  - `d_r=0.70, obs_trace=55, c_obs=0.35, tau_obs=8, s_max=2.0`（`s_max` 在 `1.8~2.2` 内等价）。
  - W8 保持 `BLOCKED` 原因：`rpe_p95_improve<20%`（主效应不足）。
- W9（`alpha_NIS`）小批三档结论（`4序列×2模态×3次`）：
  - A0：`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_A0_smallbatch`
    - `focus_rpe_p95_improve=-9.53%`，`focus_nis_drop=0%`，`committed_drop=0%`
  - A1：`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_A1_smallbatch`
    - `focus_rpe_p95_improve=-9.45%`，`focus_nis_drop=-1.92%`
  - A2：`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_A2_smallbatch`
    - `focus_rpe_p95_improve=-9.45%`，`focus_nis_drop=-1.92%`
- W10（`zeta_RV + gate`）三档结论（固定 A0）：
  - C0：`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_C0_smallbatch`
    - `committed_drop=47.03%`，`rpe_degrade=50.16%`
  - C1：`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_C1_smallbatch`
    - `committed_drop=35.65%`，`rpe_degrade=14.92%`
  - C2：`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_C2_smallbatch`
    - `committed_drop=53.36%`，`rpe_degrade=69.12%`
- 最小修正回合（参数内）：
  - R1（`zeta_only`, gate关闭）：`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_R1_zeta_only_smallbatch`
    - `committed_drop=-1.16%`，`focus_nis_drop=+40.38%`，但 `rpe_degrade=14.45%`，`focus_rpe_p95_improve=-15.61%`
- 结论：
  - W9-W10 代码链与诊断链有效（`alpha_nis_sat_rate=0`, `radar_starved=0`，字段完整）。
  - strict Gate-W10 主失败项稳定为 `focus_rpe_p95_improve<10%`，且启用质量门控会显著拉低雷达提交计数。
  - 按规则保持 `BLOCKED`，不得标记 `DONE`，不进入 W11+。

### 5.12 W9-W10 根因修复回合（2026-05-30，P0-P4 + R0-R3）
- 状态：`BLOCKED`（strict Gate-W10 仍未通过）
- 本轮源码级修复（非补丁绕过）：
  - `RRxIONode`：
    - `alpha_NIS` 改为对称区间 `[alpha_min, alpha_max]`；该回合初版遗忘支路回归 `alpha_min`，后续 visual-only 修复已改为回归中性 `1.0`。
    - `zeta_RV` 改为围绕 `1` 的双向有界缩放（`tanh` 形式）。
    - 质量门控改为“硬拒绝兜底 + 软惩罚主导”，新增 `quality_soft_scale`。
  - `summarize_w10_results.py`：
    - `quality_reject_count` 口径改为仅计 `radar_update_reject_reason=="quality_gate"`。
    - 新增 `quality_reject_rate / hard_reject_count / max_consecutive_quality_reject`。
  - `gate_w10_check.py`：
    - 新增 `health_pass` 与 `strict_pass`，`pass` 保持严格绑定 `strict_pass`。
- 小批实验矩阵：`4序列×2模态×3次`，`alpha_r_sk` vs `alpha_r_sk_nis_rv`，均启用 `event_stage2`。
- 回合结果汇总：
  - R0 默认：`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_r0`
    - `focus_rpe_p95_improve=-18.99%`，`focus_nis_drop=-51.92%`，`ATE +14.43%`，`RPE +14.61%`（FAIL）
  - R1（soft gate 扫描）最佳 `soft_hi`：
    - `focus_rpe_p95_improve=-14.97%`，`focus_nis_drop=-40.38%`，`RPE +7.70%`（FAIL）
  - R2（zeta 扫描）最佳 `zeta_c3`：
    - `focus_rpe_p95_improve=-4.12%`，`focus_nis_drop=+25.00%`，`RPE +6.89%`（FAIL）
  - R3（alpha_NIS 扫描）最佳 `alpha_a0`：
    - `focus_rpe_p95_improve=-9.30%`，`focus_nis_drop=+32.69%`，`RPE +9.81%`（FAIL）
- 结论：
  - `health_gate` 全部通过：`radar_starved=0`，拒绝率与连续拒绝段可追踪，`committed_drop` 不超阈。
  - `strict_gate` 主失败项收敛为两条：
    1) `focus_rpe_p95_improve < 10%`
    2) `RPE degrade > 5%`
  - 当前最优候选为 `soft_hi + zeta_c3`，但仍不满足 strict；W9-W10 保持 `BLOCKED`。

### 5.13 W10 visual-only 根因修复与九数据集验证（2026-05-31）
- 状态：`BLOCKED`（visual-only strict Gate-W10 仍未通过）
- 范围口径：
  - W10 后续只以 `visual` 作为投稿主线验证口径；thermal 不再参与 W10 Gate，但代码路径保留兼容。
  - 评测、汇总、门禁脚本新增 `--modalities visual|thermal|visual,thermal`；visual-only Gate 检测到 thermal 行会直接失败。
- 源码级修复：
  - `quality_soft_scale` 从“视觉好也可能惩罚雷达”的旧形式，改为“雷达质量不足为主、双退化额外惩罚”：
    `k_r max(0,tau_r_high-q_r) + k_v max(0,tau_r_high-q_r) max(0,tau_v_low-q_v)`。
  - `zeta_RV` 改为围绕参考退化点居中的 `tanh` 形式，并新增 `zeta_dr_ref/zeta_dv_ref`。
  - `alpha_NIS` 未提交/跳过分支改为回归中性 `1.0`，避免低边界粘滞；`alpha_nis_sat` 同步修正为统计上下边界真实饱和。
  - 统一 YAML 与评测默认参数更新为当前 visual-only 阻塞最优基线：`alpha_min=1.0,zeta_min=1.0,gate_soft_scale_max=1.4`。
- 九数据集 visual-only 实验结论：
  - V1（softfix）：`NIS` 相对下降 `36.88%`，但 `RPE degrade=6.96%`、`RPE95 improve=-18.83%`，strict FAIL。
  - V2（centered zeta）：`RPE degrade=6.56%`、`RPE95 improve=-18.89%`、`NIS` 相对下降不足 `15%`，strict FAIL。
  - V3（VC1）：`ATE degrade=15.69%`、`RPE95 improve=-19.79%`，strict FAIL。
  - V4（selective inflation）：当前最优阻塞基线，`ATE degrade=-10.08%`、`NIS` 相对下降 `24.11%`、`runtime=-1.98%`、`committed_drop=-0.46%`；但 `RPE degrade=5.53%`、`RPE95 improve=-5.82%`，strict FAIL。
  - V5（hard reject probe）：硬拒绝低 `q_r` 帧能局部改善 `mocap_medium`，但会使 `mocap_easy` 提交数下降 `25.93%` 且 `ATE degrade=47.85%`，不能作为全局策略。
- 风险再评估：
  - 已确认 Contribution-3 对一致性指标有效：V4 `NIS` 相对下降 `24.11%` 且 `alpha_nis_sat_rate=0`。
  - 仍未证明其能稳定改善 visual RPE95 长尾；主风险从“机制不可运行”转为“少数轨迹长尾段的协方差调制方向与误差尖峰不匹配”。
  - 下一轮不建议继续大网格扫参，应做逐帧 spike 关联分析，定位 `zeta_rv/quality_soft_scale/alpha_nis_new` 与 RPE95 异常段之间的因果关系。

### 5.14 原始仓库基线复核、NIS一致性口径与参数清理（2026-05-31）
- 状态：`BLOCKED`（相对原始 W1 有部分收益，但 visual-only strict Gate-W10 仍未通过）
- 基线口径修正：
  - 投稿主指标必须对比原始仓库输出：`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/baseline_v1/original_visual_w10_metrics.csv`。
  - W8/W10 中间结果只作为消融，不再作为“系统改进是否有效”的主基线。
  - `summarize_w10_results.py` 支持 W1 旧诊断结构：当 `radar_update_committed` 不存在但 `use_radar_update` 存在时，用 `use_radar_update` 作为 legacy committed proxy；原始 W1 不具备 NIS 列，因此不伪造原始 NIS。
- 对原始 W1 的 visual-only 复核：
  - 总体：ATE `-1.95%`、RPE `-2.49%`、runtime `+0.22%`、committed drop `1.83%`。
  - 失败：RPE95 `-2.32%`，未达到 `>=10%` 改善要求。
  - 分组问题：`gym` 与 `outdoor_campus` runtime 分别约 `+144.68%/+154.00%`；`outdoor_street` 的 ATE/RPE/RPE95 均恶化。
- NIS口径修正：
  - NIS 超限率应接近三维速度观测的理论 95% 卡方超限比例 `5%`，不是越低越好。
  - `gate_w10_check.py` 新增 `nis_gate_mode=band`，默认检查当前 NIS 是否位于 `[1%,8%]` 并接近 `5%`。
  - 当前 V4 全局 NIS 超限率为 `1.997%`，整体偏保守；分组上 `mocap_difficult/visual=11.51%` 偏过自信，其余多数 visual 组 `<1%` 偏保守。
- 废弃参数清理：
  - 删除 top-level `scheduler_backpressure_depth/high/low/timeout` 与 `scheduler_drain_timeout_s` 外部入口。
  - `rrxio_rosbag_loader.cpp` 仅读取 `dvc_rrxio/scheduler/backpressure_high`、`backpressure_low`、`backpressure_timeout_s`、`drain_timeout_s`。
  - 统一 YAML 成为 DVC 新增参数的唯一配置入口，降低旧参数残留干扰运行的风险。
- 下一步方向：
  - 不继续以“整体 NIS 下降”为目标；改为 per-dataset/per-segment NIS band 一致性。
  - 增加 RPE95 spike 与 `zeta_rv/quality_soft_scale/alpha_nis_new/radar latency` 的逐帧关联，优先定位长尾来源。
  - Gate 增加 per-dataset runtime 或 runtime p95，避免全局 median 掩盖局部运行时异常。

### 《边界警示》
- 以上修正优先级遵循“先消除参数/调度不一致，再引入新统计模型”。
- 断点 #1/#2 与 W1-W4 门禁均已完成；后续进入 W5+ 时仍需保持“单创新点分阶段消融”。

---

## 6. 《创新点评估》与落地优先级路线图

### 6.1 创新点可行性评分（1-5）
- 创新点 A：退化感知协方差重标定（`alpha_R`）
  - 可行性：**5/5**
  - 原因：已有速度协方差注入接口，接入成本最低。
- 创新点 B：方向选择性塑形（`S_k`）
  - 可行性：**4/5**
  - 原因：数学上可行，但需新增几何统计输出/重算路径。
- 创新点 C：视觉耦合 + NIS 反馈
  - 可行性：**3/5**
  - 原因：需跨模态诊断体系与调度协同，工程耦合最高。

### 6.2 不建议立即做（避免一次性过改）
- 不建议第一阶段切到 Ceres/FGO 版本。
- 不建议第一阶段改状态维度或重写视觉前端。
- 不建议在未补齐诊断前直接启用复杂自适应门控。

### 6.3 建议里程碑（落地顺序）
1. 修复参数与阈值闭环（断点 #1/#2，已完成）。
2. 加入最小诊断集（cond、trace/minEig(R)、update_use_flag，W3-W4 已完成）。
3. 接入 `alpha_R`（各向同性版本，W5-W6 已完成并通过 Gate-W6）。
4. 接入 `S_k`（方向性版本）。
5. 最后引入 `alpha_NIS` 与视觉耦合 `zeta_RV`。

### 6.4 关键伪代码（建议版本）
```cpp
if (!reve.estimate(scan, w, v, R_reve)) return;  // 显式失败

RadarQuality rq = evalRadarQuality(scan, reve_diag);
VisualQuality vq = evalVisualQuality(vio_diag);

Matrix3d R_used = R_reve;
if (enable_alpha_r) {
  R_used = alphaR(rq) * R_used;
}
if (enable_Sk) {
  R_used = S_k(rq) * R_used * S_k(rq).transpose();
}
if (enable_nis_feedback) {
  R_used = alphaNis(prev_nis_state) * R_used;
}

R_used = projectSPD(symmetrize(R_used));
vel_update.setMeasurementNoise(R_used);
addUpdateMeas<2>(v, t_meas);
```

### 《边界警示》
- `alpha_R`、`S_k`、`alpha_NIS` 同时上线会导致调参不可辨识；必须分阶段上线并固定其余模块做消融。
- NIS 反馈若无遗忘机制与上限，容易长期饱和导致雷达更新失效。

---

## 7. 测试场景与验收标准（对应本仓库）

### 7.1 场景 1：视觉链路单独驱动（IMU+图像）
- 目标：验证新增模块不会破坏原收敛与发布节拍。
- 验收：ATE/RPE 与基线持平，发布频率波动在可接受范围内。

### 7.2 场景 2：雷达速度链路驱动
- 目标：验证 `estimate -> R重标定 -> setMeasurementNoise -> addUpdateMeas<2>` 闭环。
- 验收：每帧都有可追踪的 `R_used` 诊断记录；更新成功率与质量指标一致。

### 7.3 场景 3：时序与同步边界
- 目标：验证 `timeshift_cam_imu`、bag 参数、调度源配置对执行顺序影响。
- 验收：时间差日志与预期一致，无异常回滚/倒序更新。

### 7.4 场景 4：失败路径显式暴露
- 目标：雷达估计失败、视觉退化、外参异常时不静默掩盖。
- 验收：错误计数与告警日志完整，状态保持策略清晰可解释。

### 7.5 场景 5：一致性与开销
- 目标：NIS 超限率与协方差可信度提升，运行时开销受控。
- 验收：
  - NIS 超限率向理论区间收敛
  - `corr(trace(R), vel_err^2)` 改善
  - 运行时增量满足目标预算

### 《边界警示》
- 没有 GT 速度时，差分近似速度的滤波窗口会显著影响 RMSE 结论，必须固定并在实验报告中声明。

---

## 8. 后续新方案接入与差异评估规则

### 8.1 接入规则
- 你提供新方案后，仅新增《差异评估》章节，不重写本基线全量内容。
- 差异评估固定模板：
  - 变更点
  - 影响调用链
  - 风险级别
  - 改动范围预估（文件/函数）
  - 关键伪代码
  - 可行性评分（1-5）

### 8.2 决策输出格式（保持决策完备）
每个新增点必须给出：
- 改动入口函数
- 最小改动范围
- 潜在回归
- 回滚方式
- 验收标准

### 《边界警示》
- 若新方案引入状态维度变化或线程模型改造，必须升级为“架构级变更评审”，不能按本报告的低侵入假设直接落地。

---

## 9. 文档维护约定（每次代码改动后同步更新）

### 9.1 触发条件
以下任一条件触发本报告更新：
- 修改 `RRxIONode/RRxIOFilter/VelocityUpdate/FilterBase/REVE estimator` 任一核心逻辑。
- 修改 launch/config 导致参数名、默认值、topic、调度行为变化。
- 修改诊断字段或实验评估脚本。

### 9.2 最小更新动作
1. 更新“受影响章节”中的证据行号。
2. 更新“问题闭环修正卡”状态（未开始/进行中/已完成/回滚）。
3. 追加《更新日志》并重新评估残余风险。

### 9.3 状态标记规范
- `OPEN`：问题存在未处理
- `MITIGATING`：已开始处理，尚未验收
- `RESOLVED`：已验收关闭
- `REOPENED`：回归或新证据触发重开

---

## 10. 更新日志

### 2026-05-16（v1.0）
- 新建本报告，完成基于当前仓库实现的工程落地评估基线。
- 锁定 5 个关键断点并给出 A/B 两档修正策略。
- 建立后续“每次代码改动后同步更新”维护机制。

### 2026-05-16（v1.1）
- 新增 `rrxio/publish_plan/journal_16w_execution_board.md`，将16周投稿路径落地为可执行看板（阶段门禁、周任务、验收口径）。
- 新增 `rrxio/publish_plan/templates/` 模板集：
  - `run_manifest_template.csv`
  - `dvc_diag_schema.csv`
  - `phase_gate_checklist.md`
  - `weekly_review_template.md`
- 新增 `rrxio/python/freeze_baseline_snapshot.py`，用于基线配置快照、文件哈希与版本元数据固化（W1-W2执行入口）。

### 2026-05-16（v1.2）
- 完成 W1-W4 基础实施资产补齐（未宣告完成）：
  - `rrxio/python/evaluate_iros_datasets.py`：新增 `run_manifest.csv`、`run_id`、`diag_file` 落盘。
  - `rrxio/python/summarize_baseline_results.py`：新增 `baseline_v1_metrics.csv` 与 `baseline_v1_summary.md` 统计导出。
  - `rrxio/python/gate_w2_check.py` 与 `rrxio/python/gate_w4_check.py`：新增 Gate 自动检查脚本。
  - `RRxIONode` + `REVE`：新增 `cond/inlier_ratio/trace_R/minEig_R/use_radar_update` 运行诊断落盘链路。
- 看板状态规则强化：Gate 未 PASS 前，W1-W4 仅允许 `TODO/IN_PROGRESS/BLOCKED`，禁止 `DONE`。

### 2026-05-16（v1.3）
- 修复诊断参数透传断点：
  - `rrxio/launch/rrxio_evaluate_rosbag.launch` 新增 `dvc_diag_enabled/dvc_diag_output_dir/dvc_run_id` 入参并传递到节点参数。
  - `rrxio/python/evaluate_iros_datasets.py` 将 `dvc_diag_enabled` 传参统一为 `true`（bool）。
- 修正 Gate-W4 误报：
  - `rrxio/python/gate_w4_check.py` 将雷达回调一致性判定改为“单调 + 允许尾差 `max_callback_lag<=1`（默认）”。
- 完成真实数据门禁验收（`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/baseline_v1`）：
  - `Gate-W2`: PASS（`min_trials=3`）
  - `Gate-W4`: PASS
  - 统计：`SUCCESS_ROWS=54`，`GROUPS=18`，且每组 `MIN_PER_GROUP=3`。

### 2026-05-19（v1.4）
- 完成 W5-W6（Contribution-1）代码落地与严格门禁验收：
  - `RRxIONode`：新增 `cov_mode(base|fixed|alpha_r)`、`alpha_R` 重标定、SPD 投影保护、W6 诊断列写出。
  - `VelocityUpdate`：新增最近一次速度更新 `nis_vel/is_outlier/seq` 诊断导出接口。
  - `rrxio_evaluate_rosbag.launch`：新增 `dvc_rrxio` W6 参数透传。
  - `evaluate_iros_datasets.py`：支持三模式全量批跑与 `run_manifest(cov_mode/config_tag)`。
  - 新增 `summarize_w6_results.py`、`gate_w6_check.py`。
- 全量实验与门禁结论：
  - 结果目录：`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w6_alpha_r`
  - 覆盖规模：`162` runs 全部成功。
  - Gate-W6：`PASS`（`pass=true`，退出码 `0`）。
- 风险变化：
  - 已关闭 “`alpha_R` 缺失/NIS 无闭环” 风险。
  - 保留 `ROVIO_UPDATE_SOURCE` 调度耦合与大锁临界区开销风险，进入 W7+ 前继续监控。

### 2026-05-22（v1.5）
- 完成 W6 调度确定性修正复验并通过严格门禁（小批 + 全量）：
  - 路径：`dvc_w6_fixdet_fast0_48_24_smallbatch`、`dvc_w6_fixdet_fast0_48_24_full`
  - 参数：`imu_fast_path=0`、`backpressure=48/24`（其余维持 W6 基线）
  - 报告：`gate_scheduler_report.json` 均为 `pass=true`
- 结果变化：
  - `event_stage2` 雷达提交时延长尾被压到绝对门限内（`max=41.74ms`）。
  - `RPE degrade` 从失败项收敛为改善项（`-2.91%`）。
  - `repeatability` 与 `radar_starved` 均满足门禁。

### 2026-05-22（v1.6）
- 新增统一参数输入机制（单配置文件入口）：
  - 新增：`rrxio/launch/configs/dvc_rrxio_unified_params.yaml`
  - launch 新增参数：`dvc_unified_config`，节点启动前统一加载该 YAML。
  - 评测脚本改造：`evaluate_iros_datasets.py` 按 run 生成
    `dvc_param_configs/dvc_params_<run_id>.yaml` 并仅通过 `dvc_unified_config` 注入 DVC 相关参数。
  - `run_manifest.csv` 增加 `dvc_unified_config_file` 字段，参数追踪链从“命令行散参”收敛为“单文件可追溯”。
- 验证：
  - `python3 -m py_compile rrxio/python/evaluate_iros_datasets.py` 通过。
  - `catkin build rrxio` 通过。
  - 最小冒烟：`tmp_unified_cfg_smoke` 运行成功，已生成并生效 `dvc_params_<run_id>.yaml`。

### 2026-05-22（v1.7）
- 完成 W7-W8 代码落地与严格门禁执行：
  - 新增 `cov_mode=alpha_r_sk` 路径、`S_k` 参数组、REVE 观测几何导出、W8 诊断列与门禁脚本。
  - 小批目录（默认参数）：`dvc_w8_s_k_smallbatch`
  - 小批目录（调优参数 c3）：`dvc_w8_s_k_smallbatch_tuned_c3`
- 门禁结论：
  - 两轮均 `FAIL`，当前最优（tuned-c3）失败项仅剩：
    - `rpe_p95_improve=-0.50%`（目标 `>=20%`）
  - 其余项均满足：`ATE/RPE` 未恶化、`runtime +5.60%`、`NIS` 改善、`committed` 不降、`radar_starved=0`。
- 状态变更：
  - W7-W8 保持 `IN_PROGRESS/BLOCKED`，不得标记 `DONE`（该条为当时常规路径判定）。

### 2026-05-24（v1.8）
- 完成 W7-W8 Gate重构收敛回合（阶段A/B1/B2/B3）全流程小批验证：
  - 阶段A：`d_r` 在 `0.70` 时 coverage 通过且有正向提升（`+1.304%`）；`0.80` 虽提升更高（`+4.082%`）但 coverage 失败；`0.90` 负提升。
  - 阶段B1 最优：`c_obs=0.35`，`rpe_p95_improve=+9.495%`。
  - 阶段B2 最优：`tau_obs=8`（优于 `6/10`）。
  - 阶段B3：`s_max=1.8/2.0/2.2` 结果一致（均 `+9.495%`）。
- 门禁结论：
  - strict Gate-W8 仍 `FAIL`（主失败项：`rpe_p95_improve<20%`）。
  - W7-W8 状态维持 `BLOCKED`，不得标记 `DONE`（该条为当时常规路径判定）。

### 2026-05-29（v1.9）
- 执行“W8冻结基线后推进 W9-W10”：
  - 新增 `cov_mode=alpha_r_sk_nis_rv` 与 `dvc_rrxio/contrib3/*` 运行参数链。
  - 节点侧新增 Contribution-3 诊断列：
    `d_v,q_r,q_v,zeta_rv,alpha_nis_prev,alpha_nis_new,alpha_nis_sat,quality_gate_pass,quality_gate_reason,radar_update_reject_reason,visual_feature_valid_count,visual_stale_s`。
  - 新增脚本：`summarize_w10_results.py`、`gate_w10_check.py`。
- 小批实验（`4序列×2模态×3次`）结果：
  - A组（`alpha_NIS`）：A0/A1/A2 均未达到主效应阈值，最佳 A0 仍 `focus_rpe_p95_improve=-9.53%`。
  - C组（`zeta+gate`，固定 A0）：C0/C1/C2 全部 `FAIL`，共同特征是 `committed_drop` 过高（35%~53%）并引发 `RPE` 恶化。
  - 最小修正回合 R1（`zeta-only`）：恢复提交率（`committed_drop=-1.16%`）且 NIS 改善，但 `RPE` 仍超阈，strict Gate-W10 仍 `FAIL`。
- 结论：
  - W9-W10 代码和诊断链均生效，`alpha_nis_sat_rate=0`、`radar_starved=0`。
  - 但在当前公式与数据下，strict Gate-W10 主失败项稳定为 `focus_rpe_p95_improve<10%`，阶段状态保持 `BLOCKED`，不得标记 `DONE`。

### 2026-05-30（v2.0）
- 执行 W9-W10 根因修复（P0-P4）：
  - `RRxIONode`：Contrib3 机制重构为“硬拒绝兜底 + 软惩罚主导”；`alpha_NIS` 改为 `[alpha_min,alpha_max]`；`zeta_RV` 改为以 `1` 为中心的双向有界缩放。
  - `summarize_w10_results.py`：修正 `quality_reject_count` 统计口径；新增机制健康统计列。
  - `gate_w10_check.py`：新增 `health_pass/strict_pass` 双判定并保留 strict 作为最终 `pass`。
  - `evaluate_iros_datasets.py` 与统一 YAML：新增 `alpha_min/zeta_min/zeta_span/gate_hard_qr_min/gate_soft_k_*` 参数透传与 manifest 追踪。
- 完整小批收敛回合：
  - R0：默认参数（FAIL）
  - R1：soft gate 三档（FAIL，最佳 `soft_hi`）
  - R2：zeta 三档（FAIL，最佳 `zeta_c3`）
  - R3：alpha_NIS 三档（FAIL，最佳 `alpha_a0`）
- 关键事实：
  - `health_gate` 全通过（`radar_starved=0`，拒绝率/硬拒绝可追踪）。
  - `strict_gate` 仍失败，主失败项收敛为 `focus_rpe_p95_improve<10%` 与 `RPE degrade>5%`。
  - 阶段状态保持 `W9-W10=BLOCKED`，不进入 W11+。

### 2026-05-31（v2.1）
- 执行 W10 visual-only 根因修复与九数据集验证：
  - `evaluate_iros_datasets.py` / `summarize_w10_results.py` / `gate_w10_check.py`：新增 `--modalities`，visual-only Gate 显式拒绝 thermal 混入。
  - `RRxIONode`：修正 `quality_soft_scale` 为“雷达退化主导 + 双退化额外惩罚”；`zeta_RV` 改为以 `zeta_dr_ref/zeta_dv_ref` 为中心；`alpha_NIS` 未提交分支回归中性 `1.0` 并修正饱和计数。
  - `dvc_rrxio_unified_params.yaml`：固化当前 visual-only 阻塞最优基线参数 `alpha_min=1.0,zeta_min=1.0,gate_soft_scale_max=1.4,zeta_dr_ref=0.60,zeta_dv_ref=0.60`。
- 实验：
  - 完成 V1/V2/V3/V4 九数据集 visual-only 矩阵与 V5 硬拒绝探针。
  - 当前最优：`dvc_w10_visual_v4_selective_inflation`，`health_pass=true`，`strict_pass=false`。
- 结论：
  - V4 改善 `ATE`、`NIS` 与 runtime，但未改善 visual RPE95 长尾；W10 仍保持 `BLOCKED`，不得标记 `DONE`。

### 2026-05-31（v2.2）
- 原始基线复核与 NIS 口径修正：
  - 脚本：`gate_w10_check.py` 新增外部原始基线输入与 `nis_gate_mode=band`；`summarize_w10_results.py` 支持 W1 旧诊断的 `use_radar_update` committed proxy。
  - 配置：删除废弃 top-level scheduler 回压参数，统一由 `dvc_rrxio/scheduler/*` 读取。
  - 实验：V4 相对原始 W1 visual 基线改善 ATE/RPE 与总体 runtime，但 RPE95 变差 `2.32%`；分组 NIS 出现“多数偏保守、`mocap_difficult` 偏过自信”的不一致。
  - 结论：W10 继续 `BLOCKED`；下一轮以分组 NIS band、RPE95 spike 对齐和局部 runtime 长尾为主，不再用“NIS越低越好”的目标函数。

### 2026-05-31（v2.3）
- 执行 W10 visual-only “原始 W1 基线闭环”修正与验证：
  - `RRxIONode`：稀疏低退化 base 协方差回退新增当前帧 `candidate_nis` 约束，避免 `mocap_difficult` 中“几何看似低退化但创新很大”的帧被误判为安全帧。
  - `evaluate_iros_datasets.py` 与统一 YAML：新增并固化 `sparse_low_degradation_candidate_nis_max=4.0`；`gate_soft_k_r=0.0`，使软惩罚默认只在视觉/雷达双退化时发挥作用。
  - `gate_w10_check.py`：修正 legacy 调度模式下 `dvc_sched_diag` 缺失被误判为失败的问题。
- 实验结论：
  - V15 全量 visual-only：`9序列×visual×3次×2模式` 完成。
  - 结果目录：`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_visual_v15_cross_soft_only_9x3`
  - Gate：`health_pass=true`，`strict_pass=false`。
  - 真实失败项：`ATE degrade=+16.41%`、`RPE95 improve=-1.45%`、`indoor_floor/visual RPE95 degrade=+7.09%`、NIS 分组不一致。
- 负结果复核：
  - V16 温和 NIS band 小批无法修复 NIS 与 RPE95 的耦合问题，且显著伤害 `outdoor_street`，不作为后续默认路径。
- 风险状态：
  - W10 继续 `BLOCKED`。当前证据表明问题不再是“拒绝率/日志/调度”层面，而是雷达速度观测本体与状态预测之间的一致性问题。
- 下一最小修正方向：
  - 基于 `analyze_w10_spikes.py` 对 `indoor_floor/mocap_easy/mocap_difficult/outdoor_campus` 做逐帧误差对齐。
  - 优先检查 REVE 速度残差、雷达速度偏置、时间戳偏移、雷达到体坐标外参方向，而非继续扩大后端协方差参数搜索。
