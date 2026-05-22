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
