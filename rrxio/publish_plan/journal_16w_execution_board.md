# 《仪器仪表学报》16周可发表路径执行看板（DVC-RRxIO）

## 1. 目标与边界
- 目标：在当前 `rrxio -> rovio -> reve` 工程基础上形成可复现、可解释、可投稿的完整证据链。
- 投稿目标：`中国仪器仪表学报`。
- 人力假设：`1人主导 + AI辅助`。
- 数据策略：公开数据集为主，必要时补充少量自采场景。
- 工程约束：低侵入优先，不改状态维度，不重写视觉前端，不切换滑窗优化。

## 2. 当前工程基线（已完成）
- 参数链统一：`rrxio_evaluate_rosbag.launch` 已统一使用 `bag_duration`。
- REVE 稳定性门限：`max_r_cond` 已实际参与 LSQ 条件数门控，并有非法阈值保护。
- 现有评估链路可用：
  - 启动入口：`rrxio/launch/rrxio_evaluate_rosbag.launch`
  - 批量评估脚本：`rrxio/python/evaluate_iros_datasets.py`
  - 轨迹评估脚本：`thirdparty/rpg_trajectory_evaluation/scripts/analyze_trajectories.py`
- 参数输入已统一：
  - 统一配置文件：`rrxio/launch/configs/dvc_rrxio_unified_params.yaml`
  - 运行时按 run 生成配置：`<results_root>/dvc_param_configs/dvc_params_<run_id>.yaml`
  - launch 通过 `dvc_unified_config` 单入口加载参数。
- W1-W4 实施资产已落地（待门禁验收）：
  - `rrxio/python/freeze_baseline_snapshot.py`
  - `rrxio/python/summarize_baseline_results.py`
  - `rrxio/python/gate_w2_check.py`
  - `rrxio/python/gate_w4_check.py`

## 3. 16周任务拆解（执行版）
| 周次 | 状态 | 阶段目标 | 工程任务（当前仓库入口） | 实验任务 | 阶段产出 | 验收门槛 |
|---|---|---|---|---|---|---|
| W1-W2 | DONE | 基线冻结与复现 | 固化评估链路；运行 `freeze_baseline_snapshot.py` 输出快照 | 视觉/热成像基线各重复3次 | `baseline_v1/` 结果包+快照 | 关键指标波动在可接受范围 |
| W3-W4 | DONE | 诊断链最小闭环 | 在 `RRxIONode` 与 REVE 链路增补 `cond/inlier_ratio/traceR/minEigR/update_used` 输出 | 校验诊断值随场景变化趋势 | `dvc_diag.csv` + 诊断图 | 每帧可追踪、无缺列 |
| W5-W6 | DONE | Contribution-1（已完成） | 接入 `alpha_R` 各向同性重标定（保持低侵入） + NIS 诊断闭环 + 调度确定性修正 | 全量对比 `base/fixed/alpha_r`（9序列×2模态×3次） | `w6_metrics.csv`+`w6_summary.md`+对比图+Gate报告 | Gate-W6 与 Scheduler 严格门禁双 PASS |
| W7-W8 | TODO | Contribution-2 | 接入 `S_k` 方向性塑形 + SPD保护 | 几何退化专项实验 | 退化证据图 | 退化段突跳减少 |
| W9-W10 | TODO | Contribution-3 | 接入 `alpha_NIS` 与 `zeta_RV`，明确更新接受/拒绝日志 | 双退化实验（视觉差+雷达差） | 完整 DVC 主链 | `alpha_NIS` 不长期饱和 |
| W11-W12 | TODO | 完整消融矩阵 | 统一实验配置与导出格式 | 全基线+全消融批量跑 | 消融总表+图集 | 每个主张有对应证据 |
| W13-W14 | TODO | 鲁棒性与失败模式 | 失败路径注入与可观测性增强 | 同步偏差/外参偏差/低纹理等实验 | 失败模式章节素材 | 失败路径可复现并可解释 |
| W15 | TODO | 论文初稿与图表定稿 | 统一符号、术语、图表风格，整理方法与实验章节 | 内部审阅一轮 | 初稿v1+补充材料草案 | 结构完整、证据闭环 |
| W16 | TODO | 投稿包封版 | 终稿、附录、复现说明、参数表、脚本索引 | 终审清单逐项核验 | 投稿包v1 | 满足期刊格式与技术完整性 |

## 4. 每阶段门禁（必须通过再进下一阶段）
- Gate-W2：`baseline_v1` 可复现，快照齐全，运行清单完整。
- Gate-W4：诊断日志字段完整，能解释雷达更新行为。
- Gate-W6：`alpha_R` 有一致性收益且不破坏基线稳定性。
- Gate-W8：`S_k` 在退化场景提升可复现。
- Gate-W10：完整主链在双退化场景无明显错误更新放大。
- Gate-W12：消融矩阵覆盖主张，结果可复跑。
- Gate-W14：失败模式“现象-原因-修正”闭环成立。
- Gate-W16：稿件、图表、复现资产全部可交付。

## 4.1 W1-W4 严格门禁执行状态（2026-05-16）
| Gate | 当前状态 | 自动检查脚本 | 必要证据 | 结果 |
|---|---|---|---|---|
| Gate-W2 | PASS | `rrxio/python/gate_w2_check.py` | `snapshots/*/{metadata.json,file_sha256.csv,README.txt}` + `run_manifest.csv` + `baseline_v1_metrics.csv` + `baseline_v1_summary.md` | PASS |
| Gate-W4 | PASS | `rrxio/python/gate_w4_check.py` | `dvc_diag_*.csv`（按 run_id）+ `dvc_diag_schema.csv` + `gate_w4_report.json` | PASS |

本次门禁通过批次说明：
- 数据根目录：`/home/yyy/datasets/irs_rtvi_datasets_2021`
- 结果目录：`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/baseline_v1`
- 运行配置：`features=25`、`n_trials=3`、`bag_duration=60`

门禁标注规则：
- 仅当脚本退出码为 `0` 且报告 `pass=true` 才可把对应阶段标记为 `DONE`。
- 任一门禁失败时，阶段状态必须保持 `TODO/IN_PROGRESS/BLOCKED`，不得写 `DONE`。

## 4.2 W5-W6 严格门禁执行状态（2026-05-19）
| Gate | 当前状态 | 自动检查脚本 | 必要证据 | 结果 |
|---|---|---|---|---|
| Gate-W6 | PASS | `rrxio/python/gate_w6_check.py` | `run_manifest.csv` + `w6_metrics.csv` + `w6_summary.md` + `w6_compare_metrics.png` + `w6_compare_nis.png` + `gate_w6_report.json` | PASS |

本次门禁通过批次说明：
- 数据根目录：`/home/yyy/datasets/irs_rtvi_datasets_2021`
- 结果目录：`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w6_alpha_r`
- 运行配置：`features=25`、`bag_duration=60`、`cov_mode in {base,fixed,alpha_r}`、`n_trials=3`
- 全量覆盖：`9 序列 × 2 模态 × 3 次重复 × 3 模式 = 162 runs`（`SUCCESS=162`）

Gate-W6 关键指标（alpha_r vs base）：
- NIS 超限率：`5.7756% -> 2.2623%`（相对下降 `60.83%`，绝对下降 `3.513pp`）
- ATE 中位数：`0.113182 -> 0.101191`（未恶化，改善 `10.59%`）
- RPE 中位数：`0.074655 -> 0.074027`（未恶化，改善 `0.84%`）
- 运行时中位数：`10.111s -> 10.063s`（增量 `-0.48%`）

## 4.3 W6 指标修复回合（2026-05-22，A/B/C 小批筛选）
门禁脚本：`rrxio/python/gate_scheduler_check.py`（绝对时延门禁：`median<=30ms,p95<=50ms,max<=80ms`）

| 候选 | backpressure(high/low) | 结果目录 | Gate | 关键失败项 |
|---|---|---|---|---|
| A | `128/64` | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w6_tune_a_smallbatch` | FAIL | latency: `54.35/77.63/147.96ms`; `RPE degrade=19.25%` |
| B | `192/96` | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w6_tune_b_smallbatch` | FAIL | latency: `61.18/83.15/123.94ms`; `RPE degrade=14.58%`; repeatability 1 组超阈 |
| C | `256/128` | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w6_tune_c_smallbatch` | FAIL | latency: `79.50/107.79/177.15ms`; `RPE degrade=13.86%`; repeatability 2 组超阈 |

补充结论：
- 三档均满足：`radar_starved=0`、`runtime_ratio` 均优于 `1.15` 上限、`NIS` 与 `committed_drop` 未触发硬门禁。
- 当前阻塞主因仍是 `event_stage2` 雷达提交时延长尾与 thermal 组 `RPE/ATE` 退化耦合。
- 按门禁规则，W5-W6 当前状态保持 `IN_PROGRESS/BLOCKED`，不得标注 `DONE`。

## 4.4 W6 代码级确定性修正回合（2026-05-22，长尾压降专项）
本回合改动：
- 调度层：增加 `SchedulerSnapshot` 与 `radar_needs_imu_catchup` 判定，loader 回压改为 `Hard Queue Guard + IMU Guard + Catchup Bypass`。
- worker 等待：`popNextReadyEvent()` 在无可执行事件时改为 10ms 量子等待（保持 predicate 唤醒与时序规则）。
- 追加最小修正：REVE RANSAC 抽样改为按雷达数据构建的确定性 seed，消除跨运行随机漂移。

小批门禁结果（`4序列×2模态×legacy/event_stage2×3次`）：
| 批次 | backpressure(high/low) | 结果目录 | Gate | 关键结论 |
|---|---|---|---|---|
| fixdet-1 | `64/32` | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w6_fixdet_64_32_smallbatch` | FAIL | 时延已过阈值（`20.06/29.36/53.27ms`），失败项转为 `RPE degrade=13.39%` + repeatability |
| fixdet-2 | `48/24` | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w6_fixdet_48_24_smallbatch` | FAIL | 时延继续下降（`16.68/26.32/57.70ms`），`RPE degrade=12.03%`，repeatability 仍超阈 |
| fixdet-3 | `32/16` | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w6_fixdet_32_16_smallbatch` | FAIL | 时延最低（`12.53/22.31/48.26ms`）但 `RPE degrade` 反弹到 `19.11%` |
| fixdet-4 | `48/24` + deterministic RANSAC | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w6_fixdet_seed_48_24_smallbatch` | FAIL | repeatability 全部恢复到阈值内，仅剩 `RPE degrade=13.57%` 单项失败 |

当前结论：
- `event_stage2` 雷达提交长尾问题已被显著压降并稳定通过时延门禁。
- 当前唯一高优先阻塞项：`RPE degrade` 仍高于 `5%` 硬阈值。
- （该回合结束时）阶段状态保持 `IN_PROGRESS/BLOCKED`，不得标注 `DONE`。

## 4.5 W6 收敛回合（2026-05-22，确定性修正闭环完成）
门禁脚本：`rrxio/python/gate_scheduler_check.py`（绝对时延门禁：`median<=30ms,p95<=50ms,max<=80ms`）

最终收敛参数（同门禁配置）：
- `cov_mode=alpha_r`
- `scheduler_mode=event_stage2`
- `dvc_scheduler_backpressure_high/low = 48/24`
- `dvc_scheduler_imu_fast_path = 0`（关闭 IMU fast path）
- 其余保持：`radar_imu_window_s=0.02`、`bag_duration=60`、`features=25`

验证结果：
| 批次 | 结果目录 | 规模 | Gate |
|---|---|---|---|
| 小批确认 | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w6_fixdet_fast0_48_24_smallbatch` | `4序列×2模态×legacy/event_stage2×3次` | PASS |
| 全量确认 | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w6_fixdet_fast0_48_24_full` | `9序列×2模态×legacy/event_stage2×3次` | PASS |

全量门禁关键指标（event_stage2 vs legacy）：
- 雷达提交时延（ms）：`median=9.07, p95=16.95, max=41.74`（全部满足绝对门禁）
- ATE degrade：`+1.23%`（<=5%）
- RPE degrade：`-2.91%`（改善）
- runtime increase：`-1.67%`（<=15%）
- NIS 绝对增量：`+0.367pp`（<=1pp）
- radar committed drop：`0.436%`（<=5%）
- repeatability：`PASS`（CV 全部远低于阈值）
- `radar_starved=0`（满足）

收敛结论：
- W6 阶段“时延长尾 + ATE/RPE + repeatability”已在同一硬门禁下同时闭环。
- W5-W6 阶段状态可标记为 `DONE`，允许推进 W7-W8。

## 5. 目录与命名规范（统一结果资产）
建议根目录：`<dataset_root>/results/dvc_rrxio_publish/`

```text
results/dvc_rrxio_publish/
  baseline_v1/
  dvc_w4_diag/
  dvc_w6_alpha_r/
  dvc_w8_s_k/
  dvc_w10_full/
  dvc_w12_ablation/
  dvc_w14_failure_modes/
  paper_assets/
  snapshots/
```

统一命名：`<date>_<stage>_<dataset>_<modality>_<config_tag>`

## 6. 必需接口与日志字段
- 配置命名空间：`dvc_rrxio.*`
- 诊断文件：`dvc_diag.csv`
- W3-W4 最小必需列：
  `timestamp,cond,inlier_ratio,trace_R_used,minEig_R_used,use_radar_update,radar_scan_callback_count,row_id`
- W5-W6 追加列：
  `cov_mode,d_r,alpha_r,n_targets,n_inliers,nis_vel,nis_valid,nis_exceed_95,radar_update_committed`
- 可选列：
  `runtime_reve_ms,runtime_backend_ms`

## 7. 每周执行动作（固定流程）
1. 运行基线快照脚本，记录配置指纹。
2. 更新 `run_manifest.csv`（本周所有实验条目）。
3. 执行实验并落盘到本周目录。
4. 生成图表与结果摘要。
5. 运行阶段门禁清单。
6. 更新三份文档：
   - `RRxIO_核心链路深度审计报告.md`
   - `rrxio/DVC-RRxIO_方案评估与落地修订报告.md`
   - 本看板（状态列与风险列）

## 8. 风险与回退策略
- 风险A：调度耦合导致雷达更新时效不足。
  - 策略：先做日志证据，不直接改调度模型；必要时开分支实验。
- 风险B：参数过多导致不可辨识。
  - 策略：`alpha_R -> S_k -> alpha_NIS/zeta_RV` 严格分阶段。
- 风险C：公开数据对某主张敏感性不足。
  - 策略：补充少量可控退化场景，优先验证一致性指标而非只看ATE。

## 9. 第一周立即执行清单
- [ ] 运行 `rrxio/python/freeze_baseline_snapshot.py --output_dir <...>/snapshots --tag W1_baseline`
- [x] 运行 `rrxio/python/freeze_baseline_snapshot.py --output_dir <...>/snapshots --tag W1_baseline`
- [x] 评估链路脚本支持 `run_manifest.csv`、`run_id` 与 `diag_file` 落盘
- [x] 新增汇总脚本：`rrxio/python/summarize_baseline_results.py`
- [x] 新增门禁脚本：`rrxio/python/gate_w2_check.py` 与 `rrxio/python/gate_w4_check.py`
- [x] 跑视觉/热成像基线各3次并生成 `baseline_v1_summary.md`
- [x] 执行 Gate-W2 / Gate-W4 并根据报告更新状态
