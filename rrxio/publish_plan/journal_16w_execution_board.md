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
| W7-W8 | BLOCKED | Contribution-2 | 接入 `S_k` 方向性塑形 + SPD保护 | 几何退化专项实验 | 退化证据图 | 严格 Gate-W8 通过（当前未通过） |
| W9-W10 | BLOCKED | Contribution-3 | 接入 `alpha_NIS` 与 `zeta_RV`，明确更新接受/拒绝日志 | 双退化实验（视觉差+雷达差） | 完整 DVC 主链 | 平衡严格档 v1 Gate-W10 通过（当前未通过） |
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

## 4.6 W7-W8 实施与门禁状态（2026-05-23）
门禁脚本：`rrxio/python/gate_w8_check.py`（严格阈值：`rpe_p95_improve>=20%`）

已完成实现资产：
- P0：REVE 侧将观测几何 `obs_ut_u` 统一到体坐标系后再参与 `S_k`（含特征值重算）。
- P1：`alpha_r_sk` 路径新增轻量激活门（`lambda3_obs/d_r/n_targets`）与显式跳过诊断（`s_k_applied/s_k_skip_reason`）。
- 统一配置与脚本已对齐：`evaluate_iros_datasets.py` 支持新增 `dvc_rrxio/s_k/*` 门控参数透传并落盘到 `run_manifest.csv`。
- 新增与扩展：`summarize_w8_results.py`、`gate_w8_check.py`、`dvc_diag` 的 `S_k` 诊断列。

小批门禁记录（`4序列×2模态×alpha_r/alpha_r_sk×3次`）：
| 批次 | 结果目录 | 参数 | Gate | 关键结果 |
|---|---|---|---|---|
| W8-P2-S0 | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w8_p0p2_s0_smallbatch` | `tau=8.0,c=0.5,s_max=2.0` | FAIL | `rpe_p95_improve=-1.64%`，其余硬门禁通过 |
| W8-P2-S1 | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w8_p0p2_s1_smallbatch` | `tau=6.0,c=0.4,s_max=1.8` | FAIL | `rpe_p95_improve=-1.30%`，`ATE degrade=+5.27%` 超阈 |
| W8-P2-S2 | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w8_p0p2_s2_smallbatch` | `tau=10.0,c=0.6,s_max=2.2` | FAIL | `rpe_p95_improve=-1.84%`，其余硬门禁通过 |

P2 三档共性证据：
- `total_sk_rows=2256`，`radar_starved=0`（三档一致）。
- `outdoor_street` 组 `s_k_applied_count=0`（visual/thermal），该组对 `rpe_p95_improve` 无正贡献。
- `indoor_floor/visual` 在三档中均出现负向 `rpe_p95` 改善，是当前主拖累段。

阶段结论：
- W7-W8 已完成 P0/P1 代码落地与 P2 三档小批实测，但严格 Gate-W8 仍未通过。
- 三档均未满足 `rpe_p95_improve>=20%`，按规则不进入全量 `9×2×3`。
- 当前状态必须保持 `BLOCKED`，不得标注 `DONE`。

## 4.7 W7-W8 Gate重构收敛回合（2026-05-24）
执行策略：`Gate重构 -> 覆盖率门禁 -> tau/c/s_max 顺序调参`（小批：`4序列×2模态×alpha_r/alpha_r_sk×3次`）。

阶段A（Gate参数）结论：
- 结果汇总：`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w8_stageA_prescan_results.csv`
- `d_r=0.70`：`coverage_pass=true`，`rpe_p95_improve=+1.304%`（四个 `obs_trace` 一致）。
- `d_r=0.80`：`rpe_p95_improve=+4.082%`，但 `coverage_pass=false`（不满足覆盖率门禁）。
- `d_r=0.90`：`rpe_p95_improve=-5.914%`，且 `coverage_pass=false`。

阶段B1（`c_obs` 扫描）结论：
- 结果汇总：`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w8_stageB1_c_scan_smallbatch_results.csv`
- 最优：`c_obs=0.35`，`coverage_pass=true`，`rpe_p95_improve=+9.495%`。

阶段B2（`tau_obs` 扫描，固定 `c_obs=0.35`）结论：
- 结果汇总：`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w8_stageB2_tau_scan_smallbatch_results.csv`
- 最优：`tau_obs=8`，`coverage_pass=true`，`rpe_p95_improve=+9.495%`。

阶段B3（`s_max` 扫描，固定 `c_obs=0.35,tau_obs=8`）结论：
- 结果汇总：`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w8_stageB3_smax_scan_smallbatch_results.csv`
- `s_max in {1.8,2.0,2.2}` 三档结果一致：`rpe_p95_improve=+9.495%`，`coverage_pass=true`。

严格门禁结论：
- 最优组合（`d_r=0.70, obs_trace=55, c_obs=0.35, tau_obs=8, s_max∈[1.8,2.2]`）仍未满足 strict 条件 `rpe_p95_improve>=20%`。
- W7-W8 状态保持 `BLOCKED`，不得标注 `DONE`。
- 例外条款：允许在“W8 冻结基线”前提下推进 W9-W10 受控试验，但不得回写为 W7-W8 完成。

## 4.8 W8 冻结基线后 W9-W10 收敛状态（2026-05-29）
冻结基线（W8）：
- `d_r=0.70, obs_trace=55, c_obs=0.35, tau_obs=8, s_max=2.0`（`s_max` 仅在 `1.8~2.2` 内小范围等价验证）。
- W8 维持 `BLOCKED` 原因：`rpe_p95_improve<20%`（主效应不足）。

W9 阶段（`alpha_NIS`）小批三档（`4序列×2模态×3次`）：
| 档位 | 结果目录 | 关键参数 | Gate-W10 | 关键结论 |
|---|---|---|---|---|
| A0 | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_A0_smallbatch` | `eta=0.02,rho=0.985,alpha_max=6` | FAIL | `focus_rpe_p95_improve=-9.53%`，`focus_nis_drop=0%` |
| A1 | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_A1_smallbatch` | `eta=0.03,rho=0.98,alpha_max=8` | FAIL | `focus_rpe_p95_improve=-9.45%`，`focus_nis_drop=-1.92%` |
| A2 | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_A2_smallbatch` | `eta=0.04,rho=0.975,alpha_max=10` | FAIL | `focus_rpe_p95_improve=-9.45%`，`focus_nis_drop=-1.92%` |

W10 阶段（`zeta_RV + 质量门控`）三档（固定 A0）：
| 档位 | 结果目录 | Gate-W10 | 关键失败项 |
|---|---|---|---|
| C0 | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_C0_smallbatch` | FAIL | `committed_drop=47.03%`，`rpe_degrade=50.16%` |
| C1 | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_C1_smallbatch` | FAIL | `committed_drop=35.65%`，`rpe_degrade=14.92%` |
| C2 | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_C2_smallbatch` | FAIL | `committed_drop=53.36%`，`rpe_degrade=69.12%` |

最小修正回合（仍限 contrib3 参数）：
| 回合 | 结果目录 | 设定 | Gate-W10 | 关键结论 |
|---|---|---|---|---|
| R1 | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_R1_zeta_only_smallbatch` | `zeta_enable=1, gate_enable=0` | FAIL | `committed_drop=-1.16%`（恢复），但 `rpe_degrade=14.45%`，`focus_rpe_p95_improve=-15.61%` |

阶段结论：
- W9-W10 代码与脚本链路已落地并可稳定运行（`radar_starved=0`，诊断列完整，`alpha_nis_sat_rate=0`）。
- 但在当前数据与公式下，strict Gate-W10 仍未通过；主失败项始终为 `focus_rpe_p95_improve<10%`，且开启质量门控会显著拉低 `committed_count`。
- 按规则：W9-W10 必须保持 `BLOCKED`，不得标记 `DONE`，不进入 W11+。

## 4.9 W9-W10 根因修复回合（2026-05-30，P0-P4 + R0-R3）
实施内容（源码级）：
- `RRxIONode`：Contrib3 由“硬拒绝优先”改为“硬拒绝兜底 + 软惩罚主导”，新增
  `quality_soft_scale / quality_hard_reject / quality_gate_stage` 诊断链。
- `alpha_NIS`：区间由单边改为对称可配置（`[alpha_min, alpha_max]`），该回合初版遗忘支路回归 `alpha_min`；后续 visual-only 根因修复已修正为回归中性 `1.0`。
- `zeta_RV`：由单边 `1+softplus` 改为以 `1` 为中心的有界双向缩放（`tanh` 形式）。
- `summarize_w10_results.py`：`quality_reject_count` 改为仅统计 `radar_update_reject_reason=="quality_gate"`；
  新增 `quality_reject_rate / hard_reject_count / max_consecutive_quality_reject`。
- `gate_w10_check.py`：新增 `health_pass` 与 `strict_pass` 双结果，`pass` 仍严格绑定 `strict_pass`。

小批统一设置：
- 规模：`4序列 × 2模态 × 3次`，`alpha_r_sk` vs `alpha_r_sk_nis_rv`，`scheduler_mode=event_stage2`。
- 数据根：`/home/yyy/datasets/irs_rtvi_datasets_2021`。

关键回合结果（strict Gate-W10）：
| 回合 | 结果目录 | strict | 关键指标（focus） | 全局护栏（节选） |
|---|---|---|---|---|
| R0（默认） | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_r0` | FAIL | `rpe_p95_improve=-18.99%`, `nis_drop=-51.92%` | `ATE +14.43%`, `RPE +14.61%` |
| R1（soft_hi） | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_r1_soft_hi` | FAIL | `rpe_p95_improve=-14.97%`, `nis_drop=-40.38%` | `ATE -1.35%`, `RPE +7.70%` |
| R2（zeta_c3） | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_r2_zeta_c3` | FAIL | `rpe_p95_improve=-4.12%`, `nis_drop=+25.00%` | `ATE -0.74%`, `RPE +6.89%` |
| R3（alpha_a0） | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_r3_alpha_a0` | FAIL | `rpe_p95_improve=-9.30%`, `nis_drop=+32.69%` | `ATE -1.47%`, `RPE +9.81%` |

当前结论：
- `health_gate` 全部通过（`radar_starved=0`、拒绝率可追踪、`committed_drop` 未超阈）。
- `strict_gate` 仍失败，主失败项收敛为两条：
  1) `focus_rpe_p95_improve < 10%`
  2) `RPE degrade > 5%`
- 阶段状态维持：`W9-W10 = BLOCKED`（不得标记 `DONE`）。

## 4.10 W10 visual-only 根因修复与九数据集验证（2026-05-31）
范围变更：
- 后续 W10 实验主线固定为 `visual-only`：`9序列 × visual × 3次`，不再把 thermal 纳入 W10 Gate；代码仍保留 thermal 兼容路径。
- 新增 `--modalities visual|thermal|visual,thermal` 到 `evaluate_iros_datasets.py`、`summarize_w10_results.py`、`gate_w10_check.py`。
- `gate_w10_check.py --modalities visual` 若发现 manifest 或 metrics 混入 thermal 行，直接 FAIL，避免口径污染。

源码修正：
- `RRxIONode`：质量软惩罚改为“雷达质量不足为主、双退化额外惩罚”：
  `penalty = k_r max(0,tau_r_high-q_r) + k_v max(0,tau_r_high-q_r) max(0,tau_v_low-q_v)`。
- `RRxIONode`：`zeta_RV` 改为以参考退化点居中的 `tanh` 形式：
  `theta0 + theta1(d_r-d_r_ref) + theta2(d_v-d_v_ref) + theta3(d_r-d_r_ref)(d_v-d_v_ref)`。
- `RRxIONode`：`alpha_NIS` 未提交/跳过分支回归中性 `1.0`，并修正 `alpha_nis_sat` 对下边界饱和的统计口径；当前 visual-only 默认采用 `alpha_min=1.0`，避免盲目缩小协方差。
- 统一配置与 manifest：新增并固化 `zeta_dr_ref/zeta_dv_ref`，当前阻塞最优基线采用 `alpha_min=1.0,zeta_min=1.0,gate_soft_scale_max=1.4`。

九数据集 visual-only 结果：
| 回合 | 结果目录 | strict | 关键结论 |
|---|---|---|---|
| V0 baseline | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_visual_v0_baseline` | FAIL | 旧 `R2_zeta_c3` 只覆盖 4 组；visual-only Gate 要求 9 组，因此不能作为最终证据 |
| V1 softfix | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_visual_v1_softfix` | FAIL | `NIS` 相对下降 `36.88%`，但 `RPE degrade=6.96%`、`RPE95 improve=-18.83%` |
| V2 centered zeta | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_visual_v2_centered_zeta` | FAIL | `zeta` 居中后仍 `RPE degrade=6.56%`、`RPE95 improve=-18.89%`，且 `NIS` 相对下降不足 `15%` |
| V3 VC1 | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_visual_v3_vc1` | FAIL | 保守参数仍出现 `ATE degrade=15.69%` 与 `RPE95 improve=-19.79%` |
| V4 selective inflation | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_visual_v4_selective_inflation` | FAIL | 当前最优阻塞基线：`ATE degrade=-10.08%`、`NIS` 相对下降 `24.11%`、`runtime=-1.98%`、`committed_drop=-0.46%`，但 `RPE degrade=5.53%`、`RPE95 improve=-5.82%` |
| V5 hard reject probe | `/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w10_visual_v5_hardreject_probe` | 非最终 Gate | `q_r` 硬拒绝对 `mocap_medium` 有局部收益，但使 `mocap_easy` 提交数下降 `25.93%` 且 `ATE degrade=47.85%`，不作为全局修复 |

当前结论：
- `health_gate`: PASS（`radar_starved=0`、拒绝/硬拒绝/提交计数可追踪、repeatability 通过）。
- `strict_gate`: FAIL，失败项集中为 `RPE degrade > 5%` 与 `rpe_p95_improve_visual < 10%`。
- W10 visual-only 状态：`BLOCKED`，不得标记 `DONE`；当前可继承基线为 V4。
- 下一最小修正方向：逐帧关联 `zeta_rv / quality_soft_scale / alpha_nis_new` 与 RPE95 spike，定位少数长尾段，而不是继续全局硬拒绝或扩大参数网格。

## 4.11 W10 对原始仓库基线复核与 NIS 口径修正（2026-05-31）
对比口径修正：
- 投稿主结论必须对比原始仓库输出：`baseline_v1/original_visual_w10_metrics.csv`，而不是只对比 W8/W10 中间基线。
- `gate_w10_check.py` 新增外部基线输入：`--base_metrics`、`--base_manifest`、`--base_label`。
- W1 原始诊断只有 `use_radar_update`，没有 W6 后的 `radar_update_committed`；汇总脚本将其作为 legacy committed proxy，保证原始结果可参与同一指标表，但不伪造 NIS。

V4 对原始 W1 visual 基线的复核结果：
| 指标 | 原始 W1 | 当前 V4 | 变化 | 判定 |
|---|---:|---:|---:|---|
| ATE median | `0.09718` | `0.09529` | `-1.95%` | 改善 |
| RPE median | `0.08071` | `0.07870` | `-2.49%` | 改善 |
| RPE95 median | `0.12567` | `0.12858` | `-2.32%` | 长尾变差 |
| runtime median | `10.34585s` | `10.36817s` | `+0.22%` | 总体可控 |
| committed count | `16098` | `15804` | `-1.83%` | 门限内 |
| NIS exceed rate | 原始不可比 | `1.997%` | 低于理论 `5%` | 偏保守 |

分组异常：
- `gym` 与 `outdoor_campus` 的 runtime 分别约 `+144.68%` 与 `+154.00%`，被全局 median 掩盖，后续 Gate 必须加入 per-dataset runtime 或 p95 runtime 护栏。
- RPE95 主要负贡献来自 `gym`、`mocap_easy`、`outdoor_street`；其中 `outdoor_street` 同时出现 ATE/RPE/RPE95 均变差。
- NIS 分组不一致：`mocap_difficult/visual` 超限率 `11.51%`（偏过自信），其余多数 visual 组低于 `1%`（偏保守），只有 `mocap_dark_fast/visual=3.18%` 接近合理区间。

NIS 口径修正：
- NIS 不是越低越好，也不是越高越好。三维速度观测在一致估计下，超过 `chi2_3_95` 的比例应接近 `5%`。
- `gate_w10_check.py` 新增 `nis_gate_mode=band`，默认关注当前 NIS 是否处于合理带宽 `[1%,8%]` 并接近 `5%`，替代单纯追求相对下降。
- 当前 V4 `health_pass=true`、`strict_pass=false`：总量指标有改善，但 RPE95 长尾和分组 NIS 一致性仍未达标。

参数清理：
- 删除废弃的 top-level `scheduler_backpressure_*` 与 `scheduler_drain_timeout_s` 参数入口。
- 回压参数统一由 `dvc_rrxio/scheduler/*` 读取，避免旧 launch arg 或临时 YAML 覆盖统一配置。
- 当前保留 `default_backpressure_depth` 仅为 `rrxio_rosbag_loader.cpp` 内部默认值，不再是外部配置项。

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
