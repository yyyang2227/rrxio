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

## 3. 16周任务拆解（执行版）
| 周次 | 状态 | 阶段目标 | 工程任务（当前仓库入口） | 实验任务 | 阶段产出 | 验收门槛 |
|---|---|---|---|---|---|---|
| W1-W2 | TODO | 基线冻结与复现 | 固化评估链路；运行 `freeze_baseline_snapshot.py` 输出快照 | 视觉/热成像基线各重复3次 | `baseline_v1/` 结果包+快照 | 关键指标波动在可接受范围 |
| W3-W4 | TODO | 诊断链最小闭环 | 在 `RRxIONode` 与 REVE 链路增补 `cond/inlier_ratio/traceR/minEigR/update_used` 输出 | 校验诊断值随场景变化趋势 | `dvc_diag.csv` + 诊断图 | 每帧可追踪、无缺列 |
| W5-W6 | TODO | Contribution-1 | 接入 `alpha_R` 各向同性重标定（保持低侵入） | 对比 原RRxIO/固定膨胀/alpha_R | 对比图+结果表 | NIS改善且主精度不恶化 |
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
- 建议固定列：
  `timestamp,d_R,d_V,alpha_R,s1,s2,s3,alpha_NIS,nis,cond,inlier_ratio,trace_R_used,use_radar_update`

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
- [ ] 复制 `templates/run_manifest_template.csv` 为 `run_manifest.csv` 并填入本周任务
- [ ] 跑视觉/热成像基线各3次
- [ ] 汇总 ATE/RPE/RMSE 与运行时，生成 `baseline_v1_summary.md`
- [ ] 更新两份报告的更新日志
