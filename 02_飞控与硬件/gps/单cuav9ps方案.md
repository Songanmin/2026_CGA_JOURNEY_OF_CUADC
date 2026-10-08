# 单 CUAV C-RTK 9Ps 方案 — 换装参数定案（2026-10-08）

> **现行 GPS 方案**：单 CUAV C-RTK 9Ps（ZED-F9P 四星多频 + IST8310 罗盘）替代 NEO3，走 GPS1 座串口，完全沿用
> [单neo3方案.md](单neo3方案.md) 确立的「单GPS@GPS1 串口」配方骨架；MT-01P 定高链不在本次换装范围（其参数注解见
> [单neo3+MT01P方案.md](单neo3+MT01P方案.md)）。
> 定位方案取舍以 [总体方案与执行计划.md](../../总体方案与执行计划.md)（SSOT §3.2 定位行）与最新
> [工作日志](../../07_工作日志/工作日志总编.md) 为准；本文只放参数与执行细节。
> 底稿：`../参数备份/V3.param`（4.7 系当前实态全量，出处见 `../参数备份/changes.md` 台账）。
> 写入集：[`../参数备份/param_write_cuav9ps_20261008.param`](../参数备份/param_write_cuav9ps_20261008.param)。

## 1. 换装内容与接线

| 项 | 定案 |
|---|---|
| 模块 | CUAV C-RTK 9Ps（u-blox **ZED-F9P** 四星多频 GNSS + **IST8310** 罗盘，双 UART、无 CAN；官方口径 ArduPilot 即插即用） |
| 接口 | 模块 **UART1 → 飞控 GPS1 座（SERIAL3）**，与 NEO3 同座同线序，拔 NEO3 插 9Ps 即可 |
| 串口配方 | `SERIAL3_PROTOCOL=5`（GPS）+ `SERIAL3_BAUD=115`（115200）**不变**——`GPS_AUTO_CONFIG=1` 下 ArduPilot 会自动扫波特率列表探测模块并按配置重下发（F9P 出厂波特率与此不同也能握手），配置落模块由 `GPS_SAVE_CFG=2` 保存 |
| 驱动 | `GPS1_TYPE=1`（u-blox 串口驱动，覆盖 F9P）；`GPS2_TYPE=0` 第二路关闭（单 GPS 定案不变） |
| 输出速率 | `GPS1_RATE_MS=200`（5Hz，现行配方不变） |
| 与 NEO3 的实质差异 | 多频多系统（GPS L1/L2 + 北斗 B1I/B2I + GALILEO + GLONASS），抗多径与定位精度预期优于 NEO3（09-28 复飞基线：21 星峰 23 / HDOP 0.72）；F9P 本身是 RTK 移动端芯片，未来若恢复基站注 RTCM（`GPS_INJECT_TO=127` 已就绪）可上 RTK——现行比赛口径仍为纯 GPS+视觉，不依赖 RTK 成立 |

## 2. 安装几何与参数含义（本换装的真实增量）

物理安装：**飞控上方 5cm、左侧 5cm、与飞控同向放置**（10-08 用户更正：同向，非本册初稿的「反向 180°」）。

ArduPilot 机体系为 FRD（X 前 / Y 右 / Z 下，单位米），据此换算：

| 参数 | NEO3 时代现值 | 写入值 | 含义 |
|---|---|---|---|
| `GPS1_POS_X` | 0 | **0** | 天线相位中心与飞控同前后位（若有前后向安装偏移需实测补写） |
| `GPS1_POS_Y` | 0 | **-0.05** | 左侧 5cm（Y 右为正，左取负） |
| `GPS1_POS_Z` | 0 | **-0.05** | 上方 5cm（Z 下为正，上取负） |
| `COMPASS_ORIENT` | 2（YAW_90，NEO3 内部朝向配方） | **0（ROTATION_NONE）** | 外置罗盘与飞控同向放置 = 无旋转修正；ROTATION 枚举：**NONE=0** / YAW_45=1 / YAW_90=2 / YAW_135=3 / YAW_180=4 |

说明与防坑：

- **GPS1_POS 是天线相位中心位置**，影响 GPS 位置/速度解算的杆臂补偿（急机动与 blended 定位精度相关），装好即应写准，数值就是 5cm 量级、按安装实测微调。
- **COMPASS_ORIENT 的枚举序防呆（必读）**：ORIENT 按罗盘枚举实例号生效。底稿 V3.param 中外置 IST8310 在 **instance1**（`COMPASS_DEV_ID=658953`=PRIO1，板载 RM3100=instance2 且 `COMPASS_USE2=0` 停用），故写 `COMPASS_ORIENT`。NEO3 与 9Ps 的罗盘同为 IST8310、同经 GPS 座 I2C，预期枚举 ID 不变；但本机历史上有枚举序翻转先例（09-28 快照板载在 instance1）——**导入写入集前必须先读 `COMPASS_DEV_ID`/`COMPASS_DEV_ID2` 确认外置在 instance1**，若在 instance2 则该条改写为 `COMPASS_ORIENT2`。
- **朝向初值按安装声明给，最终以户外验证为准**：CUAV 模块内部罗盘相对外壳可能自带固定旋转（NEO3 同向放置实测需 YAW_90 修正即前科，9Ps 内部朝向官方未标注、不能免检），故 ROTATION_NONE=0 是「与飞控同向放置」的声明初值。六面球面校准会吸收朝向误差、**校准一致性查不出朝向写错**；朝向错误的现场症状 = 罗盘航向与真实航向恒定偏差（90°/180°）+ 飞行中 EV62 磁航向复位频发。验证方法见 §4-④。

## 3. 导入方法（家规口径）

1. 连飞控前断开 Mission Planner（占 COM 口）。
2. `python load_params_fast.py <COM口> ../参数备份/param_write_cuav9ps_20261008.param`——默认两遍导入、每遍后重启（**写参数后等 5 秒再重启**）；本文件 4 项真实增量均为已存在的平铺参数，无懒加载展开，但两遍家规不豁免。
3. 验收：`params-diff`（verify_params）实读对账全一致 + 点名回读本文件全部键值。

## 4. 上线验证清单（按序执行，全部通过才算换装完成）

| # | 步骤 | 判据 |
|---|---|---|
| ① | 上电读枚举：`COMPASS_DEV_ID`/`DEV_ID2`、`COMPASS_PRIO1_ID` | 外置 658953（预期）在 instance1 且 PRIO1；若 ID 变化或次序翻转，按 §2 调整 ORIENT 实例号再导入 |
| ② | `check_gps.py` | 识别 u-blox F9P；窗外/户外 3D Fix 星数 ≥ NEO3 基线 21 星、HDOP ≤0.72（多频应更好；室内无 Fix 属正常） |
| ③ | 六面罗盘校准：`ops.py compass`（fc_compass_calib.py，10-08 新入库） | 模长一致性差 ≤150mG、范围 80~850mG（工具自动判）；校准前确认翻转半径 1m 内无铁磁物、拆桨 |
| ④ | **户外航向比对**（朝向终判）：慢速原地旋转/直线推行，对比罗盘航向 vs GPS 航迹角（或已知方位） | 偏差 <10° 且无常值差；若恒差 ≈90°/180°，按步进修 `COMPASS_ORIENT`（0 的近邻候选：2/6=±90°，4=180°）→ 重校准 → 复测 |
| ⑤ | `GPS1_POS_X/Y/Z` 点名回读 = 0 / -0.05 / -0.05 | 与实物安装一致（前后向若有偏移补实测值） |
| ⑥ | PreArm 检查清单过一遍（重点磁场项：204 号曾报 `mag field xy diff 125>100`） | 磁场警告不再现或显著收敛 |

## 5. 与现行定案的衔接（本换装不动的东西）

- **罗盘承载定案不变**：外置（GPS 模块自带）主用 + 板载 RM3100 停用（`COMPASS_USE2=0`）；换装只是外置罗盘换了载体，停用板载的理由（远离机内干扰源）反而更强——9Ps 抬高 5cm。
- **EKF 源配置不变**：`EK3_SRC1_POSXY/VELXY/VELZ=3`、`POSZ=1`（气压计主高度）、`YAW=1`（罗盘）。
- **MT-01P 定高链不受影响**：当前实态 `RNGFND1_TYPE=0` 挂账关闭（V3.param），恢复步骤照 单neo3+MT01P方案.md §2.2，与本换装无耦合。
- **旧罗盘校准值全部作废**：`COMPASS_OFS*/DIA*/ODI*/SCALE*`（外置实例）换模组+换位置后必须重校（§4-③），不要沿用底稿数值导入。
- **电机干扰补偿挂账**：`COMPASS_MOTCT=2` 及 `COMPASS_MOT/MOT2` 现值基于 NEO3 旧安装位学得，换装后已失真——下次带电校准窗口重学（compassmot，MP 或另立脚本）；重学前若 PreArm 磁场项异常，可临时 `COMPASS_MOTCT=0` 关补偿。
- `COMPASS_AUTO_ROT=2`/`COMPASS_AUTODEC=1`/`COMPASS_DEC` 保持底稿现值不动；DEC 期望在校准与户外验证后复核。

## 6. 回退预案

NEO3 模块留作备份：拔 9Ps 插回 NEO3，重导入 单neo3配方（`COMPASS_ORIENT=2` + NEO3 时代 `GPS1_POS` 归零 + 旧校准值需重校）即回退；应急切换卡 `05_操作与比赛/备用GPS_NEO3应急切换.md` 口径仍适用（座位与串口配方相同）。
