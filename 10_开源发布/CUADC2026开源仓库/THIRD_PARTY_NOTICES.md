# THIRD_PARTY_NOTICES —— 第三方材料边界声明

本仓库只包含**自研内容**。以下是随仓分发的第三方构件、不随仓分发但有依赖关系的内容，以及各自的许可与获取方式。

## 1. 随仓分发的第三方构件

| 内容 | 位置 | 来源与许可 | 说明 |
|---|---|---|---|
| ArduPilot 固件二进制（.apj ×3） | `flight_control/firmware/` | ArduPilot，GPLv3 | CUAV V6X 目标构建（官方 4.6.3 与 4.7-beta 自构建、SIH 台架专用版）。源码：https://github.com/ArduPilot/ardupilot ；对应参数见 `flight_control/params/`，SIH 台架固件的构建校验值见同目录 SIH固件_SHA256SUMS.txt |
| SIH 仿真固件校验清单 | `flight_control/firmware/SIH固件_SHA256SUMS.txt` | 同上 | SIH（Sim In Hardware）为 ArduPilot 官方仿真机制 |

## 2. 不随仓分发的依赖（复现需自行获取）

| 内容 | 本队使用版本 | 许可 | 获取方式 |
|---|---|---|---|
| YOLOv8 预训练权重 `yolov8n.pt` | Ultralytics 8.x | AGPL-3.0 | https://github.com/ultralytics/ultralytics （本仓库 `perception/training/` 的训练脚本按 COCO 预训练直训路线引用它，二进制不分发） |
| MobileSAM 预训练权重 `mobile_sam.pt` | Meta FAIR | Apache-2.0 | https://github.com/ChaoningZhang/MobileSAM （白筒采集标注工具的辅助分割用） |
| ArduPilot 源码 | 4.7-beta（4.6.3 备份） | GPLv3 | https://github.com/ArduPilot/ardupilot |
| ROS 2 | Humble（Ubuntu 22.04） | Apache-2.0 | https://docs.ros.org |
| Gazebo | Harmonic（配套 jetson 端为兼容版本） | Apache-2.0 | https://gazebosim.org |
| CUAV 设备文档 | NEO 3 Pro / V6X / LGC 等 | 厂商版权 | https://doc.cuav.net （本仓库 `flight_control/devices/` 与 GPS 方案册为**自撰**参数记录，非文档拷贝） |
| 训练数据集（实拍/合成） | v2~v3.3 台账见 `perception/training/deliver_v2/dataset_version.md` 等 | 自有，未随仓分发 | 数据集体量大且含场地实拍，仅随仓发布构建/审计脚本与版本台账；如需数据请联系我们 |

## 3. Gazebo 模型资产

`simulation/gazebo_env/cuadc_rescue_sim/` 的 worlds/SDF 引用的 `models/`（含比赛附件贴图与第三方 mesh，
约 22MB）**不随仓分发**（贴图源自大赛附件，组委会版权）。仓库内提供了 `scripts/make_zd550_meshes.py` 等
自研网格生成脚本与 `docs/SETUP.md` 说明按需重建；`Frame ZD550.stl`（机架参考模型）随仓分发。

## 4. 致谢（思路借鉴，未搬运内容）

- [Accelerate11/2026CUADC](https://github.com/Accelerate11/2026CUADC)（哈工大）：教学文档组织与「留给后人的话」经验传承形式；
- [Bob12315/cuadc2026](https://github.com/Bob12315/cuadc2026)（西安石油）：文档按受众三入口、AI 协作契约实践；
- [livepoorloser/cuadc（cuadc-demo-2026-08 快照）](https://github.com/livepoorloser/cuadc)：THIRD_PARTY_NOTICES 规范。

以上借鉴限于**组织形式与理念**，本仓库全部文字与代码为独立撰写，未复制上述仓库内容。
各文档中引用其公开结论处均已在行文内注明。
