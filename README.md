# CUADC 2026 · 多旋翼无人机侦察与救援 —— 全流程备赛开源

2026 中国大学生飞行器设计创新大赛（CUADC）**多旋翼无人机侦察与救援**项目的完整备赛资料开源：
机载软件（ROS 2 三包）、视觉感知与训练流水线、飞控参数与调试工具链、Gazebo/SITL 仿真与回归闸门、
比赛操作卡、13 卷成套说明书，以及我们最看重的——**错误教训实录与经验方法论**。

> 我们把「踩过的坑」当作与代码同等重要的产出。
> 想快速了解这个项目付出过哪些学费，直接去 [docs/lessons/错误与教训实录.md](docs/lessons/错误与教训实录.md)。

## 系统总览

| 分系统 | 方案 |
|---|---|
| 机架/动力 | ZD550 四旋翼，6S，好盈 OPTO 电调（无 BEC，独立 UBEC 6.0V 供舵机母线） |
| 飞控 | CUAV V6X + ArduPilot 4.7（beta），GUIDED 全自主航线 |
| 定位 | 单 GPS（CUAV NEO 3 Pro / CUAV 9Ps）+ 激光测距 MT-01P 定高；RTK 方案有专册设计 |
| 机载电脑 | NVIDIA Jetson（Ubuntu 22.04 + ROS 2），USB 相机（MJPG 透传 30fps） |
| 视觉 | YOLOv8 检测（危化品十类判读）+ YOLOv8-seg（白筒/H 圆），TensorRT 部署，~56FPS 上机 |
| 任务状态机 | ROS 2 `cuadc_mission`：起飞→悬停识别→搜索→视觉对准→双投放→侦察→返航降落，20Hz FSM + 九门控投放 |
| 仿真回归 | Gazebo + ArduPilot SITL 全任务联跑，「闸门」回归制度（每版代码必过闸才上真机） |
| 地面工具 | `ops.py` 统一入口调试全家桶（连飞控/查 GPS/预解锁诊断/舵机电机测试/日志回拉） |

**项目状态（2026-10 口径）**：仿真全任务 119s（双投 11.1+6.4cm 落 A 区、拍照 6/6）；真机自主首飞达成（10-07）；真机视觉投放验证进行中。完整过程见 [docs/journal/](docs/journal/)。

## 目录地图

```text
CUADC2026开源仓库/
├── docs/                  ★ 文档总入口（按受众分层，先看这里的 README）
│   ├── getting_started/   新人路径：学习路线 + 预备知识
│   ├── design/            设计文档：总体方案(SSOT 全文)/视觉算法/接口契约/时间同步/GPS 方案
│   ├── manuals/           成套说明书 01~09、12 册（环境→装机→飞控→外设→视觉→机载→仿真→比赛→原理）
│   ├── lessons/           ★ 教训专卷：错误与教训实录 / 经验与方法总结 / 全库错误总编
│   ├── journal/           工作日志总编（2026-09-05 ~ 10-09 备赛全程，105 个日期小节）
│   └── ai_collaboration/  AI 协作契约（本队与 AI 结对开发的工作规则）
├── perception/            视觉感知工程：数据构建/审计/采集标注合成工具/训练流水线/版本台账
├── onboard/               机载 ROS 2 三包：cuadc_mission / cuadc_perception / cuadc_interfaces + systemd
├── flight_control/        飞控：主参数 + 26 份参数快照与台账 + 设备参数册 + 固件(SHA256) + 任务航线
├── simulation/            仿真：Gazebo 救援仿真包 + SIH 台架 + SITL 联跑与回归闸门脚本
├── ground_tools/          地面工具链：ops.py 调试全家桶 + dataflash 取证解析工具
├── operations/            比赛操作：上场流程卡/上电手动飞行/应急切换/连通度验证/台架 AB 复测卡
├── LICENSE                MIT（代码）· 文档 CC BY 4.0
└── THIRD_PARTY_NOTICES.md 第三方材料边界（固件/预训练权重/数据）
```

各工程目录均有自己的 `README.md` 讲「这是什么、怎么跑」；文档内部的队内旧路径（如 `07_工作日志/...`）
按 [docs/README.md](docs/README.md) 的映射表对应新位置。

## 三种入口

- **🚁 新人学习**：**首选 [docs/复现指导_零基础搭建.md](docs/复现指导_零基础搭建.md)（零基础复现指导，14 章 425KB，每步带验收标准与真实坑标注）**；或传统路线 [docs/getting_started/00_总览与学习路线.md](docs/getting_started/00_总览与学习路线.md) → 预备知识 → 13 册说明书逐卷推进。
- **🔧 复现系统**：[docs/manuals/01_环境搭建.md](docs/manuals/01_环境搭建.md) 起步 → `onboard/` 编译（各包 README）→ `flight_control/` 参数导入（注意「两遍导入」纪律）→ `simulation/` 先过 SITL → 真机前必读 `operations/`。
- **💣 只想看教训**：[docs/lessons/错误与教训实录.md](docs/lessons/错误与教训实录.md)（我们犯过的错）→ [docs/lessons/经验与方法总结.md](docs/lessons/经验与方法总结.md)（沉淀出的方法论）。

## 快速上手（clone 之后）

1. 读 [docs/README.md](docs/README.md) 了解文档布局；
2. 想跑仿真：`simulation/gazebo_env/`（模型资产获取方式见 THIRD_PARTY_NOTICES）+ `simulation/gazebo_env/jetson/操作手册.md`；
3. 想看任务状态机：`onboard/cuadc_mission/src/mission_node.cpp`（配套设计文档 `docs/design/源码导航.md`）；
4. 想连真机：先读 [docs/lessons/11_错误与经验总结.md](docs/lessons/11_错误与经验总结.md) §0「家规」——**任何新线缆上电前必过万用表**。

## 安全声明

本项目包含真实无人机飞行与硬件调试内容。所有外场/真机操作请遵守当地法规，
拆桨测试、安全员在场、保留人工接管手段——这些不是建议，是我们用摔机换来的家规（见教训实录）。
代码与文档按「现状」提供，复现者需自行承担飞行安全责任。

## License 与致谢

- 代码：[MIT](LICENSE)；文档：CC BY 4.0（署名-本队）。第三方材料边界见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
- 备赛过程中深受同赛开源队伍启发，特别致谢：[哈工大 Accelerate11/2026CUADC](https://github.com/Accelerate11/2026CUADC)（Re0 教学与「留给后人的话」的直接示范）、西安石油 Bob12315/cuadc2026（文档三入口与 AI 协作契约）、cuadc-demo-2026-08（THIRD_PARTY_NOTICES 规范）。我们「教训专卷一级化」的做法就是对这一社区传统的回应。
- 依赖的开源生态：ArduPilot、ROS 2、Gazebo、Ultralytics YOLOv8、pymavlink/mavros。
