# docs —— 文档总入口

本目录是全部文档的家，按**受众**分五档。原文档产自队内仓库（学科模块制），开源时按
[仓库根 README](../README.md) 的分类方案重新归位；队内旧路径与新位置的对照见文末映射表。

## 按你是谁选入口

| 你是谁 | 路线 |
|---|---|
| 🚁 **新人/想系统学** | [getting_started/00_总览与学习路线.md](getting_started/00_总览与学习路线.md) → [getting_started/01_预备知识.md](getting_started/01_预备知识.md) → [manuals/](manuals/) 逐卷推进（遇报错查 [lessons/11_错误与经验总结.md](lessons/11_错误与经验总结.md)） |
| 🔧 **开发者/要复现** | [design/](design/) 看设计（总体方案=SSOT 是唯一权威）→ 对应 manuals 卷动手 → 各工程目录 README |
| 💣 **复盘者/其他队伍** | [lessons/错误与教训实录.md](lessons/错误与教训实录.md)（对话级错误挖掘）→ [lessons/经验与方法总结.md](lessons/经验与方法总结.md)（方法论）→ [lessons/11_错误与经验总结.md](lessons/11_错误与经验总结.md)（按模块速查表）→ [journal/](journal/) 看任意一天的完整过程 |
| 🤖 **AI 协作者** | [ai_collaboration/AGENTS.md](ai_collaboration/AGENTS.md)（本队与 AI 结对开发的 7 条硬规则，接手前必读） |

## 文档清单

```text
docs/
├── getting_started/
│   ├── 00_总览与学习路线.md          13 卷说明书的总路线图
│   ├── 01_预备知识.md                名词与概念入门（原「10_预备知识」）
│   └── ../复现指导_零基础搭建.md     ★ 零基础复现指导（14 章 425KB，手把手+验收+真实坑）
├── design/                           设计文档（怎么想出来的）
│   ├── 总体方案与执行计划.md        ★ SSOT：方案唯一权威版本全文（含开源版头注）
│   ├── 视觉算法设计.md              危化判读/白筒/H 圆三任务算法设计
│   ├── 接口契约.md                  视觉↔状态机↔侦察接口契约 v1.3
│   ├── 时间同步设计.md              取帧时刻戳 + odom 插值设计
│   ├── 源码导航.md                  onboard/ 三包的 file:line 级导航
│   ├── 外参测量清单_相机与投放口.md
│   └── GPS方案/                     单 NEO3 / 单 CUAV 9Ps / NEO3+MT01P 三册（含全量参数）
├── manuals/                          成套说明书（怎么一步步做）
│   ├── README.md                    13 卷入口
│   ├── 01_环境搭建.md ~ 09_算法与系统原理.md, 12_记录模板.md
│   └── RTK-01-R_接线针脚图.svg/png   自绘接线图（04 册配套）
├── lessons/                          ★ 教训专卷（付出了什么学费）
│   ├── 错误与教训实录.md            新撰：从 164 个 AI 协作会话+日志+提交中挖掘的全部错误
│   ├── 经验与教训总集.md            ★ 新撰：全部经验的穷尽总集（十大方法论+分模块细则+工具清单+给后人 40 条）
│   ├── 经验与方法总结.md            快速版方法论（总集的精简前身）
│   └── 11_错误与经验总结.md         原「全库错误总编」：按模块速查的坑+对策表（§0 家规必读）
├── journal/                          过程记录（每天发生了什么）
│   ├── 工作日志总编.md              2026-09-05 ~ 10-09 备赛全程，105 个日期小节
│   └── README.md                    日志制度说明
└── ai_collaboration/
    └── AGENTS.md                    AI 协作契约：SSOT 唯一/日志单文档制/只读区/白名单入库/
                                     结构变更双同步/飞控与接线安全/含 CR 文件读写禁令
```

## 队内旧路径 → 本仓库新位置（目录级映射）

旧文档正文里提到的队内路径按此表换算（逐文件级全表见开源发布目录的《完整映射表》）：

| 队内旧路径 | 本仓库位置 |
|---|---|
| `总体方案与执行计划.md`（仓根） | `docs/design/总体方案与执行计划.md` |
| `01_视觉感知/视觉算法设计.md` | `docs/design/视觉算法设计.md` |
| `01_视觉感知/数据与训练/**` | `perception/training/**` |
| `02_飞控与硬件/*.param`、`右区.plan` | `flight_control/params/`、`flight_control/mission/` |
| `02_飞控与硬件/参数备份/**` | `flight_control/params_backup/**` |
| `02_飞控与硬件/设备参数/**` | `flight_control/devices/**` |
| `02_飞控与硬件/firmware/**` | `flight_control/firmware/**` |
| `02_飞控与硬件/gps/**` | `docs/design/GPS方案/**` |
| `02_飞控与硬件/调试工具/**` | `ground_tools/ops_toolkit/**` |
| `02_飞控与硬件/飞行日志/`（架次台账） | `ground_tools/log_analysis/` |
| `03_机载软件/**`（三包/scripts/systemd） | `onboard/**` |
| `03_机载软件/接口契约.md` 等 4 篇设计文档 | `docs/design/` |
| `04_仿真/SIH台架/**` | `simulation/sih_bench/**` |
| `04_仿真/仿真环境/**` | `simulation/gazebo_env/**` |
| `05_操作与比赛/**` | `operations/**` |
| `06_使用说明书/00、10 册` | `docs/getting_started/` |
| `06_使用说明书/11 册` | `docs/lessons/11_错误与经验总结.md` |
| `06_使用说明书/其余册` | `docs/manuals/` |
| `07_工作日志/**` | `docs/journal/**` |
| `AGENTS.md`（仓根） | `docs/ai_collaboration/AGENTS.md` |
