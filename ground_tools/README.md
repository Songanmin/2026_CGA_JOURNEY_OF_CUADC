# ground_tools · 地面工具链

地面侧（Windows 调试机 / Jetson 机载电脑）自研工具两族：

| 子目录 | 内容 |
|---|---|
| [ops_toolkit/](ops_toolkit/README.md) | `ops.py` 统一入口调试全家桶（~40 个脚本）：连飞控 / 查 GPS / 预解锁诊断 / 电机·舵机测试 / RTK 配置 / 参数两遍导入与对账（verify_params）/ 日志回拉 / 一键任务 / 虚拟 GPS 台架工具 / 救援航线生成器等，README 内有全命令表 |
| [log_analysis/](log_analysis/README.md) | dataflash 飞行日志取证与分析工具（`_dfparse.py` 自研解析器、`_analyze.py`、`_a232.py`、`_flightwin.py` 等，含一次真实摔机事故的 forensic 解析脚本），`README.md` 即**架次台账**（每次飞行的日志号与结论索引） |

## 使用前必读

- 安全纪律（拆桨测试、先断 Mission Planner、参数两遍导入、写后等 5 秒再重启）见 [ops_toolkit/README.md](ops_toolkit/README.md) 与 [../docs/lessons/11_错误与经验总结.md](../docs/lessons/11_错误与经验总结.md) §0 家规；
- 日志判读方法论见 [../docs/lessons/经验与方法总结.md](../docs/lessons/经验与方法总结.md)（分层隔离排障法、判别式探针法两节即由这套工具的使用实践沉淀）。
