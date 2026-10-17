#!/usr/bin/env bash
# CUADC 感知自启入口（systemd 拉起，P0.3）
# 职责：ROS2 环境 → 工作空间 → 起真侦察判读节点（加载 engine + 黑帧预热后待命）
# 注意：engine 缺失/SHA 不符时节点拒启，systemd 会按 Restart=on-failure 重试——
#       这是故意的 fail-closed（宁可反复失败留痕，不许带错权重上岗）。
# UDP-only Fast DDS profile：/dev/shm 共享内存段被同机其他 DDS 进程搅乱时仍能
# 通信（09-12 实测：与仿真栈同机时 SHM 匹配失败、节点不可见，UDP-only 修复）。
source /opt/ros/humble/setup.bash
source /home/nvidia/cuadc_ws/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/cuadc_udp_profile.xml
# 判读→数传填单桥（09-26 拍板并入自启；真三跳带业务验收全绿）：
# 后台守护循环自愈，随本服务 cgroup 生死；mavros 未起时桥空转无害，
# 判读主链死亡时桥 fail-silent（设计如此）。日志窗口级低频追加，无需轮转。
# 相机 + 桶检测（视觉起飞门禁/搜索/对准/投放依赖；10-07 补部署缺口——
# 此前冷启动缺这两节点，视觉起飞门禁 0 帧静默拦截起飞）
nohup bash -c 'source /opt/ros/humble/setup.bash; source /home/nvidia/cuadc_ws/install/setup.bash; for i in $(seq 1 30); do [ -e /dev/video0 ] && break; sleep 2; done; python3 /home/nvidia/camera_node.py' > /tmp/camera_node.log 2>&1 < /dev/null &
nohup bash -c 'source /opt/ros/humble/setup.bash; source /home/nvidia/cuadc_ws/install/setup.bash; sleep 5; ros2 launch cuadc_perception bucket_perception.launch.py' > /tmp/bucket_perc.log 2>&1 < /dev/null &
nohup bash -c 'while true; do ros2 launch cuadc_perception recon_gcs_bridge.launch.py >> /tmp/cuadc_recon_bridge.log 2>&1; sleep 2; done' >/dev/null 2>&1 &
exec ros2 run cuadc_perception hazard_recon_node --ros-args \
  --params-file /home/nvidia/cuadc_ws/install/cuadc_perception/share/cuadc_perception/config/hazard_recon_params.yaml
