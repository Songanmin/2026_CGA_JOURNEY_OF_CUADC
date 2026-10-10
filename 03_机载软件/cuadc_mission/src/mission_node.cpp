// =============================================================================
// CUADC 2026 多旋翼侦察与救援 · 任务状态机节点（M1 骨架）
//
// 设计依据（唯一权威方案）：01_设计/总体方案与执行计划.md（SSOT）
//   - §5.1 全生命周期状态（本文件状态枚举与其逐一对应，含 DONE 终态与
//     PILOT_OVERRIDE / ABORT 两条旁路）
//   - §5.4 参数基线（config/mission_params.yaml 默认值的出处）
//   - §10.2 坑清单（任务原点 / ENU 航向转换 / 锁航向 / odom 断流 ...）
// 参考实现：查阅区/08_参考/code/2026code/状态机/cuadc_full_mission_node_3_v2_public.cpp
//   （按其 README 要求"先理解逻辑、再按 ROS 2 重新实现"，本文件为重写版骨架）
//
// M1 范围（能飞）：
//   [已实现] 准备五连 + LOCK_FRAME 独立态 + CommandTOL 起飞 + 预设航线
//            （时间插值平滑）+ 侦察段（拍照请求/确认握手）+ 返航
//            + 降落确认（≤0.30m + 速度门限 + 稳定 1.5s）+ 独立 DISARM 自动上锁
// M3 范围（能投，本次接入）：
//   [已实现] 感知接入（PoseArray 薄契约 + 哨兵校验 + P0.4 插值换算世界系）
//            + SEARCH 目标锁定五步（drop_logic::BucketMap）
//            + ALIGN 两段对准（粗 0.15/精 0.08 + 一次重捕获 + 超时弃桶）
//            + RELEASE 九门控（含反盲投 target_age）+ 单发舵机（DO_SET_SERVO）
//   [安全] enable_release_output=false 为干跑模式（只打日志不发舵机）；
//          m1_no_vision_mode=true 时 SEARCH/ALIGN/RELEASE 全部旁路（无视觉演练）
//   [未开始] safety_monitor 独立进程、原地小幅搜索降级（M4）
//
// 2026-09-12 算法与路线完善（"横着搜"修复的后续，详见当日工作日志）：
//   - 航线构建收敛 route_logic.hpp 纯函数（罗盘→ENU 换算/蛇形生成，单测守门，
//     防出生朝向与场地方向错 90° 这类 bug 复发）；搜索带 4→6 消覆盖盲区；
//     侦察带改沿 Y 长轴扫（带位沿 X 均布，覆盖余量 0.11→0.86m）
//   - SEARCH 首段转场提速（transit 速度）+ 超时提前降级（search_degrade_timeout_s）
//   - 视觉起飞门禁（vision_takeoff_min_frames，SSOT §5.4 起飞前 ≥3 帧有效）
//   - ALIGN/RELEASE 视觉速度伺服（参数化默认关，SSOT §5.2-209，失新回退位置保持）
//   - sim_release_bridge 仿真投放判分桥（SITL 联动 virtual_drop_judge_node）
//
// 2026-10-08 悬停识别定案（"悬停优先+蛇形兜底"，详见当日工作日志）：
//   - SEARCH 首段转场改指投放区中心识别点（search_hover_x/y_m），定点悬停建图，
//     锁定即进 ALIGN（省蛇形 ~19s）；窗口（search_hover_timeout_s，到位起算）
//     到期未锁转蛇形兜底——锁定五步/降级预案/60s 提前降级全部不变
//   - 依据：单点悬停 2.5m 覆盖半宽 ≈1.32m 盖不住 2~5m 桶距，纯单点方案否决
//   - m1_no_vision_mode 恒跳过悬停段（无锁定可能，不白烧窗口）
//
// 2026-10-08 启动门禁放宽（用户定案"3D DGPS 即可启动"）：
//   - 门禁语义显式化：任意有效 fix 即锁框架（mavros fix_type>2 → 3D/DGPS/RTK
//     一律 STATUS_FIX），首个 fix 打 INFO、持续 NO_FIX 打节流 WARN 留痕
//   - 配套载体修正（3D DGPS 启动后真能起飞）：TAKEOFF「已在空中」守卫改 odom
//     差（GNSS 分钟级漂移 ±5m 地面误触拒飞）；0.9× 完成判据增 odom 或分支
//     （单点 GPS 垂直噪声 ±2~4m 防 Takeoff timeout）+ 命令未发出不判完成
//
// 2026-10-08 落地判定改三级设计（用户拍板，平移哈工大 hgd_cudac 8_20 版；
//   实飞版 v16 同款 landed_state 主判据，10-08 晚考古证实）：
//   - 主判据=FC 自身 landed_state（EXTENDED_SYS_STATE，落地检测器内部量，与
//     GNSS 垂直精度无关）；明确非地面状态一律判否——绝不允许高度读数把空中
//     报告翻转成 DISARM；兜底=odom 相对高硬钳 ≤min(0.30,0.10)+速度门限
//   - 解除原红线②：GNSS 0.30m 判据单点解下不可用 → 落地判定与 GNSS 解耦，
//     基本 GPS 下自动上锁（桨停转着陆分）恢复可用
//
// 2026-10-08 借鉴哈工大 hgd_cudac ws_offboard 2026-8-20 版（详见当日工作
// 日志与 SSOT §5.3 参考实现节）：
//   - LOCK_FRAME 增位置静止判据：水平速度 ≤lock_position_max_speed_m_s(0.15)
//     持续与航向同窗才放行——防「边滑边锁」（其 position_lock_stability_s 同款）
//   - 释放八门控→九门控：增角速度 ≤release_max_ang_rate_deg_s(5°/s，odom 三轴
//     模长)——倾角瞬时值可过零通过而机体正在振荡，速度型门控补该盲区
//   - PILOT_OVERRIDE 后不再向 FC 发任何新指令（stow 重试门闸）——借鉴其
//     command_authority「故障不得恢复软件控制权」语义的单节点等价实现
//
// 2026-10-09 高度基准改定「解锁时定位高度」（用户定案，替代 10-08 上电冻结）：
//   - 任务相对高度 = 实际定位高度 − 解锁时定位高度（解锁上升沿重冻 GNSS 基准）；
//   - 动因：锁框架→解锁的飞手等待期 GNSS 分钟级漂移 ±5m（210 号实测）使
//     0.9× 起飞判据永远够不着（TAKEOFF 超时无法起飞）；起飞命令本身是 AGL
//     相对爬升不受影响，仅判据基准移动；
//   - on_armed_edge 沿检测已有首帧保护（节点晚启不误采空中高度）。
//
// 安全铁律（SSOT §7）：
//   1. 每一版必须先过 SITL 再上真机；真机首测拆桨；
//   2. 飞手切走 GUIDED = 人工接管，本节点立即停发目标点并退让（绝不抢权）；
//   3. m1_no_vision_mode=true 仅为 M1 无视觉演练模式，M2 接入感知后必须关闭。
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "cuadc_mission/drop_logic.hpp"
#include "cuadc_mission/odom_interp.hpp"
#include "cuadc_mission/route_logic.hpp"

using namespace std::chrono_literals;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr const char * kMissionVersion = "cuadc-m3-2026-09b";
constexpr int kNPayloads = 2;  ///< SSOT：两瓶（1 号筒 + 2 号筒）
/// A1-1：服务 ACK 超时（到点清理客户端 pending；清理≠撤销远端动作，结果=未知）
constexpr double kServiceTimeoutS = 2.0;

/// 三维点（本地 ENU 坐标系，米；z 向上，相对上电原点）
struct Point3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

/// 一段直线航线：按"距离/速度=时长"做时间插值，飞机追插值点而非直冲终点
struct Segment
{
  Point3 start;
  Point3 end;
  rclcpp::Time start_time;
  double duration_s = 1.0;
};

// odom 历史样本 = cuadc_odom::Sample（odom_interp.hpp 纯函数；P0.4 判定核心
// 09-20 自本文件 interpolate_odom() 抽出，时间域等号/大间隔/回跳由
// test_odom_interp 双端锁死；见 10_机载代码/时间同步设计.md）

/// 角度归一化到 (-pi, pi]
double normalize_angle(double x)
{
  return std::atan2(std::sin(x), std::cos(x));
}

/// 角度归一化到 [0, 360)
double normalize_degrees(double x)
{
  x = std::fmod(x, 360.0);
  return x < 0.0 ? x + 360.0 : x;
}

/// 罗盘角差 a-b，归一化到 [-180, 180]
double diff_degrees(double a, double b)
{
  double d = std::fmod(a - b, 360.0);
  if (d > 180.0) {
    d -= 360.0;
  }
  if (d < -180.0) {
    d += 360.0;
  }
  return d;
}

double distance_xy(const Point3 & a, const Point3 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double distance_xyz(const Point3 & a, const Point3 & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

// =============================================================================
// 状态定义（SSOT §5.1 全生命周期）
// =============================================================================
enum class State
{
  // ---- 准备阶段（地面）----
  WAIT_FCU,         ///< 等飞控连接（MAVROS ↔ Pixhawk 握手）
  WAIT_NAV_STABLE,  ///< 等定位 + 罗盘可用
  LOCK_FRAME,       ///< 锁定任务坐标系：航向 2s 内变化 ≤2° + 机体静止（水平
                    ///< ≤0.15m/s 持续同窗）才放行（防罗盘漂移/边滑边锁）
  PRESTREAM,        ///< 在起飞点预发 setpoint 1.5s（切 GUIDED 前必须已有目标流）
  WAIT_GUIDED,      ///< 等飞手切 GUIDED（= 飞手最终确认）
  WAIT_ARM,         ///< 等解锁（默认飞手手动；auto_arm_on_guided 才自动）
  // ---- 任务主线 ----
  TAKEOFF,          ///< CommandTOL 起飞，0.9×目标高后切 setpoint
  SEARCH,           ///< 2.5m 蛇形搜索；M1 演练=飞预设航线，正式=等感知锁定独立筒
  ALIGN,            ///< 粗对准（M3：目标锁定五步之后，fail-closed）
  RELEASE,          ///< 九门控投放 ×2（M3，fail-closed）
  // ---- 侦察段 ----
  RECON_CLIMB,      ///< 爬到侦察高度
  RECON_SURVEY,     ///< 蛇形 6 航点 + 每点拍照请求/确认（1s 超时兜底）
  // ---- 返航与收尾 ----
  RETURN_CLIMB,     ///< 先爬到返航高度，再横越场地
  RETURN_HOME,      ///< 返航到起飞点上方
  LAND,             ///< 降落交还飞控；落地判定 ≤0.30m + 速度门限 + 稳定 1.5s
  DISARM,           ///< 独立态：确认落地后自动上锁（规则 6.1.4 桨停转才有着陆分）
  DONE,             ///< 结束
  // ---- 旁路 ----
  PILOT_OVERRIDE,   ///< 飞手切走 GUIDED：程序立即退让并退出
  ABORT             ///< 致命故障：能降则降，不能则退出
};

std::string state_name(State s)
{
  switch (s) {
    case State::WAIT_FCU: return "WAIT_FCU";
    case State::WAIT_NAV_STABLE: return "WAIT_NAV_STABLE";
    case State::LOCK_FRAME: return "LOCK_FRAME";
    case State::PRESTREAM: return "PRESTREAM";
    case State::WAIT_GUIDED: return "WAIT_GUIDED";
    case State::WAIT_ARM: return "WAIT_ARM";
    case State::TAKEOFF: return "TAKEOFF";
    case State::SEARCH: return "SEARCH";
    case State::ALIGN: return "ALIGN";
    case State::RELEASE: return "RELEASE";
    case State::RECON_CLIMB: return "RECON_CLIMB";
    case State::RECON_SURVEY: return "RECON_SURVEY";
    case State::RETURN_CLIMB: return "RETURN_CLIMB";
    case State::RETURN_HOME: return "RETURN_HOME";
    case State::LAND: return "LAND";
    case State::DISARM: return "DISARM";
    case State::DONE: return "DONE";
    case State::PILOT_OVERRIDE: return "PILOT_OVERRIDE";
    case State::ABORT: return "ABORT";
  }
  return "UNKNOWN";
}

class CuadcMissionNode final : public rclcpp::Node
{
public:
  CuadcMissionNode()
  : Node("cuadc_mission")
  {
    declare_parameters();
    load_parameters();
    log_effective_config();

    // ---- 订阅：飞控状态 / 定位 / 罗盘 / 姿态 / 速度 / 侦察确认 / 感知检测 / 落地检测 ----
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", rclcpp::QoS(10).reliable(),
      std::bind(&CuadcMissionNode::state_callback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom", rclcpp::SensorDataQoS(),
      std::bind(&CuadcMissionNode::odom_callback, this, std::placeholders::_1));
    compass_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/mavros/global_position/compass_hdg", rclcpp::SensorDataQoS(),
      std::bind(&CuadcMissionNode::compass_callback, this, std::placeholders::_1));
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      std::bind(&CuadcMissionNode::pose_callback, this, std::placeholders::_1));
    velocity_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/mavros/local_position/velocity_local", rclcpp::SensorDataQoS(),
      std::bind(&CuadcMissionNode::velocity_callback, this, std::placeholders::_1));
    recon_ack_sub_ = create_subscription<std_msgs::msg::UInt32>(
      "/cuadc/recon/capture_done", 10,
      std::bind(&CuadcMissionNode::recon_ack_callback, this, std::placeholders::_1));
    bucket_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/perception/drop_buckets_body", rclcpp::SensorDataQoS(),
      std::bind(&CuadcMissionNode::bucket_callback, this, std::placeholders::_1));
    // GNSS/RTK 高度（NavSatFix.altitude，RTK 模块→FC→mavros）：**实际高度唯一源**
    //（2026-10-08 用户定案：实际高度=RTK 推送高度，基准高度=上电冻结，任务相对
    // 高度=两者之差；FC 高度源同步切 GNSS，EK3_SRC1_POSZ=3，命令/飞行/判读同系）。
    // 启动门禁（2026-10-08 用户定案放宽）：任意有效 fix 即过——mavros 把
    // fix_type>2（3D/3D+DGPS/RTK）统一映射 STATUS_FIX，不要求 RTK Float/Fixed；
    // 完全无 fix 时 ground_amsl_valid_=false → 任务坐标锁定不进行（fail-closed：
    // 室内无任何 fix 不可飞）。历史注：10-06 曾按「param7=MSL」语义修
    // 「场地海拔>3m 静默拒飞」；10-07 查明本固件 param7=相对爬升量，海拔不进命令。
    global_fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/mavros/global_position/global", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
          if (!ground_amsl_valid_) {
            RCLCPP_INFO(get_logger(), "首个有效 GPS fix（3D/DGPS 及以上即过启动门禁）：海拔 %.2fm",
              msg->altitude);
          }
          ground_amsl_m_ = msg->altitude;
          ground_amsl_valid_ = true;
          last_gps_time_ = now();
          have_gps_ = true;
        } else {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
            "NavSatFix NO_FIX 帧（门禁只要 3D/DGPS；持续无 fix=锁不了框架不起飞）");
        }
      });

    // 落地检测（2026-10-08 采纳哈工大 hgd_cudac 8_20 版设计，实飞版 v16 同款）：FC 自报
    // EXTENDED_SYS_STATE.landed_state 是落地判定的权威源——来自飞控落地检测器
    // （垂直速度+油门+EKF 内部量），与 GNSS 垂直精度完全无关，基本 GPS 下照常工作
    extended_state_sub_ = create_subscription<mavros_msgs::msg::ExtendedState>(
      "/mavros/extended_state", rclcpp::SensorDataQoS(),
      [this](const mavros_msgs::msg::ExtendedState::SharedPtr msg) {
        extended_state_ = *msg;
        have_extended_state_ = true;
        last_extended_state_time_ = now();
      });

    // ---- 发布：目标点 / 状态广播 / 侦察拍照请求 ----
    setpoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", 10);
    mission_state_pub_ = create_publisher<std_msgs::msg::String>("/cuadc/mission_state", 10);
    recon_capture_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/cuadc/recon/capture_request", 10);
    buckets_world_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/cuadc/debug/buckets_world", 10);
    velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/mavros/setpoint_velocity/cmd_vel", 10);

    // ---- 服务客户端：全部异步发送 + 节流，绝不阻塞状态机 ----
    takeoff_client_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    land_client_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
    arm_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    command_client_ = create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");
    sim_release_client_ = create_client<std_srvs::srv::Trigger>("/drop_controller/release");

    state_enter_time_ = now();
    mission_start_time_ = now();
    last_request_time_ = now() - rclcpp::Duration::from_seconds(2.0);
    last_odom_time_ = now();
    last_servo_time_ = now() - rclcpp::Duration::from_seconds(2.0);
    last_vision_time_ = now();

    // 状态机心跳：50ms 一拍（20Hz）
    timer_ = create_wall_timer(50ms, std::bind(&CuadcMissionNode::tick, this));

    RCLCPP_INFO(get_logger(), "CUADC mission node ready [%s]", kMissionVersion);
    if (m1_no_vision_mode_) {
      RCLCPP_WARN(
        get_logger(),
        "M1 无视觉演练模式：SEARCH 飞预设航线后跳过对准/投放，直接转侦察段。"
        "正式飞行必须设 m1_no_vision_mode:=false");
    } else {
      RCLCPP_WARN(
        get_logger(),
        "正式模式：感知未接入时 SEARCH 将 fail-and-return（fail-closed）；"
        "enable_release_output=%s", enable_release_output_ ? "true(实弹)" : "false(干跑)");
    }
  }

private:
  // ===========================================================================
  // 参数：全部声明 + 下限保护（SSOT §10.2-6：启动即加载，YAML 与代码不脱节）
  // ===========================================================================
  void declare_parameters()
  {
    declare_parameter<bool>("m1_no_vision_mode", true);
    declare_parameter<bool>("auto_arm_on_guided", false);

    declare_parameter<double>("takeoff_alt_m", 4.0);
    declare_parameter<double>("search_alt_m", 2.0);
    declare_parameter<double>("recon_alt_m", 4.0);
    declare_parameter<double>("return_alt_m", 4.0);

    declare_parameter<double>("transit_speed_m_s", 4.0);
    declare_parameter<double>("search_speed_m_s", 2.0);
    declare_parameter<double>("recon_speed_m_s", 3.0);

    declare_parameter<double>("waypoint_accept_radius_m", 0.40);

    declare_parameter<double>("heading_lock_window_s", 2.0);
    declare_parameter<double>("heading_lock_tol_deg", 2.0);
    declare_parameter<double>("heading_lock_timeout_s", 30.0);
    // LOCK_FRAME 位置静止判据（2026-10-08 借鉴 hgd：防边滑边锁；持续窗=航向同窗）
    declare_parameter<double>("lock_position_max_speed_m_s", 0.15);

    declare_parameter<double>("prestream_hold_s", 1.5);
    declare_parameter<double>("takeoff_timeout_s", 60.0);
    declare_parameter<double>("mission_timeout_s", 240.0);
    declare_parameter<double>("odom_timeout_s", 1.0);
    declare_parameter<double>("gps_timeout_s", 2.0);
    declare_parameter<double>("recon_photo_timeout_s", 1.0);

    declare_parameter<double>("landing_confirm_s", 1.5);
    declare_parameter<double>("landing_alt_gate_m", 0.30);
    declare_parameter<double>("landing_hspeed_gate_m_s", 0.20);
    declare_parameter<double>("landing_vspeed_gate_m_s", 0.15);
    // landed_state 新鲜窗（2026-10-08 哈工大三级设计：过期才退 odom 高度兜底）
    declare_parameter<double>("extended_state_timeout_s", 2.5);

    // 场地几何 = 仿真场地取值（RULE_MAPPING.md：投放区(30,0) 5m×8m；侦察区(55,0)同尺寸），
    // 上真机按赛场实测修订（见 config/mission_params.yaml 同名参数注释）
    declare_parameter<double>("search_x_min_m", 27.5);
    declare_parameter<double>("search_x_max_m", 32.5);
    declare_parameter<double>("search_half_width_m", 4.0);
    declare_parameter<int>("search_lanes", 6);
    // SSOT P4-3：SEARCH 进行这么久仍无 3 筒锁定 → 提前按"2 筒+1 未知"降级
    declare_parameter<double>("search_degrade_timeout_s", 60.0);
    // SEARCH 沉降期（2026-09-12 真视觉排障）：起飞后 EKF 在飞行中对齐罗盘航向
    // （mavros "in-flight yaw alignment complete"），期间喂图的桶世界坐标随 yaw
    // 漂移，会凑出"稳定"的假航迹导致锁错目标。沉降期内只飞不喂图，等航向收敛。
    // 真值替身免疫此问题（其换算误差自相消），故 M3 五轮从未暴露。
    declare_parameter<double>("search_settle_s", 8.0);

    // 悬停识别（悬停优先+蛇形兜底，2026-10-08 用户定案，SSOT §5.1）：首段转场
    // 改指投放区中心识别点，到位后定点悬停建图，锁定即跳过蛇形；窗口到期未锁
    // 转蛇形兜底。timeout=0 关闭该段；m1_no_vision_mode 恒跳过（无锁定可能）。
    declare_parameter<double>("search_hover_x_m", 30.0);
    declare_parameter<double>("search_hover_y_m", 0.0);
    declare_parameter<double>("search_hover_timeout_s", 12.0);

    declare_parameter<double>("recon_x_m", 52.5);
    declare_parameter<double>("recon_lane_len_m", 5.0);
    declare_parameter<double>("recon_half_width_m", 4.0);
    declare_parameter<int>("recon_lanes", 3);

    // 视觉起飞门禁（SSOT §5.4：起飞前 ≥3 帧有效；0 = 关闭；M1 无视觉模式不生效）
    declare_parameter<int>("vision_takeoff_min_frames", 3);

    // ALIGN/RELEASE 视觉速度伺服（SSOT §5.2-209：对准段发速度指令，≥10Hz +
    // 500ms 看护失新即切位置保持）。默认关（位置 setpoint 对准），M2 视觉接入
    // 实测后置 true。Kp 为世界系 (m/s)/m；SSOT 的 Kp=0.003 px→m/s 属感知层，
    // 世界系等效值按检测距离换算，0.8/0.25 m/s 限幅 = §5.4 对准/释放定位基线。
    declare_parameter<bool>("align_servo_enabled", false);
    declare_parameter<double>("align_servo_kp", 1.0);
    declare_parameter<double>("align_servo_max_align_m_s", 0.8);
    declare_parameter<double>("align_servo_max_release_m_s", 0.25);
    declare_parameter<double>("align_servo_deadzone_m", 0.02);
    declare_parameter<double>("align_servo_watchdog_s", 0.5);

    // 仿真投放判分桥：RELEASE fire 时同步调用 /drop_controller/release（仅 SITL，
    // 虚拟判定节点按调用瞬间位置判 A/B 区；真机必须 false）
    declare_parameter<bool>("sim_release_bridge", false);

    // ---- A4 事件流（RELEASE 证据 JSONL 追加落盘；空=禁用）----
    // 记录 aim 冻结/fire 指令/舵机结果/payload 完成/弃桶——赛后归因证据链
    declare_parameter<std::string>("event_log_path", "");

    // ---- 时间同步（P0.4，SSOT §4.3 容差照抄）----
    declare_parameter<double>("perception_max_delay_s", 1.5);   // 感知管线延迟上界
    declare_parameter<double>("odom_future_tol_s", 0.05);       // 未来戳容差
    declare_parameter<double>("odom_interp_max_gap_s", 0.2);    // 插值最大帧间隔
    declare_parameter<double>("odom_history_span_s", 3.0);      // odom 历史时间窗

    // ---- M3 感知契约（接口契约.md §1）----
    declare_parameter<std::string>("expected_frame_id", "cuadc_body_flu");
    declare_parameter<double>("contract_version", 1.0);         // 哨兵 orientation.z
    declare_parameter<double>("min_detection_confidence", 0.25);

    // ---- M3 锁定五步（drop_logic::LockParams，SSOT §5.2/§5.4）----
    declare_parameter<int>("lock_min_confirm_frames", 5);
    // 锁定用航迹新鲜窗：LockParams 默认 1.0s 适合 HIT 密集桶位（3 筒同视野）；
    // 我们的场地桶距 2~5m、逐带经过，须给"建图级"窗口让三筒先后入图后统一锁定
    // （桶是静止物，长窗无风险；RELEASE 反盲投新鲜度由 release_max_target_age
    // 单独把守，与此无关）。2026-09-12 仿真验证：1.0s 时锁定永远无法满足。
    declare_parameter<double>("lock_fresh_window_s", 60.0);
    declare_parameter<double>("lock_assoc_gate_m", 0.50);
    declare_parameter<double>("lock_ema_alpha", 0.25);
    declare_parameter<int>("lock_diameter_window", 9);
    declare_parameter<double>("lock_jitter_gate_m", 0.15);
    declare_parameter<double>("lock_min_spacing_m", 0.20);
    declare_parameter<double>("lock_min_diameter_diff_m", 0.025);
    declare_parameter<double>("lock_diameter_min_m", 0.08);
    declare_parameter<double>("lock_diameter_max_m", 0.35);
    declare_parameter<double>("lock_diameter_match_tol_m", 0.035);
    declare_parameter<double>("lock_rank_stable_s", 0.8);
    declare_parameter<double>("lock_blacklist_radius_m", 0.25);
    declare_parameter<double>("lock_blacklist_window_s", 120.0);

    // ---- M3 对准（SSOT §5.4：粗/精 0.15/0.08，稳定 0.8s，超时 12s）----
    declare_parameter<double>("align_alt_m", 1.8);
    declare_parameter<double>("align_coarse_radius_m", 0.15);
    declare_parameter<double>("align_fine_radius_m", 0.08);
    declare_parameter<double>("align_stable_s", 0.8);
    declare_parameter<double>("align_timeout_s", 12.0);

    // ---- M3 释放九门控（SSOT §5.2）----
    declare_parameter<double>("release_max_horiz_err_m", 0.10);
    declare_parameter<double>("release_max_vert_err_m", 0.10);
    declare_parameter<double>("release_max_hspeed_m_s", 0.08);
    declare_parameter<double>("release_max_vspeed_m_s", 0.05);
    declare_parameter<double>("release_max_tilt_deg", 5.0);
    declare_parameter<double>("release_max_ang_rate_deg_s", 5.0);
    declare_parameter<double>("release_max_yaw_err_deg", 5.0);
    declare_parameter<double>("release_stability_s", 0.8);
    declare_parameter<double>("release_hold_s", 1.5);
    declare_parameter<double>("release_gate_timeout_s", 6.0);
    declare_parameter<double>("release_max_target_age_s", 0.5);
    declare_parameter<double>("release_max_lead_m", 0.15);

    // ---- ALIGN 段目标跟踪容忍窗（2026-09-12 仿真实测调参）----
    // 锁定后飞机要转场 2~5m 去目标桶，途中目标暂出视野属正常；tracker 若沿用
    // RELEASE 反盲投的 0.5s 新鲜窗，转场必判"丢失"→2s 重捕获窗内到不了目标
    // →误弃桶（实测 0/2）。放松 tracker 容忍窗（新鲜 1s + 重捕获窗 4s ≈ 5s 容忍）；
    // 防盲投不受影响——RELEASE 门控用自己的 max_target_age_s(0.5s) 独立把守。
    declare_parameter<double>("align_lost_fresh_s", 1.0);
    declare_parameter<double>("align_reacquire_window_s", 4.0);

    // ---- M3 载荷与舵机 ----
    declare_parameter<std::string>("drop_order", "conservative");  // pair_23 先大保底（默认）
    declare_parameter<std::vector<double>>(
      "payload_offset_1", std::vector<double>{0.0, 0.0});          // §6 标定后填
    declare_parameter<std::vector<double>>(
      "payload_offset_2", std::vector<double>{0.0, 0.0});
    declare_parameter<int>("servo_channel_1", 9);                  // AUX 9/10
    declare_parameter<int>("servo_channel_2", 10);
    declare_parameter<double>("servo_stowed_pwm", 1100.0);
    declare_parameter<double>("servo_release_pwm", 1700.0);
    declare_parameter<bool>("enable_release_output", false);       // 干跑开关
  }

  void load_parameters()
  {
    m1_no_vision_mode_ = get_parameter("m1_no_vision_mode").as_bool();
    auto_arm_on_guided_ = get_parameter("auto_arm_on_guided").as_bool();

    takeoff_alt_m_ = std::max(1.0, get_parameter("takeoff_alt_m").as_double());
    search_alt_m_ = std::max(1.0, get_parameter("search_alt_m").as_double());
    recon_alt_m_ = std::max(2.0, get_parameter("recon_alt_m").as_double());
    return_alt_m_ = std::max(1.0, get_parameter("return_alt_m").as_double());

    transit_speed_m_s_ = std::max(0.2, get_parameter("transit_speed_m_s").as_double());
    search_speed_m_s_ = std::max(0.2, get_parameter("search_speed_m_s").as_double());
    recon_speed_m_s_ = std::max(0.2, get_parameter("recon_speed_m_s").as_double());
    waypoint_accept_radius_m_ = std::max(
      0.10, get_parameter("waypoint_accept_radius_m").as_double());

    heading_lock_window_s_ = std::max(0.5, get_parameter("heading_lock_window_s").as_double());
    heading_lock_tol_deg_ = std::max(0.5, get_parameter("heading_lock_tol_deg").as_double());
    heading_lock_timeout_s_ = std::max(5.0, get_parameter("heading_lock_timeout_s").as_double());
    lock_position_max_speed_m_s_ = std::max(
      0.05, get_parameter("lock_position_max_speed_m_s").as_double());

    prestream_hold_s_ = std::max(0.5, get_parameter("prestream_hold_s").as_double());
    takeoff_timeout_s_ = std::max(10.0, get_parameter("takeoff_timeout_s").as_double());
    mission_timeout_s_ = std::max(30.0, get_parameter("mission_timeout_s").as_double());
    odom_timeout_s_ = std::max(0.2, get_parameter("odom_timeout_s").as_double());
    gps_timeout_s_ = std::max(0.2, get_parameter("gps_timeout_s").as_double());
    recon_photo_timeout_s_ = std::max(0.2, get_parameter("recon_photo_timeout_s").as_double());

    landing_confirm_s_ = std::max(0.5, get_parameter("landing_confirm_s").as_double());
    landing_alt_gate_m_ = std::max(0.05, get_parameter("landing_alt_gate_m").as_double());
    landing_hspeed_gate_m_s_ = std::max(
      0.05, get_parameter("landing_hspeed_gate_m_s").as_double());
    landing_vspeed_gate_m_s_ = std::max(
      0.05, get_parameter("landing_vspeed_gate_m_s").as_double());
    extended_state_timeout_s_ = std::max(
      0.2, get_parameter("extended_state_timeout_s").as_double());

    search_x_min_m_ = get_parameter("search_x_min_m").as_double();
    search_x_max_m_ = get_parameter("search_x_max_m").as_double();
    search_half_width_m_ = std::max(0.5, get_parameter("search_half_width_m").as_double());
    search_lanes_ = std::max(1, static_cast<int>(get_parameter("search_lanes").as_int()));
    search_degrade_timeout_s_ = std::max(
      10.0, get_parameter("search_degrade_timeout_s").as_double());
    search_settle_s_ = std::max(
      0.0, get_parameter("search_settle_s").as_double());
    search_hover_x_m_ = get_parameter("search_hover_x_m").as_double();
    search_hover_y_m_ = get_parameter("search_hover_y_m").as_double();
    search_hover_timeout_s_ = std::max(
      0.0, get_parameter("search_hover_timeout_s").as_double());

    recon_x_m_ = get_parameter("recon_x_m").as_double();
    recon_lane_len_m_ = std::max(0.5, get_parameter("recon_lane_len_m").as_double());
    recon_half_width_m_ = std::max(0.5, get_parameter("recon_half_width_m").as_double());
    recon_lanes_ = std::max(1, static_cast<int>(get_parameter("recon_lanes").as_int()));

    vision_takeoff_min_frames_ =
      std::max(0, static_cast<int>(get_parameter("vision_takeoff_min_frames").as_int()));

    align_servo_enabled_ = get_parameter("align_servo_enabled").as_bool();
    align_servo_kp_ = std::max(0.05, get_parameter("align_servo_kp").as_double());
    align_servo_max_align_m_s_ = std::max(
      0.05, get_parameter("align_servo_max_align_m_s").as_double());
    align_servo_max_release_m_s_ = std::max(
      0.05, get_parameter("align_servo_max_release_m_s").as_double());
    align_servo_deadzone_m_ = std::max(
      0.0, get_parameter("align_servo_deadzone_m").as_double());
    align_servo_watchdog_s_ = std::max(
      0.1, get_parameter("align_servo_watchdog_s").as_double());
    if (align_servo_enabled_) {
      RCLCPP_WARN(get_logger(),
        "align_servo_enabled=true：ALIGN/RELEASE 用速度伺服（失新 %.0fms 切位置保持）",
        align_servo_watchdog_s_ * 1000.0);
    }

    sim_release_bridge_ = get_parameter("sim_release_bridge").as_bool();
    event_log_path_ = get_parameter("event_log_path").as_string();
    if (sim_release_bridge_) {
      RCLCPP_WARN(get_logger(),
        "sim_release_bridge=true：fire 同步调 /drop_controller/release（仅仿真判分）");
    }

    perception_max_delay_s_ = std::max(
      0.2, get_parameter("perception_max_delay_s").as_double());
    odom_future_tol_s_ = std::max(0.01, get_parameter("odom_future_tol_s").as_double());
    odom_interp_max_gap_s_ = std::max(
      0.05, get_parameter("odom_interp_max_gap_s").as_double());
    odom_history_span_s_ = std::max(1.0, get_parameter("odom_history_span_s").as_double());
    interp_params_.future_tol_s = odom_future_tol_s_;
    interp_params_.max_delay_s = perception_max_delay_s_;
    interp_params_.max_gap_s = odom_interp_max_gap_s_;

    // ---- M3 契约 ----
    expected_frame_id_ = get_parameter("expected_frame_id").as_string();
    contract_version_ = get_parameter("contract_version").as_double();
    min_detection_confidence_ = std::clamp(
      get_parameter("min_detection_confidence").as_double(), 0.0, 1.0);

    // ---- M3 锁定参数 → BucketMap ----
    auto & lp = bucket_map_.params();
    lp.min_confirm_frames = static_cast<int>(
      std::max<int64_t>(2, get_parameter("lock_min_confirm_frames").as_int()));
    lp.fresh_window_s = std::max(
      1.0, get_parameter("lock_fresh_window_s").as_double());
    lp.assoc_gate_m = std::max(0.2, get_parameter("lock_assoc_gate_m").as_double());
    lp.ema_alpha = std::clamp(get_parameter("lock_ema_alpha").as_double(), 0.05, 1.0);
    lp.diameter_window = static_cast<int>(
      std::max<int64_t>(3, get_parameter("lock_diameter_window").as_int()));
    lp.jitter_gate_m = std::max(0.02, get_parameter("lock_jitter_gate_m").as_double());
    lp.min_spacing_m = std::max(0.05, get_parameter("lock_min_spacing_m").as_double());
    lp.min_diameter_diff_m = std::max(
      0.005, get_parameter("lock_min_diameter_diff_m").as_double());
    lp.diameter_min_m = get_parameter("lock_diameter_min_m").as_double();
    lp.diameter_max_m = get_parameter("lock_diameter_max_m").as_double();
    lp.diameter_match_tol_m = std::max(
      0.005, get_parameter("lock_diameter_match_tol_m").as_double());
    lp.rank_stable_s = std::max(0.0, get_parameter("lock_rank_stable_s").as_double());
    lp.blacklist_radius_m = std::max(
      0.05, get_parameter("lock_blacklist_radius_m").as_double());
    lp.blacklist_window_s = std::max(
      10.0, get_parameter("lock_blacklist_window_s").as_double());

    // ---- M3 对准 ----
    align_alt_m_ = std::max(1.0, get_parameter("align_alt_m").as_double());
    align_coarse_radius_m_ = std::max(
      0.05, get_parameter("align_coarse_radius_m").as_double());
    align_fine_radius_m_ = std::min(
      align_coarse_radius_m_, std::max(0.02, get_parameter("align_fine_radius_m").as_double()));
    align_stable_s_ = std::max(0.2, get_parameter("align_stable_s").as_double());
    align_timeout_s_ = std::max(3.0, get_parameter("align_timeout_s").as_double());

    // ---- M3 释放门控 → ReleaseGate ----
    auto & gp = release_gate_.params();
    gp.max_horizontal_error_m = std::max(
      0.02, get_parameter("release_max_horiz_err_m").as_double());
    gp.max_vertical_error_m = std::max(
      0.02, get_parameter("release_max_vert_err_m").as_double());
    gp.max_hspeed_m_s = std::max(
      0.01, get_parameter("release_max_hspeed_m_s").as_double());
    gp.max_vspeed_m_s = std::max(
      0.01, get_parameter("release_max_vspeed_m_s").as_double());
    gp.max_tilt_deg = std::max(1.0, get_parameter("release_max_tilt_deg").as_double());
    gp.max_angular_rate_deg_s = std::max(
      1.0, get_parameter("release_max_ang_rate_deg_s").as_double());
    gp.max_yaw_err_deg = std::max(
      1.0, get_parameter("release_max_yaw_err_deg").as_double());
    gp.stability_s = std::max(0.2, get_parameter("release_stability_s").as_double());
    gp.hold_s = std::max(0.0, get_parameter("release_hold_s").as_double());
    gp.timeout_s = std::max(
      gp.stability_s + gp.hold_s, get_parameter("release_gate_timeout_s").as_double());
    gp.max_target_age_s = std::max(
      0.1, get_parameter("release_max_target_age_s").as_double());
    gp.max_lead_m = std::max(0.02, get_parameter("release_max_lead_m").as_double());
    gp.release_height_m = align_alt_m_;
    // tracker 容忍窗与 RELEASE 反盲投解耦（见 declare 处注释）：
    // 防盲投口径 = release_gate 的 max_target_age_s，不再借用给 tracker
    track_params_.fresh_window_s = std::max(
      0.3, get_parameter("align_lost_fresh_s").as_double());
    track_params_.reacquire_window_s = std::max(
      1.0, get_parameter("align_reacquire_window_s").as_double());

    // ---- M3 载荷与舵机 ----
    drop_order_ = get_parameter("drop_order").as_string();
    if (cuadc_drop::drop_order_sequence(drop_order_).empty()) {
      RCLCPP_WARN(get_logger(), "drop_order 非法(%s)，回退 conservative", drop_order_.c_str());
      drop_order_ = "conservative";
    }
    payload_offset_xy_[0] = load_offset("payload_offset_1");
    payload_offset_xy_[1] = load_offset("payload_offset_2");
    servo_channel_[0] = static_cast<int>(get_parameter("servo_channel_1").as_int());
    servo_channel_[1] = static_cast<int>(get_parameter("servo_channel_2").as_int());
    servo_stowed_pwm_ = get_parameter("servo_stowed_pwm").as_double();
    servo_release_pwm_ = get_parameter("servo_release_pwm").as_double();
    enable_release_output_ = get_parameter("enable_release_output").as_bool();
    if (enable_release_output_) {
      RCLCPP_WARN(get_logger(),
        "enable_release_output=true：实弹模式，发射前确认拆桨/安全员/接管手段！");
    }
  }

  /// 载荷投放口偏置（机体系 x 前 y 左，§6 两步法标定产物），非法长度回退 0。
  std::pair<double, double> load_offset(const std::string & name)
  {
    const auto v = get_parameter(name).as_double_array();
    if (v.size() != 2) {
      RCLCPP_WARN(get_logger(), "%s 需要 2 个值 [x, y]，已回退 (0,0)", name.c_str());
      return {0.0, 0.0};
    }
    return {v[0], v[1]};
  }

  /// A3：启动打印"生效配置"全量 + 指纹（SSOT §10.2-6 防 YAML 与代码脱节；
  /// Re0/ROS §21.5 effective configuration 的落地）。指纹 = 排序后"名=值"行
  /// 的 FNV-1a 64（无依赖实现；用途=跨次启动漂移比对，非安全哈希）。
  void log_effective_config()
  {
    const auto list = get_node_parameters_interface()->list_parameters({}, 16);
    std::vector<std::string> lines;
    lines.reserve(list.names.size());
    for (const auto & name : list.names) {
      rclcpp::Parameter p;
      if (get_parameter(name, p)) {
        lines.push_back(name + "=" + p.value_to_string());
      }
    }
    std::sort(lines.begin(), lines.end());
    std::uint64_t h = 1469598103934665603ULL;
    for (const auto & line : lines) {
      for (const char c : line) {
        h ^= static_cast<std::uint8_t>(c);
        h *= 1099511628211ULL;
      }
    }
    RCLCPP_INFO(get_logger(), "生效配置 %zu 项，指纹=%016llX（FNV-1a64，漂移比对用）",
      lines.size(), static_cast<unsigned long long>(h));
    for (const auto & line : lines) {
      RCLCPP_INFO(get_logger(), "  cfg %s", line.c_str());
    }
  }

  /// A4 事件流：RELEASE 关键证据 JSONL 追加落盘（event_log_path 空=禁用）。
  /// fields 须为合法 JSON 字段串（值来自内部枚举/数值，无引号转义需求）。
  /// 打开失败静默跳过——证据链是尽力而为，绝不影响任务主链。
  void append_event(const std::string & event, const std::string & fields)
  {
    if (event_log_path_.empty()) {
      return;
    }
    std::ofstream f(event_log_path_, std::ios::app);
    if (!f.is_open()) {
      return;
    }
    f << "{\"t\":" << std::fixed << std::setprecision(3) << now().seconds()
      << ",\"state\":\"" << state_name(state_) << "\",\"event\":\"" << event << "\"";
    if (!fields.empty()) {
      f << ',' << fields;
    }
    f << "}\n";
  }

  // ===========================================================================
  // 数据回调：数据一到就更新成员变量，供 tick() 随时查询
  // ===========================================================================
  void state_callback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    const bool was_guided = guided_active_;
    const bool prev_armed = was_armed_.value_or(msg->armed);  // 首帧不视为边沿
    fcu_state_ = *msg;
    guided_active_ = fcu_state_.connected && fcu_state_.mode == "GUIDED";
    // 解锁上升沿：只做起飞 AMSL 健康门冻结（fail-closed）。任务高度体系与解锁沿
    // 无关——实际=RTK 推送、基准=上电冻结（lock_frame 完成），2026-10-08 用户定案
    if (!prev_armed && fcu_state_.armed) {
      on_armed_edge();
    }
    was_armed_ = fcu_state_.armed;
    // 飞手切走 GUIDED = 人工接管（SSOT §5.1 旁路）：立即停发目标点并退让
    if (was_guided && !guided_active_ && autonomous_state(state_)) {
      publish_setpoint_enabled_ = false;
      enter(State::PILOT_OVERRIDE);
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    position_ = Point3{
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z};
    horizontal_speed_m_s_ = std::hypot(
      msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    vertical_speed_m_s_ = msg->twist.twist.linear.z;
    // 角速度三轴模长（deg/s）：九门控第 7 项输入（2026-10-08 借鉴 hgd）
    const auto & angular = msg->twist.twist.angular;
    angular_rate_deg_s_ = std::sqrt(
      angular.x * angular.x + angular.y * angular.y + angular.z * angular.z) *
      (180.0 / M_PI);
    have_odom_ = true;
    last_odom_time_ = now();

    // P0.4：维护 odom 历史供视觉帧时间插值（M2 消费；判定核心在 odom_interp.hpp）
    cuadc_odom::Sample sample;
    // 统一换成节点时钟类型再取秒，避免 rclcpp::Time 因时钟类型不同抛异常
    sample.t = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()).seconds();
    sample.x = position_.x;
    sample.y = position_.y;
    sample.z = position_.z;
    const auto & q = msg->pose.pose.orientation;
    sample.yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    odom_history_.push_back(sample);
    // 双重裁剪：按时间窗裁 + 硬上限兜底（防时间戳异常时不收敛）
    const double cutoff = sample.t - odom_history_span_s_;
    while (!odom_history_.empty() && odom_history_.front().t < cutoff) {
      odom_history_.pop_front();
    }
    constexpr std::size_t kMaxOdomHistory = 600U;
    while (odom_history_.size() > kMaxOdomHistory) {
      odom_history_.pop_front();
    }
  }

  void compass_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!std::isfinite(msg->data)) {
      return;
    }
    current_compass_deg_ = normalize_degrees(msg->data);
    // ENU 航向 = 90° − 罗盘航向（SSOT §10.2-1，坐标系列坑之首，勿改；
    // 换算收敛到 route_logic.hpp 纯函数，test_route_logic 回归锚点守门）
    current_heading_enu_ = normalize_angle(
      cuadc_route::heading_enu_rad_from_compass_deg(current_compass_deg_));
    have_compass_ = true;
  }

  void recon_ack_callback(const std_msgs::msg::UInt32::SharedPtr msg)
  {
    if (std::find(recon_acks_.begin(), recon_acks_.end(), msg->data) == recon_acks_.end()) {
      recon_acks_.push_back(msg->data);
    }
  }

  /// 姿态：倾角（机体系 z 轴与铅垂夹角）+ 与锁定航向的偏差（九门控用；角速度
  /// 三轴模长在 odom 回调另算）。
  /// 单位四元数下 cos(倾角) = 1 − 2(qx² + qy²)。
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto & q = msg->pose.orientation;
    const double n2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (n2 < 1e-9) {
      return;
    }
    const double cos_tilt = std::clamp(1.0 - 2.0 * (q.x * q.x + q.y * q.y) / n2, -1.0, 1.0);
    tilt_deg_ = std::acos(cos_tilt) * 180.0 / kPi;
    const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    yaw_err_deg_ = std::fabs(normalize_angle(yaw - mission_yaw_)) * 180.0 / kPi;
  }

  /// 世界系速度（velocity_local 为 ENU）：弹道前移 sanity 与瞄准补偿用。
  void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    vx_local_ = msg->twist.linear.x;
    vy_local_ = msg->twist.linear.y;
  }

  /// 感知检测（接口契约.md §1 薄契约）：哨兵校验 → P0.4 插值换算世界系 →
  /// SEARCH 段喂 BucketMap；对准/释放段由 update_align/update_release 消费最新帧。
  void bucket_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (msg->header.frame_id != expected_frame_id_) {
      ++perception_bad_frames_;
      return;
    }
    for (const auto & p : msg->poses) {
      if (std::fabs(p.orientation.z - contract_version_) > 0.01) {
        ++perception_bad_frames_;
        return;                                   // 版本哨兵不一致：整帧拒用
      }
    }
    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
    const auto ref = interpolate_odom(stamp);
    if (!ref.has_value()) {
      ++perception_stale_frames_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "感知帧被拒(P0.4: 太旧/未来戳/断流), 累计 %d 帧", perception_stale_frames_);
      return;                                     // 拒绝不外推（时间同步设计.md §3）
    }
    latest_detections_.clear();
    for (const auto & p : msg->poses) {
      if (!std::isfinite(p.orientation.y) ||
        p.orientation.y < min_detection_confidence_)
      {
        continue;    // 置信度非有限/低于门限：跳过该 pose（A5 小扫除补 isfinite）
      }
      if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) ||
        !std::isfinite(p.position.z) || !std::isfinite(p.orientation.x))
      {
        continue;
      }
      const Point3 world = body_to_local_at(*ref, {p.position.x, p.position.y, p.position.z});
      latest_detections_.push_back(cuadc_drop::Detection2D{world.x, world.y, p.orientation.x});
    }

    // 调试输出：插值换算后的世界系坐标（P0.4 验收 / rviz；z 未保留，仅平面）。
    // 空帧照发，与检测话题同节奏。
    geometry_msgs::msg::PoseArray dbg;
    dbg.header.stamp = msg->header.stamp;          // = 取帧时刻
    dbg.header.frame_id = "map";
    for (const auto & d : latest_detections_) {
      geometry_msgs::msg::Pose dp;
      dp.position.x = d.x;
      dp.position.y = d.y;
      dp.orientation.w = 1.0;
      dbg.poses.push_back(dp);
    }
    buckets_world_pub_->publish(dbg);

    have_vision_ = true;
    last_vision_time_ = now();
    ++vision_valid_frames_;    // 视觉起飞门禁计数（SSOT §5.4 起飞前 ≥3 帧有效）
    if (state_ == State::SEARCH &&
      (now() - state_enter_time_).seconds() > search_settle_s_)
    {
      const double t = stamp.seconds();
      for (const auto & d : latest_detections_) {
        bucket_map_.update(d.x, d.y, d.diameter, t);
      }
    }
  }

  // ===========================================================================
  // 坐标系与航线（SSOT §10.2-1：起飞前锁死任务原点与航向，全程不转机头）
  // ===========================================================================

  /// 场地坐标（原点=起飞点，x 朝锁定的机头方向，y 向左）→ 本地 ENU
  /// （旋转/平移收敛到 route_logic.hpp 纯函数，test_route_logic 守门）
  Point3 field_to_local(double x, double y, double rel_z) const
  {
    const Point3 origin = home_.value_or(Point3{});
    const cuadc_route::Point3 p = cuadc_route::field_to_local(
      mission_yaw_, origin.x, origin.y, origin.z, x, y, rel_z);
    return Point3{p.x, p.y, p.z};
  }

  /// P0.4：把时间戳 t 夹逼/插值到 odom 历史上的位姿（SSOT §4.3 容差参数照抄）。
  /// 判定核心收敛 odom_interp.hpp 纯函数（09-20 自本函数抽出，行为逐分支保持，
  /// 等号/大间隔/回跳由 test_odom_interp 双端锁死）；拒绝即丢弃，绝不外推。
  std::optional<cuadc_odom::Sample> interpolate_odom(const rclcpp::Time & t) const
  {
    const auto r = cuadc_odom::interpolate(odom_history_, t.seconds(), interp_params_);
    if (r.status != cuadc_odom::InterpStatus::kOk) {
      return std::nullopt;
    }
    return r.out;
  }

  /// 把机体系目标（x前 y左 z上）按"取帧瞬间"的位姿换算到本地 ENU（M2 消费）。
  /// 相机-机体外参与投放口偏置在 M3 标定后叠加，本函数只做插值位姿的旋转平移。
  Point3 body_to_local_at(const cuadc_odom::Sample & ref, const Point3 & body) const
  {
    const double c = std::cos(ref.yaw);
    const double s = std::sin(ref.yaw);
    return Point3{
      ref.x + c * body.x - s * body.y,
      ref.y + s * body.x + c * body.y,
      ref.z + body.z};
  }

  /// 锁定任务坐标系。注意 EKF origin ≠ 起飞点，任务原点必须取上电瞬间（未解锁
  /// 地面态）odom——本捕获即任务高度基准（2026-10-08 用户定案），此后不重锚
  void lock_frame()
  {
    home_ = position_;
    mission_yaw_ = current_heading_enu_;
    locked_compass_deg_ = current_compass_deg_;
    yaw_qz_ = std::sin(mission_yaw_ * 0.5);
    yaw_qw_ = std::cos(mission_yaw_ * 0.5);
    target_ = *home_;
    frame_locked_ = true;
    build_search_route();
    build_recon_route();
    // 任务高度体系（2026-10-08 用户定案）：实际高度=RTK 推送（NavSatFix 海拔），
    // GNSS 基准高度 2026-10-09 用户定案改在解锁上升沿冻结（on_armed_edge）：
    // 锁框架→解锁之间飞手等待的分钟级时间里 GNSS 漂移 ±5m（210 号实测），
    // 上电冻结的旧基准曾使 0.9× 起飞判据永远够不着（TAKEOFF 超时无法起飞）。
    // 本函数只锁 home z（odom 载体，航线 setpoint 系）与航向。
    RCLCPP_INFO(
      get_logger(), "任务坐标系已锁定: home=(%.2f, %.2f, %.2f) 航向=%.1f°（GNSS 基准待解锁沿冻结）",
      home_->x, home_->y, home_->z, locked_compass_deg_);
  }

  /// 弓字形搜索航线：search_lanes 条平行带（沿 X 扫，带位沿 Y 均布），奇偶带交替方向。
  /// 覆盖判据（test_route_logic 守门）：间距/2 ≤ 相机横向半覆盖 = 0.527×search_alt。
  /// 2.5m 高（10-08 统一）半覆盖≈1.32m，6 带（间距 1.6m）中点 0.8m 无盲区余量更大；
  /// 2.0m 时半覆盖 1.05m，4 带（2.67m）带间中点 1.33m 曾有 0.28m 盲区。
  void build_search_route()
  {
    search_route_.clear();
    const double x0 = std::min(search_x_min_m_, search_x_max_m_);
    const double x1 = std::max(search_x_min_m_, search_x_max_m_);
    const auto route = cuadc_route::build_serpentine(
      x0, x1, -search_half_width_m_, search_half_width_m_, search_lanes_, true);
    for (const auto & fp : route) {
      search_route_.push_back(field_to_local(fp.x, fp.y, search_alt_m_));
    }
  }

  /// 侦察航线（2026-09-12 重排）：recon_lanes 条带改沿 Y（区长轴 8m）扫全宽，
  /// 带位沿 X 在 [recon_x, recon_x+recon_lane_len] 均布——带间距 2.5m，
  /// 4m 高横向半覆盖 2.11m，余量 0.86m（原沿 X 短带 4m 间距余量仅 0.11m）。
  /// 2×recon_lanes 航点（默认 6，SSOT §5.1），奇偶带交替方向。
  void build_recon_route()
  {
    recon_route_.clear();
    const auto route = cuadc_route::build_serpentine(
      recon_x_m_, recon_x_m_ + recon_lane_len_m_,
      -recon_half_width_m_, recon_half_width_m_, recon_lanes_, false);
    for (const auto & fp : route) {
      recon_route_.push_back(field_to_local(fp.x, fp.y, recon_alt_m_));
    }
  }

  // ===========================================================================
  // 主状态机：50ms 一拍（20Hz）。每拍只回答三个问题：
  //   现在在哪个状态？该干什么？干完了吗（干完就 enter 下一态）？
  // ===========================================================================
  void tick()
  {
    check_service_results();
    // 飞手接管后不再向 FC 发任何新指令（2026-10-08 借鉴 hgd command_authority：
    // PILOT_OVERRIDE 为最高优先级终态，故障收尾不得恢复软件控制权）；
    // check_service_results 只回收在途结果不重发，照常运行
    if (state_ != State::PILOT_OVERRIDE) {
      service_pending_servo();
    }

    // 任务总超时看门狗（SSOT §5.4：240s，比赛 5 分钟）；
    // 一旦进入返航/降落段不再受它约束，让飞机安全回家
    if (mission_started_ && !returning_or_landed() &&
      (now() - mission_start_time_).seconds() > mission_timeout_s_)
    {
      fail_and_return("Mission timeout");
      return;
    }

    switch (state_) {
      case State::WAIT_FCU:
        publish_setpoint_enabled_ = false;
        if (fcu_state_.connected) {
          enter(State::WAIT_NAV_STABLE);
        }
        break;

      case State::WAIT_NAV_STABLE:
        publish_setpoint_enabled_ = false;
        if (!fcu_state_.connected) {
          enter(State::WAIT_FCU);
        } else if (fcu_state_.armed) {
          fail("Aircraft armed before mission frame lock");
        } else if (navigation_ready()) {
          enter(State::LOCK_FRAME);
        }
        break;

      case State::LOCK_FRAME: {
        publish_setpoint_enabled_ = false;
        // 两个判据都必须每拍推进——短路求值会冻住静止计时器（10-08 借鉴 hgd
        // 增位置静止判据：罗盘稳但机体被推动时，锁出的航线会整体平移）
        const bool heading_ok = heading_locked();
        const bool still_ok = position_still();
        if (!fcu_state_.connected) {
          enter(State::WAIT_FCU);
        } else if (fcu_state_.armed) {
          fail("Aircraft armed before mission frame lock");
        } else if (!navigation_ready()) {
          heading_lock_since_.reset();  // 定位断了重新计时
          still_since_.reset();
        } else if (heading_ok && still_ok) {
          lock_frame();
          publish_setpoint_enabled_ = true;
          enter(State::PRESTREAM);
        } else if ((now() - state_enter_time_).seconds() > heading_lock_timeout_s_) {
          fail("LOCK_FRAME 超时：航向漂移超容差或机体未静止（罗盘漂移/震动？检查校准与干扰）");
        }
        break;
      }

      case State::PRESTREAM:
        target_ = home_.value_or(position_);
        if (!navigation_ready()) {
          fail("Navigation lost during prestream");
        } else if ((now() - state_enter_time_).seconds() >= prestream_hold_s_) {
          enter(State::WAIT_GUIDED);
        }
        break;

      case State::WAIT_GUIDED:
        target_ = home_.value_or(position_);
        if (guided_active_) {
          enter(State::WAIT_ARM);
        }
        break;

      case State::WAIT_ARM:
        target_ = home_.value_or(position_);
        if (!guided_active_) {
          enter(State::WAIT_GUIDED);
        } else if (fcu_state_.armed) {
          mission_started_ = true;
          mission_start_time_ = now();
          publish_setpoint_enabled_ = false;  // 起飞交 CommandTOL，不与之抢权
          enter(State::TAKEOFF);
        } else if (auto_arm_on_guided_) {
          request_arm(true);
        }
        break;

      case State::TAKEOFF:
        publish_setpoint_enabled_ = false;
        // 起飞由 FC 自飞（CommandTOL），FSM 只做监视——遥测 odom 断流不应中止起飞，
        // 只延后转换判定（60s 超时兜底）；2026-10-07 台架实测：爬升中 odom 断流 >1s
        // 曾被本态门禁判「Odometry stale」直接进 LAND（209 号日志两次复现）
        if (!flight_gate_ok(false)) {
          break;
        }
        if (!takeoff_sent_) {
          // 「已在空中」守卫（10-07 加，10-08 改 odom 载体）：进入 TAKEOFF 时
          // odom 相对锁框架点已爬升 >1m = 真在空中（接管/延迟进入场景），拒发
          // 起飞防基准错位冲高。载体原为 GNSS 相对高——分钟级漂移 ±5m（210 号
          // 实锤解锁沿 +4.84m）在地面即误触本守卫拒飞（3D DGPS 启动定案的实测
          // 拦路虎一）；odom 同载体差值漂移相消，真空中进入（地面锁框架后手动
          // 拉起）odom 看得见爬升、照样拦住。只在未发起飞命令时守卫：已发后
          // 爬升越 1m 是命令本身的预期效果
          if (position_.z - home_.value_or(position_).z > 1.0) {
            fail_and_return("TAKEOFF 进入时已在空中——拒绝起飞");
            break;
          }
          // 视觉起飞门禁（SSOT §5.4：起飞前 ≥3 帧有效；M1 无视觉模式跳过）
          if (vision_takeoff_gate_ok()) {
            request_takeoff();
          } else {
            // 2026-10-07 补：门禁拦截此前完全静默，台架无摄像头时表现为
            // 「TAKEOFF 60s 零日志超时」排障三日不可见——必须留痕
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
              "视觉起飞门禁未过: %d/%d 帧（感知在流吗？台架无视觉验证起飞链请用 --no-vision）",
              vision_valid_frames_, vision_takeoff_min_frames_);
          }
        }
        // 起飞完成 0.9×（GNSS 口径主判据 + odom 或分支，2026-10-08 定案配套）：
        // 单点 GPS 垂直噪声 ±2~4m，GNSS 读数偏低会拖到 Takeoff timeout（拦路虎
        // 二）；odom=FC 爬升闭环自身变量（EK3_SRC1_POSZ 未切前=气压计），免疫
        // GNSS 噪声。命令未发出时不判完成——视觉门禁拦截期不许噪声高读数跳过
        // 起飞直入 SEARCH
        if (takeoff_sent_ &&
          (relative_altitude() >= takeoff_alt_m_ * 0.90 ||
          position_.z - home_.value_or(position_).z >= takeoff_alt_m_ * 0.90))
        {
          publish_setpoint_enabled_ = true;
          start_search();
        } else if ((now() - state_enter_time_).seconds() > takeoff_timeout_s_) {
          fail_and_return("Takeoff timeout");
        }
        break;

      case State::SEARCH:
        if (flight_gate_ok()) {
          update_search();
        }
        break;

      case State::ALIGN:
        if (flight_gate_ok()) {
          update_align();
        }
        break;

      case State::RELEASE:
        if (flight_gate_ok()) {
          update_release();
        }
        break;

      case State::RECON_CLIMB:
        if (flight_gate_ok()) {
          target_ = sample_segment();
          if (segment_complete(waypoint_accept_radius_m_)) {
            recon_index_ = 0U;
            photo_requested_ = false;
            if (recon_route_.empty()) {
              start_return();
            } else {
              start_segment(position_, recon_route_.front(), recon_speed_m_s_);
              enter(State::RECON_SURVEY);
            }
          }
        }
        break;

      case State::RECON_SURVEY:
        if (flight_gate_ok()) {
          update_recon_survey();
        }
        break;

      case State::RETURN_CLIMB:
        if (flight_gate_ok()) {
          target_ = sample_segment();
          if (segment_complete(waypoint_accept_radius_m_)) {
            start_segment(
              position_, Point3{home_->x, home_->y, home_->z + return_alt_m_},
              transit_speed_m_s_);
            enter(State::RETURN_HOME);
          }
        }
        break;

      case State::RETURN_HOME:
        if (flight_gate_ok()) {
          target_ = sample_segment();
          if (segment_complete(std::max(waypoint_accept_radius_m_, 0.5))) {
            publish_setpoint_enabled_ = false;  // 降落交还飞控
            enter(State::LAND);
          }
        }
        break;

      case State::LAND:
        publish_setpoint_enabled_ = false;
        if (landing_candidate()) {
          if (!landing_stable_since_.has_value()) {
            landing_stable_since_ = now();
          } else if ((now() - *landing_stable_since_).seconds() >= landing_confirm_s_) {
            enter(fcu_state_.armed ? State::DISARM : State::DONE);
          }
        } else {
          landing_stable_since_.reset();
          if (fcu_state_.armed) {
            request_land();
          }
        }
        break;

      case State::DISARM:
        publish_setpoint_enabled_ = false;
        if (!fcu_state_.armed) {
          enter(State::DONE);
        } else if (landing_candidate()) {
          request_arm(false);
        } else {
          enter(State::LAND);
        }
        break;

      case State::DONE:
        publish_setpoint_enabled_ = false;
        if (!done_logged_) {
          done_logged_ = true;
          std::string summary = "任务结束: 投放 " + std::to_string(payloads_released_) +
            "/" + std::to_string(kNPayloads) +
            ", 拍照确认数=" + std::to_string(recon_acks_.size());
          if (payload_result_unknown_ > 0) {
            summary += ", fire结果未知 " + std::to_string(payload_result_unknown_);
          }
          if (mission_failed_) {
            summary += " | 失败原因: " + terminal_reason_;
          }
          RCLCPP_INFO(get_logger(), "%s", summary.c_str());
        }
        if ((now() - state_enter_time_).seconds() > 1.0) {
          rclcpp::shutdown();
        }
        break;

      case State::PILOT_OVERRIDE:
        publish_setpoint_enabled_ = false;
        // 控制权已在飞手手里，程序体面退场
        if ((now() - state_enter_time_).seconds() > 1.0) {
          rclcpp::shutdown();
        }
        break;

      case State::ABORT:
        publish_setpoint_enabled_ = false;
        if (fcu_state_.armed) {
          enter(State::LAND);
        } else {
          rclcpp::shutdown();
        }
        break;
    }

    // A1-4：位置 setpoint 移到状态推进之后发布——本轮更新的目标当拍出，
    // 消除旧实现"先发旧目标、下拍才发新目标"的 50ms 滞后（11 册 §13.1）。
    // 伺服激活拍停发位置 setpoint（改发速度），避免两路 setpoint 交叠打架。
    // DONE/PILOT_OVERRIDE/ABORT 三个会 shutdown 的态都先停发，不会向已关闭
    // 的上下文发布。
    if (publish_setpoint_enabled_ && frame_locked_ && !servo_active_) {
      publish_setpoint();
    }
  }

  // ===========================================================================
  // 各阶段更新函数
  // ===========================================================================
  void start_search()
  {
    if (search_route_.empty()) {
      fail_and_return("Search route is empty");
      return;
    }
    search_index_ = 0U;
    // 悬停优先（2026-10-08 用户定案，SSOT §5.1）：首段先直线转场至投放区中心
    // 悬停识别点，到位后定点悬停建图——锁定即省掉蛇形 ~19s；窗口到期未锁转
    // 蛇形兜底（update_search）。M1 无视觉模式锁定不可能，不进悬停段白烧窗口；
    // timeout=0 同样直接走蛇形。
    if (!m1_no_vision_mode_ && search_hover_timeout_s_ > 0.0) {
      search_hover_active_ = true;
      start_segment(
        position_,
        field_to_local(search_hover_x_m_, search_hover_y_m_, search_alt_m_),
        transit_speed_m_s_);
      enter(State::SEARCH);
      return;
    }
    // 兜底首段 = 起飞点→搜索区入口（转场 ~28m），用转场速度；带内扫描仍 2m/s
    // （转场 4m/s 省 ~7s，SSOT §5.4 分速度基线：转场 4.0/搜索 2.0）
    start_segment(position_, search_route_.front(), transit_speed_m_s_);
    enter(State::SEARCH);
  }

  void update_search()
  {
    if (m1_no_vision_mode_) {
      // M1 演练模式：沿预设航线飞行，走完直接转侦察段
      target_ = sample_segment();
      if (!segment_complete(waypoint_accept_radius_m_)) {
        return;
      }
      ++search_index_;
      if (search_index_ < search_route_.size()) {
        start_segment(position_, search_route_[search_index_], search_speed_m_s_);
        return;
      }
      RCLCPP_WARN(
        get_logger(), "M1 演练：搜索航线完成，跳过对准/投放（M3 范围），转侦察段");
      start_recon_climb();
      return;
    }

    // 正式模式（SSOT §5.2）：航线飞行 + 每拍尝试锁定（HIT 经验：en route 用
    // 3 筒全锁，航线尽头允许"2 筒 + 1 未知"降级）。检测已在 bucket_callback 喂图。
    target_ = sample_segment();

    // SSOT P4-3：SEARCH 超时提前降级（2 筒+1 未知），不死等飞完全程再降；
    // 悬停段同样适用（转场/悬停途中锁定成功都从这里即时进 ALIGN）
    if ((now() - state_enter_time_).seconds() > search_degrade_timeout_s_ &&
      try_enter_align(true))
    {
      return;
    }
    if (try_enter_align(false)) {
      return;
    }

    // 悬停识别段（悬停优先+蛇形兜底，SSOT §5.1）：到位起算窗口，sample_segment
    // 终点钳位即悬停点，原地保持；锁定已由上方 try_enter_align 即时接管。窗口
    // 到期未锁→转蛇形兜底（喂图受 search_settle_s 沉降期把守，转场 ~8s 已耗完）。
    // 转场久未到位不单独设卡：60s 提前降级 + 240s 任务超时兜底，与兜底转场同暴露。
    if (search_hover_active_) {
      if (!hover_since_.has_value()) {
        if (!segment_complete(waypoint_accept_radius_m_)) {
          return;
        }
        hover_since_ = now();
        RCLCPP_INFO(get_logger(), "到达悬停识别点，窗口 %.0fs 起",
          search_hover_timeout_s_);
        return;
      }
      if ((now() - *hover_since_).seconds() > search_hover_timeout_s_) {
        search_hover_active_ = false;
        RCLCPP_WARN(get_logger(), "悬停识别 %.0fs 未锁定，转蛇形兜底",
          search_hover_timeout_s_);
        start_segment(position_, search_route_.front(), transit_speed_m_s_);
      }
      return;
    }

    if (!segment_complete(waypoint_accept_radius_m_)) {
      return;
    }
    ++search_index_;
    if (search_index_ < search_route_.size()) {
      start_segment(position_, search_route_[search_index_], search_speed_m_s_);
      return;
    }
    if (try_enter_align(false) || try_enter_align(true)) {
      return;
    }
    fail_and_return("Search finished without lockable buckets");
  }

  /// 锁定成功 → 排序冻结目标集 → 进入 ALIGN（degraded=true 允许"2 筒 + 1 未知"）。
  bool try_enter_align(bool degraded)
  {
    const auto lock = bucket_map_.try_lock(now().seconds(), degraded);
    if (!lock.ok) {
      // A3：锁定失败原因不再静默丢弃（blocking_reason 可观测性，11 册 §13.1）
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
        "锁定未就绪(%s): %s", degraded ? "degraded" : "normal", lock.reason.c_str());
      return false;
    }
    payload_targets_ = cuadc_drop::sorted_targets(lock.targets, drop_order_);
    if (payload_targets_.size() < static_cast<std::size_t>(kNPayloads)) {
      RCLCPP_WARN(get_logger(),
        "锁定目标 %zu 个 < 两瓶需求，仅按可用目标投放", payload_targets_.size());
    }
    payload_index_ = 0;
    tracker_ = std::make_unique<cuadc_drop::TargetTracker>(
      &payload_targets_[payload_index_], track_params_);
    align_stage_coarse_ = true;
    align_stable_since_.reset();
    align_enter_time_ = now();
    RCLCPP_INFO(get_logger(), "目标集冻结(%s, %zu 筒)，首目标 (%.2f, %.2f) d=%.2f",
      degraded ? "degraded" : "normal", payload_targets_.size(),
      payload_targets_[0].frozen_x, payload_targets_[0].frozen_y,
      payload_targets_[0].frozen_diameter);
    enter(State::ALIGN);
    return true;
  }

  /// 对准（SSOT §5.2/§5.4）：粗 0.15m → 精 0.08m → 稳定 0.8s → 冻结瞄准点进 RELEASE。
  /// 丢视觉只允许一次重捕获（TargetTracker），再丢 = 弃桶拉黑；绝不按冻结坐标盲投。
  void update_align()
  {
    if (!tracker_) {
      fail_and_return("ALIGN without tracker");
      return;
    }
    const double t = now().seconds();
    tracker_->update(latest_detections_, t);
    if (tracker_->assess(t) == cuadc_drop::Assess::kAbandon) {
      RCLCPP_WARN(get_logger(), "目标 %d 丢视觉且重捕获机会已用尽，弃桶拉黑",
        tracker_->target->tid);
      abandon_current_bucket(t);
      return;
    }
    const auto & tg = *tracker_->target;
    target_ = Point3{tg.working_x, tg.working_y, home_->z + align_alt_m_};
    // 速度伺服（SSOT §5.2-209，参数化默认关）：目标失新 watchdog 之外自动
    // 回退位置 setpoint 保持（servo_active_=false → tick 恢复发位置）
    servo_active_ = align_servo_enabled_ && target_fresh(t);
    if (servo_active_) {
      publish_servo_velocity(target_, align_servo_max_align_m_s_);
    }
    const double err = distance_xy(position_, target_);
    if (align_stage_coarse_) {
      if (err <= align_coarse_radius_m_) {
        align_stage_coarse_ = false;               // 粗 → 精
        align_stable_since_.reset();
      }
    } else if (err <= align_fine_radius_m_) {
      if (!align_stable_since_.has_value()) {
        align_stable_since_ = now();
      } else if ((now() - *align_stable_since_).seconds() >= align_stable_s_) {
        freeze_release_aim();                      // 精对准保持到位 → 冻结投放
        return;
      }
    } else {
      align_stable_since_.reset();
    }
    if ((now() - align_enter_time_).seconds() > align_timeout_s_) {
      RCLCPP_WARN(get_logger(), "目标 %d 对准超时(%.1fs)，弃桶拉黑",
        tracker_->target->tid, (now() - align_enter_time_).seconds());
      abandon_current_bucket(t);
    }
  }

  /// 冻结瞄准点 = 冻结时刻新鲜活动估计 + §6 标定偏置 + 弹道前移（sanity 钳位）。
  void freeze_release_aim()
  {
    const auto & gp = release_gate_.params();
    const auto aim = cuadc_drop::aim_point(*tracker_->target,
      payload_offset_xy_[payload_index_].first,
      payload_offset_xy_[payload_index_].second,
      mission_yaw_, vx_local_, vy_local_, gp);
    aim_point_ = Point3{aim.ax, aim.ay, home_->z + align_alt_m_};
    release_gate_.reset();
    sequencer_ = cuadc_drop::DropSequencer{};
    servo_release_pending_ = false;
    servo_reject_streak_ = 0;      // A4-1：新载荷重置拒绝连击
    servo_fire_unknown_ = false;
    servo_cmd_is_fire_ = false;
    target_ = aim_point_;                          // 冻结飞机位置（SSOT §5.2）
    RCLCPP_INFO(get_logger(), "瞄准点冻结 (%.2f, %.2f)，进入释放门控",
      aim_point_.x, aim_point_.y);
    {
      std::ostringstream f;
      f << std::fixed << std::setprecision(3)
        << "\"payload\":" << payload_index_
        << ",\"tid\":" << tracker_->target->tid
        << ",\"ax\":" << aim_point_.x << ",\"ay\":" << aim_point_.y;
      append_event("aim_frozen", f.str());
    }
    enter(State::RELEASE);
  }

  /// 触发单发投放（gate kFire 与 A4-1 拒绝重投共用的唯一入口；单发保护在
  /// DropSequencer::fire 内部——gate 幂等 + 非 IDLE 拒绝，双保险）。
  void fire_payload(double t)
  {
    if (!sequencer_.fire(t)) {
      return;
    }
    RCLCPP_INFO(get_logger(), "释放门控通过：载荷 %zu → 舵机 CH%d %.0fµs%s",
      payload_index_, servo_channel_[payload_index_], servo_release_pwm_,
      enable_release_output_ ? "" : "（干跑，不发指令）");
    if (enable_release_output_) {
      servo_release_pending_ = true;
      servo_cmd_deadline_ = now() + rclcpp::Duration::from_seconds(2.0);
    }
    {
      std::ostringstream f;
      f << "\"payload\":" << payload_index_
        << ",\"ch\":" << servo_channel_[payload_index_]
        << ",\"pwm\":" << servo_release_pwm_
        << ",\"dry_run\":" << (enable_release_output_ ? "false" : "true");
      append_event("fire", f.str());
    }
    if (sim_release_bridge_) {
      request_sim_release();   // 仿真判分：虚拟判定节点按此刻位置判 A/B 区
    }
  }

  /// 释放（SSOT §5.2 释放九门控）：全过且连续稳定+保持才 fire；6s 超时弃桶。
  /// 反盲投由 target_age 门限保证——目标估计超龄永远到不了 fire。
  /// A4-1：fire 指令被 FCU 拒绝 = 瓶未脱，回退 sequencer 重投且计数不推进
  /// （旧实现拒绝只打 WARN 照样计数，真投虚报）；超时=结果未知，计数打标。
  void update_release()
  {
    if (!tracker_) {
      fail_and_return("RELEASE without tracker");
      return;
    }
    const double t = now().seconds();
    tracker_->update(latest_detections_, t);       // 释放段持续复核目标
    cuadc_drop::GateSample s;
    s.horiz_err_m = distance_xy(position_, aim_point_);
    s.vert_err_m = position_.z - aim_point_.z;
    s.hspeed_m_s = horizontal_speed_m_s_;
    s.vspeed_m_s = vertical_speed_m_s_;
    s.tilt_deg = tilt_deg_;
    s.angular_rate_deg_s = angular_rate_deg_s_;
    s.yaw_err_deg = yaw_err_deg_;
    s.target_age_s = t - tracker_->target->last_vision_t;
    s.vx_m_s = vx_local_;
    s.vy_m_s = vy_local_;
    s.t = t;
    const auto verdict = release_gate_.feed(s);
    target_ = aim_point_;
    servo_active_ = align_servo_enabled_ && target_fresh(t);
    if (servo_active_) {
      publish_servo_velocity(aim_point_, align_servo_max_release_m_s_);
    }

    if (verdict.status == cuadc_drop::GateStatus::kFire) {
      fire_payload(t);
    }
    // A4-1 重投路径：fire 被拒后 sequencer 已回退 IDLE，而 gate 幂等只发一次
    // kFire——按"gate 已放行 + sequencer 空闲 + 无在途发送"补一次触发。
    if (verdict.status == cuadc_drop::GateStatus::kFired && enable_release_output_ &&
      !servo_release_pending_ && sequencer_.is_idle())
    {
      fire_payload(t);
    }
    if (servo_release_pending_) {
      if (send_servo(servo_channel_[payload_index_], servo_release_pwm_, true)) {
        servo_release_pending_ = false;
      } else if (now() > servo_cmd_deadline_) {
        fail_and_return("Servo release command could not be sent");
        return;
      }
    }
    // A4-1：0.7s 保持到期但 fire 结果未回（ACK 通常 <100ms）→ 先等结果再走
    // kStow，避免"拒绝也计数"；结果最多等 kServiceTimeoutS 超时清理。
    if (enable_release_output_ && sequencer_.hold_elapsed(t) && command_future_.valid()) {
      return;
    }
    if (sequencer_.tick(t) == cuadc_drop::SeqResult::kStow) {
      if (enable_release_output_) {
        servo_stow_pending_ = true;
        servo_stow_channel_ = servo_channel_[payload_index_];   // 回仓逐拍重试
      }
      const bool fire_unknown = servo_fire_unknown_;
      if (fire_unknown) {
        ++payload_result_unknown_;
        RCLCPP_WARN(get_logger(),
          "载荷 %zu fire 结果超时未知，计数按已投推进并打标（累计未知 %d）",
          payload_index_, payload_result_unknown_);
        servo_fire_unknown_ = false;
      }
      finish_payload(t, fire_unknown);
      return;
    }
    if (verdict.status == cuadc_drop::GateStatus::kAbort) {
      RCLCPP_WARN(get_logger(), "释放门控弃桶(目标 %d)：%s",
        tracker_->target->tid, verdict.reason.c_str());
      abandon_current_bucket(t);
    }
  }

  /// 弃桶拉黑（已投/已弃同一防线：0.25m/120s 内绝不回锁）。
  void abandon_current_bucket(double t)
  {
    if (tracker_) {
      bucket_map_.blacklist(
        tracker_->target->frozen_x, tracker_->target->frozen_y, t);
    }
    append_event("bucket_abandoned",
      "\"tid\":" + std::to_string(tracker_ ? tracker_->target->tid : -1));
    advance_payload_or_finish();
  }

  void finish_payload(double t, bool fire_unknown)
  {
    ++payloads_released_;
    RCLCPP_INFO(get_logger(), "载荷 %zu 投放完成（累计 %d/%d）",
      payload_index_, payloads_released_, kNPayloads);
    append_event("payload_done", "\"count\":" + std::to_string(payloads_released_) +
      ",\"result_unknown\":" + (fire_unknown ? "true" : "false"));
    if (tracker_) {
      bucket_map_.blacklist(
        tracker_->target->frozen_x, tracker_->target->frozen_y, t);
    }
    advance_payload_or_finish();
  }

  /// 下一瓶（范围 = kNPayloads）：取排序里下一个未拉黑目标；没有则转侦察段。
  void advance_payload_or_finish()
  {
    const double t = now().seconds();
    for (std::size_t i = payload_index_ + 1;
      i < payload_targets_.size() && i < static_cast<std::size_t>(kNPayloads); ++i)
    {
      if (bucket_map_.is_blacklisted(
          payload_targets_[i].frozen_x, payload_targets_[i].frozen_y, t))
      {
        continue;
      }
      payload_index_ = i;
      tracker_ = std::make_unique<cuadc_drop::TargetTracker>(
        &payload_targets_[payload_index_], track_params_);
      align_stage_coarse_ = true;
      align_stable_since_.reset();
      align_enter_time_ = now();
      enter(State::ALIGN);
      return;
    }
    RCLCPP_INFO(get_logger(), "载荷全部处理完毕，转侦察段");
    start_recon_climb();
  }

  /// 回仓指令重试：舵机停在释放位是机械风险（可能影响第二瓶/降落），
  /// 必须逐拍重试到服务恢复，不设超时放弃。
  void service_pending_servo()
  {
    if (!servo_stow_pending_) {
      return;
    }
    if (send_servo(servo_stow_channel_, servo_stowed_pwm_, false)) {
      servo_stow_pending_ = false;
    }
  }

  /// 舵机指令（MAV_CMD_DO_SET_SERVO）：独立 0.3s 节流（释放→回仓仅隔 0.7s，
  /// 不能用全局 1s 节流）。A4-2：上一条指令结果未回时返回 false（串行化，
  /// 防止覆盖未决 future）；is_fire 归因结果——拒绝→回退重投，成功→清连击。
  bool send_servo(int channel, double pwm, bool is_fire)
  {
    if (command_future_.valid()) {
      return false;   // A4-2：上一条指令结果未回，下拍重试
    }
    if (now() - last_servo_time_ < rclcpp::Duration::from_seconds(0.3)) {
      return false;
    }
    if (!command_client_->service_is_ready()) {
      return false;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    req->command = 183;                 // MAV_CMD_DO_SET_SERVO
    req->confirmation = false;
    req->param1 = static_cast<float>(channel);
    req->param2 = static_cast<float>(pwm);
    auto far = command_client_->async_send_request(req);
    command_req_id_ = far.request_id;
    command_sent_at_ = now();
    command_future_ = far.future.share();
    servo_cmd_is_fire_ = is_fire;
    last_servo_time_ = now();
    RCLCPP_INFO(get_logger(), "SERVO CH%d → %.0fµs", channel, pwm);
    return true;
  }

  void start_recon_climb()
  {
    start_segment(
      position_, Point3{position_.x, position_.y, home_->z + recon_alt_m_},
      transit_speed_m_s_);
    enter(State::RECON_CLIMB);
  }

  void update_recon_survey()
  {
    target_ = sample_segment();
    if (!segment_complete(waypoint_accept_radius_m_)) {
      return;
    }
    // 到达视点：请求拍照（只发一次，防重复）
    if (!photo_requested_) {
      request_recon_photo(recon_index_);
      photo_requested_ = true;
      photo_request_time_ = now();
      return;
    }
    // 等拍照确认；侦察节点未在线时按 recon_photo_timeout_s 兜底放行
    if (!recon_ack_received(recon_index_) &&
      (now() - photo_request_time_).seconds() < recon_photo_timeout_s_)
    {
      return;
    }
    photo_requested_ = false;
    ++recon_index_;
    if (recon_index_ >= recon_route_.size()) {
      start_return();
      return;
    }
    start_segment(position_, recon_route_[recon_index_], recon_speed_m_s_);
  }

  void start_return()
  {
    if (!home_.has_value()) {
      fail("Return requested without home");
      return;
    }
    // 先爬到返航高度再横越场地（SSOT §5.1 RETURN_CLIMB）
    start_segment(
      position_, Point3{position_.x, position_.y, home_->z + return_alt_m_},
      transit_speed_m_s_);
    enter(State::RETURN_CLIMB);
  }

  // ===========================================================================
  // 航段与航线平滑：时间插值 + smoothstep 缓入缓出
  // ===========================================================================
  void start_segment(const Point3 & start, const Point3 & end, double speed)
  {
    const double duration = std::max(0.5, distance_xyz(start, end) / std::max(0.1, speed));
    segment_ = Segment{start, end, now(), duration};
  }

  /// 当前段的插值点：进度 t 用 smoothstep = t²(3−2t)，段首加速段尾减速
  Point3 sample_segment() const
  {
    const double t = std::clamp(
      (now() - segment_.start_time).seconds() / segment_.duration_s, 0.0, 1.0);
    const double smooth = t * t * (3.0 - 2.0 * t);
    return Point3{
      segment_.start.x + smooth * (segment_.end.x - segment_.start.x),
      segment_.start.y + smooth * (segment_.end.y - segment_.start.y),
      segment_.start.z + smooth * (segment_.end.z - segment_.start.z)};
  }

  /// 段完成 = 时间到 且 实际位置进入接受半径（双条件，防吹偏）
  bool segment_complete(double radius) const
  {
    return (now() - segment_.start_time).seconds() >= segment_.duration_s &&
      distance_xyz(position_, segment_.end) <= radius;
  }

  // ===========================================================================
  // 安全门禁（TODO(M4)：safety_monitor 独立进程做第二道防线）
  // ===========================================================================
  bool navigation_ready() const
  {
    // ground_amsl_valid_：RTK fix 可得（2026-10-08 全切 GNSS 定案——无 fix 即
    // 不锁定任务坐标系，fail-closed，室内无 RTK 不可飞）
    return fcu_state_.connected && have_odom_ && have_compass_ && odom_fresh() &&
      ground_amsl_valid_;
  }

  bool odom_fresh() const
  {
    return have_odom_ && (now() - last_odom_time_).seconds() <= odom_timeout_s_;
  }

  /// RTK 高度遥测新鲜度（飞行门禁用；超时=gps_timeout_s）
  bool gps_fresh() const
  {
    return have_gps_ && (now() - last_gps_time_).seconds() <= gps_timeout_s_;
  }

  /// 每个飞行态更新前的安全门禁：掉连接/掉解锁 → 中止；GUIDED 丢 → 接管；
  /// 定位过期 → 有序返航（require_odom=false 供 TAKEOFF 用：起飞由 FC 自飞，
  /// 遥测断流只延后状态转换，不中止已下发的起飞——60s 超时兜底）
  bool flight_gate_ok(bool require_odom = true)
  {
    if (!fcu_state_.connected || !fcu_state_.armed) {
      fail("FCU disconnected or disarmed during mission");
      return false;
    }
    if (!guided_active_) {
      enter(State::PILOT_OVERRIDE);
      return false;
    }
    if (require_odom && (!odom_fresh() || !gps_fresh())) {
      fail_and_return("Odometry/GPS stale during mission");
      return false;
    }
    return true;
  }

  double relative_altitude() const
  {
    // 任务相对高度（用户定案）：= 实际高度 − 基准高度；
    // 基准=解锁上升沿冻结的 gps_base_alt_m_（2026-10-09 改：原上电冻结跨不过
    // 锁框架→解锁的 GNSS 漂移）。基准未冻结时恒 0——飞不飞得起来先过门禁
    return gps_base_frozen_ ? ground_amsl_m_ - gps_base_alt_m_ : 0.0;
  }

  /// 自主飞行态清单：接管保护只在这些状态触发
  bool autonomous_state(State s) const
  {
    return s == State::WAIT_ARM || s == State::TAKEOFF || s == State::SEARCH ||
      s == State::ALIGN || s == State::RELEASE || s == State::RECON_CLIMB ||
      s == State::RECON_SURVEY || s == State::RETURN_CLIMB || s == State::RETURN_HOME;
  }

  bool returning_or_landed() const
  {
    return state_ == State::RETURN_CLIMB || state_ == State::RETURN_HOME ||
      state_ == State::LAND || state_ == State::DISARM || state_ == State::DONE ||
      state_ == State::PILOT_OVERRIDE || state_ == State::ABORT;
  }

  /// landed_state 新鲜度（EXTENDED_SYS_STATE 断流窗）
  bool extended_state_fresh() const
  {
    return have_extended_state_ &&
      (now() - last_extended_state_time_).seconds() <= extended_state_timeout_s_;
  }

  /// FC 自报在地面（落地检测器权威输出）
  bool on_ground_reported() const
  {
    return extended_state_fresh() &&
      extended_state_.landed_state ==
      mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND;
  }

  /// 落地判定（2026-10-08 采纳哈工大 hgd_cudac 8_20 版三级设计，实飞版 v16 同款；配合 LAND 态
  /// 稳定 1.5s 确认）：
  /// ①主判据 = FC 自身 landed_state——落地检测器内部量，与 GNSS 垂直精度无关，
  ///   基本 GPS 下照常工作（解我方原红线②死结）；明确的非地面状态一律判否，
  ///   绝不允许高度读数把空中报告翻转成 DISARM（哈工大同款防线）；
  /// ②兜底（landed_state 缺失/过期）= odom 相对高（FC 闭环同载体，免疫 GNSS
  ///   噪声）硬钳 ≤min(0.30 参数, 0.10) + 速度门限——此路彻底不再依赖 GNSS
  ///   相对高（原 GNSS 0.30m 判据单点解下 ±2~4m 噪声不可用）；
  /// ③FC 报在地面时连 odom 断流都不推翻——飞控落地检测不依赖机载里程计。
  bool landing_candidate() const
  {
    const bool velocity_low =
      horizontal_speed_m_s_ <= landing_hspeed_gate_m_s_ &&
      std::abs(vertical_speed_m_s_) <= landing_vspeed_gate_m_s_;
    if (extended_state_fresh()) {
      if (on_ground_reported()) {
        return !odom_fresh() || velocity_low;
      }
      if (extended_state_.landed_state !=
        mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED)
      {
        return false;
      }
    }
    const double alt = position_.z - home_.value_or(position_).z;
    return odom_fresh() &&
      std::abs(alt) <= std::min(landing_alt_gate_m_, 0.10) &&
      velocity_low;
  }

  /// LOCK_FRAME 判定：航向在窗口期内漂移 ≤tol 才算锁定，超限则重新计时
  bool heading_locked()
  {
    const double drift = std::fabs(diff_degrees(current_compass_deg_, heading_lock_ref_deg_));
    if (drift > heading_lock_tol_deg_) {
      heading_lock_ref_deg_ = current_compass_deg_;
      heading_lock_since_ = now();
      return false;
    }
    if (!heading_lock_since_.has_value()) {
      heading_lock_ref_deg_ = current_compass_deg_;
      heading_lock_since_ = now();
      return false;
    }
    return (now() - *heading_lock_since_).seconds() >= heading_lock_window_s_;
  }

  /// LOCK_FRAME 位置静止判据（2026-10-08 借鉴 hgd ws_offboard）：水平速度 ≤阈值
  /// 持续与航向同窗；超速即重新计时。
  bool position_still()
  {
    if (horizontal_speed_m_s_ > lock_position_max_speed_m_s_) {
      still_since_.reset();
      return false;
    }
    if (!still_since_.has_value()) {
      still_since_ = now();
      return false;
    }
    return (now() - *still_since_).seconds() >= heading_lock_window_s_;
  }

  // ===========================================================================
  // 目标点发布与飞控指令（全部异步 + 1s 节流，绝不阻塞状态机）
  // ===========================================================================
  void publish_setpoint()
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.pose.position.x = target_.x;
    msg.pose.position.y = target_.y;
    msg.pose.position.z = target_.z;
    msg.pose.orientation.z = yaw_qz_;  // 纯偏航四元数：全程锁头不转机头
    msg.pose.orientation.w = yaw_qw_;
    setpoint_pub_->publish(msg);
  }

  /// 速度伺服（SSOT §5.2-209）：v = Kp·误差（世界系 ENU），死区内置零，
  /// 水平限幅 vmax（对准 0.8 / 释放定位 0.25），垂直限幅 ±0.5 温和降升。
  /// 20Hz 节拍 ≥10Hz 要求；目标失新由 target_fresh() 把总闸（500ms 看护）。
  void publish_servo_velocity(const Point3 & aim, double vmax)
  {
    const double ex = aim.x - position_.x;
    const double ey = aim.y - position_.y;
    const double ez = aim.z - position_.z;
    const double e = std::hypot(ex, ey);
    double vx = 0.0;
    double vy = 0.0;
    if (e > align_servo_deadzone_m_) {
      const double v = std::min(align_servo_kp_ * e, vmax);
      vx = v * ex / e;
      vy = v * ey / e;
    }
    const double vz = std::clamp(align_servo_kp_ * ez, -0.5, 0.5);
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";   // MAVROS setpoint 族统一 ENU local（符号在 SITL 闭环复核）
    msg.twist.linear.x = vx;
    msg.twist.linear.y = vy;
    msg.twist.linear.z = vz;
    msg.twist.angular.z = 0.0;     // 全程锁头，不转机头（SSOT §10.2-1）
    velocity_pub_->publish(msg);
  }

  /// 伺服看护：目标估计失新超 watchdog 即判不新鲜（回退位置保持）
  bool target_fresh(double t) const
  {
    return tracker_ && (t - tracker_->target->last_vision_t) <= align_servo_watchdog_s_;
  }

  /// 解锁上升沿：只做起飞 AMSL 健康门冻结（fail-closed）。任务高度体系与解锁沿
  /// 无关——实际=RTK 推送、基准=上电冻结（lock_frame 完成），2026-10-08 用户定案。
  /// 若节点晚于解锁启动（首帧即 armed=true）则无上升沿→AMSL 门不冻结→
  /// takeoff 拒发（fail-closed，落地重解锁即恢复）
  void on_armed_edge()
  {
    if (ground_amsl_valid_) {
      takeoff_base_amsl_ = ground_amsl_m_;
      takeoff_base_frozen_ = true;
      // 任务高度基准=解锁时定位高度（用户定案 2026-10-09）：解锁沿重冻 GNSS 基准，
      // 此刻起 任务相对高度=实际−本值；之后漂移由「同源同刻」口径抵消
      gps_base_alt_m_ = ground_amsl_m_;
      gps_base_frozen_ = true;
    }
    RCLCPP_INFO(get_logger(),
      "解锁: 起飞AMSL=%.1fm%s, GNSS基准=%.2fm（解锁沿冻结）, 当前任务相对高=%.2fm",
      takeoff_base_amsl_, takeoff_base_frozen_ ? "" : "（无效:无海拔采样）",
      gps_base_alt_m_, relative_altitude());
  }

  /// 视觉起飞门禁（SSOT §5.4：起飞前 ≥3 帧有效；无视觉模式/关闭时不拦）
  bool vision_takeoff_gate_ok() const
  {
    if (m1_no_vision_mode_ || vision_takeoff_min_frames_ <= 0) {
      return true;
    }
    return vision_valid_frames_ >= vision_takeoff_min_frames_;
  }

  /// 仿真投放判分桥：调虚拟判定节点 /drop_controller/release（Trigger）
  void request_sim_release()
  {
    if (!sim_release_client_->service_is_ready() || sim_release_future_.valid()) {
      return;
    }
    auto far = sim_release_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
    sim_rel_req_id_ = far.request_id;
    sim_rel_sent_at_ = now();
    sim_release_future_ = far.future.share();
  }

  bool request_allowed() const
  {
    return (now() - last_request_time_).seconds() >= 1.0;
  }

  void request_takeoff()
  {
    if (!request_allowed() || takeoff_future_.valid() || !takeoff_client_->service_is_ready()) {
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    if (!takeoff_base_frozen_) {
      // 冻结=锁框架时 GPS 采样可用，此处保留为位置源健康门（fail-closed），
      // 基准值本身已不进命令（本固件 param7=相对爬升量）
      RCLCPP_WARN(get_logger(), "takeoff 拒发：地面海拔基准尚未冻结（锁框架前不起飞）");
      return;
    }
    // 本固件（4.7.1/dbe79216）GUIDED 起飞 param7=相对当前高度的爬升量（frame 强制
    // GLOBAL_RELATIVE_ALT，complete_alt=start_alt+z）——发 AGL 爬升量；发海拔会被按
    // 「再爬 290m」解释（205 号日志 24.9m 冲高定案，2026-10-07）
    req->altitude = static_cast<float>(takeoff_alt_m_);
    RCLCPP_INFO(get_logger(), "takeoff 命令: 相对爬升 %.1fm（AGL；地面基准 %.1fm 仅监控）",
      req->altitude, takeoff_base_amsl_);
    req->yaw = static_cast<float>(locked_compass_deg_);
    last_request_time_ = now();
    auto far = takeoff_client_->async_send_request(req);
    takeoff_req_id_ = far.request_id;
    takeoff_sent_at_ = now();
    takeoff_future_ = far.future.share();
    takeoff_sent_ = true;
  }

  void request_land()
  {
    if (!request_allowed() || land_future_.valid() || !land_client_->service_is_ready()) {
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    req->yaw = static_cast<float>(locked_compass_deg_);
    last_request_time_ = now();
    auto far = land_client_->async_send_request(req);
    land_req_id_ = far.request_id;
    land_sent_at_ = now();
    land_future_ = far.future.share();
  }

  void request_arm(bool arm)
  {
    if (!request_allowed() || arm_future_.valid() || !arm_client_->service_is_ready()) {
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = arm;
    last_request_time_ = now();
    auto far = arm_client_->async_send_request(req);
    arm_req_id_ = far.request_id;
    arm_sent_at_ = now();
    arm_future_ = far.future.share();
  }

  /// A1-1：future 在途超过 kServiceTimeoutS 未回 → 清理客户端 pending 并报告。
  /// 清理≠撤销远端动作，结果语义=未知，由调用方按各服务的兜底路径处理。
  /// 返回 true 表示 future 已被超时清理，调用方跳过本轮结果检查。
  template<typename SrvT>
  bool service_timed_out(rclcpp::Client<SrvT> & client,
    const typename rclcpp::Client<SrvT>::SharedFuture & future,
    int64_t & request_id, const rclcpp::Time & sent_at, const char * name)
  {
    if (!future.valid() ||
      future.wait_for(0s) == std::future_status::ready ||
      (now() - sent_at).seconds() < kServiceTimeoutS)
    {
      return false;
    }
    client.remove_pending_request(request_id);
    RCLCPP_WARN(get_logger(),
      "%s 服务 %.1fs 无 ACK：已清理 pending（结果=未知，远端动作不撤回）",
      name, kServiceTimeoutS);
    return true;
  }

  /// 非阻塞回收服务结果（wait_for(0s) 只看一眼绝不等待）。
  /// A1-1：每个在途请求都超时清理（防 future 恒 valid 卡死重发）；超时=结果
  /// 未知——起飞/降落/上锁按遥测与节流重发兜底，舵机 fire 走 unknown 打标。
  void check_service_results()
  {
    // MAV_RESULT 结果码→人读名（MAVLink spec；2026-10-06 补：此前 takeoff 被静默拒绝
    // 时结果码被丢弃，排障三日不可见——ACK 必须留痕）
    auto mav_result_name = [](int8_t r) -> const char* {
      switch (r) {
        case 0: return "ACCEPTED";
        case 1: return "TEMPORARILY_REJECTED";
        case 2: return "DENIED";
        case 3: return "UNSUPPORTED";
        case 4: return "FAILED";
        case 5: return "IN_PROGRESS";
        case 6: return "CANCELLED";
        default: return "UNKNOWN";
      }
    };
    if (service_timed_out(*takeoff_client_, takeoff_future_, takeoff_req_id_,
        takeoff_sent_at_, "takeoff"))
    {
      takeoff_future_ = {};
      takeoff_sent_ = false;   // 结果未知：允许重发（TAKEOFF 有高度门+超时兜底）
    }
    if (takeoff_future_.valid() && takeoff_future_.wait_for(0s) == std::future_status::ready) {
      const auto res = takeoff_future_.get();
      if (res->success) {
        RCLCPP_INFO(get_logger(), "takeoff ACK: 接受（result=%s）",
          mav_result_name(res->result));
      } else {
        RCLCPP_WARN(get_logger(), "takeoff ACK: 被拒 success=0 result=%s —— 按节流重发，超时兜底",
          mav_result_name(res->result));
        takeoff_sent_ = false;
      }
      takeoff_future_ = {};
    }
    if (service_timed_out(*land_client_, land_future_, land_req_id_,
        land_sent_at_, "land"))
    {
      land_future_ = {};       // LAND 态按节流重发，结果未知由落地遥测把守
    }
    if (land_future_.valid() && land_future_.wait_for(0s) == std::future_status::ready) {
      const auto res = land_future_.get();
      if (res->success) {
        RCLCPP_INFO(get_logger(), "land ACK: 接受（result=%s）",
          mav_result_name(res->result));
      } else {
        RCLCPP_WARN(get_logger(), "land ACK: 被拒 success=0 result=%s —— 按节流重发",
          mav_result_name(res->result));
      }
      land_future_ = {};
    }
    if (service_timed_out(*arm_client_, arm_future_, arm_req_id_,
        arm_sent_at_, "arm"))
    {
      arm_future_ = {};        // DISARM/LAND 态按节流重发
    }
    if (arm_future_.valid() && arm_future_.wait_for(0s) == std::future_status::ready) {
      const auto res = arm_future_.get();
      if (res->success) {
        RCLCPP_INFO(get_logger(), "arm ACK: 接受（result=%s）",
          mav_result_name(res->result));
      } else {
        RCLCPP_WARN(get_logger(), "arm ACK: 被拒 success=0 result=%s —— 按节流重发",
          mav_result_name(res->result));
      }
      arm_future_ = {};
    }
    if (service_timed_out(*command_client_, command_future_, command_req_id_,
        command_sent_at_, "command"))
    {
      command_future_ = {};
      if (servo_cmd_is_fire_) {
        // fire 结果未知（远端可能已执行）：DO_SET_SERVO 幂等，计数按已投
        // 推进但在 kStow 处打标（payload_result_unknown_）
        servo_cmd_is_fire_ = false;
        servo_fire_unknown_ = true;
        append_event("servo_result", "\"result\":\"timeout\"");
        RCLCPP_WARN(get_logger(),
          "舵机 fire 指令 %.1fs 无 ACK：结果未知，计数按已投推进并打标",
          kServiceTimeoutS);
      }
    }
    if (command_future_.valid() &&
      command_future_.wait_for(0s) == std::future_status::ready)
    {
      const auto res = command_future_.get();
      command_future_ = {};
      const bool was_fire = servo_cmd_is_fire_;
      servo_cmd_is_fire_ = false;
      if (res->success) {
        if (was_fire) {
          servo_reject_streak_ = 0;    // fire 被接受：连击清零
          append_event("servo_result", "\"result\":\"accepted\"");
        }
      } else if (was_fire) {
        ++servo_reject_streak_;
        // A4-1：拒绝=瓶未脱，回退 sequencer 重投（update_release 补触发），
        // 计数不推进——旧实现这里只 WARN 照样计数，真投虚报
        if (sequencer_.abort_fire()) {
          RCLCPP_ERROR(get_logger(),
            "舵机 fire 指令被 FCU 拒绝（连续第 %d 次）：瓶未脱，已回退重投"
            "（计数不推进）", servo_reject_streak_);
          append_event("servo_result",
            "\"result\":\"rejected\",\"streak\":" + std::to_string(servo_reject_streak_));
          if (servo_reject_streak_ >= 3) {
            fail_and_return("Servo fire command rejected 3 times by FCU");
            return;
          }
        } else {
          RCLCPP_WARN(get_logger(),
            "舵机 fire 结果=拒绝，但 sequencer 已不可回退（已 finish/复位），按告警处理");
        }
      } else {
        RCLCPP_WARN(get_logger(), "舵机回仓指令被 FCU 拒绝，下拍重试");
      }
    }
    if (service_timed_out(*sim_release_client_, sim_release_future_,
        sim_rel_req_id_, sim_rel_sent_at_, "sim_release"))
    {
      sim_release_future_ = {};
    }
    if (sim_release_future_.valid() &&
      sim_release_future_.wait_for(0s) == std::future_status::ready)
    {
      const auto res = sim_release_future_.get();
      // 虚拟判定节点 message 带"误差/分区/得分"，M3 仿真验收脚本按此采集
      RCLCPP_INFO(get_logger(), "仿真投放判定: success=%d %s",
        res->success ? 1 : 0, res->message.c_str());
      sim_release_future_ = {};
    }
  }

  void request_recon_photo(std::size_t index)
  {
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.point.x = static_cast<double>(index);
    if (index < recon_route_.size()) {
      msg.point.y = recon_route_[index].x;  // 视点坐标随请求附带，侦察节点取证用
      msg.point.z = recon_route_[index].y;
    }
    recon_capture_pub_->publish(msg);
  }

  bool recon_ack_received(std::size_t index) const
  {
    return std::find(
      recon_acks_.begin(), recon_acks_.end(),
      static_cast<std::uint32_t>(index)) != recon_acks_.end();
  }

  // ===========================================================================
  // 失败处理与状态切换
  // ===========================================================================

  /// 硬失败：立即降落（掉连接/掉解锁等没有返航余量的情况）
  void fail(const std::string & reason)
  {
    mission_failed_ = true;
    terminal_reason_ = reason;
    RCLCPP_ERROR(get_logger(), "MISSION_ABORT: %s", reason.c_str());
    enter(fcu_state_.armed ? State::LAND : State::ABORT);
  }

  /// 软失败：先有序返航再降落（能回家绝不原地掉）
  void fail_and_return(const std::string & reason)
  {
    mission_failed_ = true;
    terminal_reason_ = reason;
    RCLCPP_ERROR(get_logger(), "MISSION_FAILURE: %s", reason.c_str());
    if (fcu_state_.armed && home_.has_value() && guided_active_ && odom_fresh()) {
      start_return();
    } else {
      enter(fcu_state_.armed ? State::LAND : State::ABORT);
    }
  }

  /// 状态切换唯一入口：记录进入时间 + 广播 + 日志（排障全靠这条轨迹）
  void enter(State next)
  {
    if (state_ == next) {
      return;
    }
    state_ = next;
    state_enter_time_ = now();
    servo_active_ = false;   // 换态必回位置 setpoint，伺服只在对准/释放态内存活
    std_msgs::msg::String msg;
    msg.data = state_name(state_);
    mission_state_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "STATE -> %s", msg.data.c_str());
  }

  // ===========================================================================
  // 成员变量
  // ===========================================================================

  // ---- 通信 ----
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr compass_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr recon_ack_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr bucket_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr global_fix_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr extended_state_sub_;
  // 地面海拔（AMSL）：锁框架时冻结，仅作监控与 request_takeoff 的位置源健康门
  // （起飞命令 param7=相对爬升量，海拔不进命令；10-07 二次修=锁框架冻结）
  double ground_amsl_m_ = 0.0;
  bool ground_amsl_valid_ = false;
  bool takeoff_base_frozen_ = false;
  double takeoff_base_amsl_ = 0.0;
  // 任务高度体系（2026-10-08 用户定案：实际=RTK 推送、基准=上电冻结、相对=差值）。
  // ground_amsl_m_=实际高度（NavSatFix 海拔，Ek3_SRC1_POSZ=3 下与 FC 闭环高度同源）；
  // gps_base_alt_m_=基准高度（lock_frame 一次性冻结）；相对高度=relative_altitude()
  // gps_timeout_s_=RTK 遥测新鲜度门限（飞行门禁 gps_fresh 用）
  double gps_base_alt_m_ = 0.0;
  bool gps_base_frozen_ = false;
  bool have_gps_ = false;
  rclcpp::Time last_gps_time_;
  double gps_timeout_s_ = 2.0;
  // 落地检测（哈工大三级设计）：FC 自报 landed_state 权威 + odom 兜底
  mavros_msgs::msg::ExtendedState extended_state_;
  bool have_extended_state_ = false;
  rclcpp::Time last_extended_state_time_;
  double extended_state_timeout_s_ = 2.5;
  std::optional<bool> was_armed_;   ///< 解锁上升沿检测（首帧不视为边沿）
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr recon_capture_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr buckets_world_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr sim_release_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture takeoff_future_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture land_future_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture arm_future_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture command_future_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture sim_release_future_;

  // ---- M3 投放决策（drop_logic.hpp，逻辑与 tools/drop_logic.py 镜像）----
  cuadc_drop::BucketMap bucket_map_;
  cuadc_drop::TrackParams track_params_;
  cuadc_drop::ReleaseGate release_gate_;
  cuadc_drop::DropSequencer sequencer_;
  std::vector<cuadc_drop::FrozenTarget> payload_targets_;  ///< 锁定后不再增删（指针稳定）
  std::unique_ptr<cuadc_drop::TargetTracker> tracker_;
  std::vector<cuadc_drop::Detection2D> latest_detections_; ///< 最新一帧（世界系）
  Point3 aim_point_;
  std::size_t payload_index_ = 0;
  bool align_stage_coarse_ = true;
  std::optional<rclcpp::Time> align_stable_since_;
  rclcpp::Time align_enter_time_;
  bool have_vision_ = false;
  rclcpp::Time last_vision_time_;
  double tilt_deg_ = 0.0;
  double yaw_err_deg_ = 0.0;
  double vx_local_ = 0.0;
  double vy_local_ = 0.0;
  int perception_bad_frames_ = 0;      ///< 哨兵违约帧计数（排障）
  int perception_stale_frames_ = 0;    ///< P0.4 拒绝帧计数（排障）
  bool servo_release_pending_ = false;
  bool servo_stow_pending_ = false;
  bool servo_active_ = false;          ///< 速度伺服激活拍（tick 停发位置 setpoint）
  int vision_valid_frames_ = 0;        ///< 契约有效感知帧累计（起飞门禁用）
  int servo_stow_channel_ = 9;
  rclcpp::Time servo_cmd_deadline_;
  rclcpp::Time last_servo_time_;
  int payloads_released_ = 0;
  // ---- A4-1/A4-2：舵机指令归因与拒绝防线 ----
  bool servo_cmd_is_fire_ = false;     ///< 在途指令归因（A4-2 串行化后至多一条）
  int servo_reject_streak_ = 0;        ///< 连续 fire 被拒计数（≥3 弃任务）
  bool servo_fire_unknown_ = false;    ///< fire 结果超时未知（计数打标）
  int payload_result_unknown_ = 0;     ///< fire 结果未知的载荷数（DONE 汇总）
  // ---- A1-1：在途请求归因（id 供 remove_pending_request，时刻供超时判定）----
  int64_t takeoff_req_id_ = 0;
  int64_t land_req_id_ = 0;
  int64_t arm_req_id_ = 0;
  int64_t command_req_id_ = 0;
  int64_t sim_rel_req_id_ = 0;
  rclcpp::Time takeoff_sent_at_;
  rclcpp::Time land_sent_at_;
  rclcpp::Time arm_sent_at_;
  rclcpp::Time command_sent_at_;
  rclcpp::Time sim_rel_sent_at_;

  // ---- 状态机运行时 ----
  State state_ = State::WAIT_FCU;
  mavros_msgs::msg::State fcu_state_;
  Point3 position_;
  Point3 target_;
  Segment segment_;
  std::optional<Point3> home_;
  std::vector<Point3> search_route_;
  bool search_hover_active_ = false;     ///< 悬停识别段在途（悬停优先+蛇形兜底）
  std::optional<rclcpp::Time> hover_since_;  ///< 悬停窗口起点（到位起算）
  std::vector<Point3> recon_route_;
  std::vector<std::uint32_t> recon_acks_;
  std::deque<cuadc_odom::Sample> odom_history_;
  cuadc_odom::InterpParams interp_params_;   ///< P0.4 判定参数（odom_interp.hpp）
  std::optional<rclcpp::Time> landing_stable_since_;
  std::optional<rclcpp::Time> heading_lock_since_;
  std::optional<rclcpp::Time> still_since_;     ///< LOCK_FRAME 静止判据计时

  // ---- 布尔标志 ----
  bool guided_active_ = false;
  bool have_odom_ = false;
  bool have_compass_ = false;
  bool frame_locked_ = false;
  bool publish_setpoint_enabled_ = false;
  bool mission_started_ = false;
  bool mission_failed_ = false;
  bool done_logged_ = false;
  bool takeoff_sent_ = false;
  bool photo_requested_ = false;
  bool m1_no_vision_mode_ = true;
  bool auto_arm_on_guided_ = false;

  // ---- 计数器 ----
  std::size_t search_index_ = 0U;
  std::size_t recon_index_ = 0U;
  int search_lanes_ = 6;
  int recon_lanes_ = 3;

  // ---- 参数缓存（默认值 = SSOT §5.4 基线）----
  double takeoff_alt_m_ = 4.0;
  double search_alt_m_ = 2.0;
  double recon_alt_m_ = 4.0;
  double return_alt_m_ = 4.0;
  double transit_speed_m_s_ = 4.0;
  double search_speed_m_s_ = 2.0;
  double recon_speed_m_s_ = 3.0;
  double waypoint_accept_radius_m_ = 0.40;
  double heading_lock_window_s_ = 2.0;
  double heading_lock_tol_deg_ = 2.0;
  double heading_lock_timeout_s_ = 30.0;
  double prestream_hold_s_ = 1.5;
  double takeoff_timeout_s_ = 60.0;
  double mission_timeout_s_ = 240.0;
  double odom_timeout_s_ = 1.0;
  double recon_photo_timeout_s_ = 1.0;
  double landing_confirm_s_ = 1.5;
  double landing_alt_gate_m_ = 0.30;
  double landing_hspeed_gate_m_s_ = 0.20;
  double landing_vspeed_gate_m_s_ = 0.15;
  double search_x_min_m_ = 27.5;
  double search_x_max_m_ = 32.5;
  double search_half_width_m_ = 4.0;
  double search_degrade_timeout_s_ = 60.0;
  double search_settle_s_ = 8.0;
  double search_hover_x_m_ = 30.0;
  double search_hover_y_m_ = 0.0;
  double search_hover_timeout_s_ = 12.0;
  double recon_x_m_ = 52.5;
  double recon_lane_len_m_ = 5.0;
  double recon_half_width_m_ = 4.0;
  double perception_max_delay_s_ = 1.5;
  double odom_future_tol_s_ = 0.05;
  double odom_interp_max_gap_s_ = 0.2;
  double odom_history_span_s_ = 3.0;

  // ---- M3 参数（默认值 = SSOT §5.2/§5.4 基线）----
  std::string expected_frame_id_ = "cuadc_body_flu";
  double contract_version_ = 1.0;
  double min_detection_confidence_ = 0.25;
  double align_alt_m_ = 1.8;
  double align_coarse_radius_m_ = 0.15;
  double align_fine_radius_m_ = 0.08;
  double align_stable_s_ = 0.8;
  double align_timeout_s_ = 12.0;
  std::string drop_order_ = "conservative";
  int vision_takeoff_min_frames_ = 3;
  bool align_servo_enabled_ = false;
  double align_servo_kp_ = 1.0;
  double align_servo_max_align_m_s_ = 0.8;
  double align_servo_max_release_m_s_ = 0.25;
  double align_servo_deadzone_m_ = 0.02;
  double align_servo_watchdog_s_ = 0.5;
  bool sim_release_bridge_ = false;
  std::string event_log_path_;   ///< A4：RELEASE 证据 JSONL 路径（空=禁用）
  std::pair<double, double> payload_offset_xy_[2] = {{0.0, 0.0}, {0.0, 0.0}};
  int servo_channel_[2] = {9, 10};
  double servo_stowed_pwm_ = 1100.0;
  double servo_release_pwm_ = 1700.0;
  bool enable_release_output_ = false;   ///< 干跑开关：false 只打日志不发舵机

  // ---- 坐标系缓存 ----
  double horizontal_speed_m_s_ = 0.0;
  double vertical_speed_m_s_ = 0.0;
  double angular_rate_deg_s_ = 0.0;             ///< odom 三轴角速度模长（九门控输入）
  double current_compass_deg_ = 0.0;
  double current_heading_enu_ = 0.0;
  double mission_yaw_ = 0.0;
  double locked_compass_deg_ = 0.0;
  double heading_lock_ref_deg_ = 0.0;
  double lock_position_max_speed_m_s_ = 0.15;   ///< LOCK_FRAME 静止门限
  double yaw_qz_ = 0.0;
  double yaw_qw_ = 1.0;

  // ---- 时间戳 ----
  rclcpp::Time state_enter_time_;
  rclcpp::Time mission_start_time_;
  rclcpp::Time last_request_time_;
  rclcpp::Time last_odom_time_;
  rclcpp::Time photo_request_time_;

  std::string terminal_reason_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CuadcMissionNode>());
  rclcpp::shutdown();
  return 0;
}
