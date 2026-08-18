// Copyright 2026 Pusan National University, Advanced Robotics Lab
// SPDX-License-Identifier: LicenseRef-PNU-ARL-Academic-Research-Only-1.0
//----------------------------------------------------------------------------
// ROS2 node: subscribe to command topics, command EPOS, publish encoder state
//----------------------------------------------------------------------------

#include <cmath>
#include <chrono>
#include <cstring>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

#include "Definitions.h"

namespace
{
std::string ErrorInfo(unsigned int code)
{
  char errorInfo[256] = {0};
  if (VCS_GetErrorInfo(code, errorInfo, sizeof(errorInfo)) != 0)
  {
    return std::string(errorInfo);
  }
  return std::string();
}

class EposMotionBridgeNode : public rclcpp::Node
{
public:
  EposMotionBridgeNode()
  : Node("epos_motion_bridge_node")
  {
    declare_parameter<int>("node_id_theta", 1);
    declare_parameter<int>("node_id_linear", 2);
    declare_parameter<std::string>("device_name", "EPOS4");
    declare_parameter<std::string>("protocol_stack_name", "CANopen");
    declare_parameter<std::string>("interface_name", "CAN_ix_usb_can 0");
    declare_parameter<std::string>("port_name", "CAN0");
    declare_parameter<int>("baudrate", 1000000);

    declare_parameter<double>("gear_ratio", 590.0);
    declare_parameter<int>("encoder_cpr", 2048);
    declare_parameter<bool>("invert_direction", false);
    declare_parameter<bool>("absolute", true);
    declare_parameter<int>("profile_velocity", 2000);
    declare_parameter<int>("profile_acceleration", 4000);
    declare_parameter<int>("profile_deceleration", 4000);
    declare_parameter<bool>("define_zero_on_start", true);

    declare_parameter<bool>("enable_linear", true);
    declare_parameter<double>("linear_mm_per_rev", 2.0);
    declare_parameter<double>("linear_gear_ratio", 57.0 / 13.0);
    declare_parameter<int>("linear_encoder_cpr", 2048);
    declare_parameter<bool>("linear_invert_direction", false);
    declare_parameter<bool>("linear_absolute", true);
    declare_parameter<int>("linear_profile_velocity", 2000);
    declare_parameter<int>("linear_profile_acceleration", 4000);
    declare_parameter<int>("linear_profile_deceleration", 4000);
    declare_parameter<bool>("linear_define_zero_on_start", true);

    declare_parameter<std::string>("topic_cmd_theta", "/needle/cmd/theta_deg");
    declare_parameter<std::string>("topic_cmd_linear", "/needle/cmd/d_mm");
    declare_parameter<std::string>("topic_cmd_theta_velocity", "/needle/cmd/theta_velocity");
    declare_parameter<std::string>("topic_cmd_linear_velocity", "/needle/cmd/d_velocity");
    declare_parameter<std::string>("topic_execute_fast_mode", "/needle/cmd/execute_fast_mode");
    declare_parameter<std::string>("topic_state_theta", "/needle/state/theta_deg");
    declare_parameter<std::string>("topic_state_linear", "/needle/state/d_mm");
    declare_parameter<int>("fast_profile_velocity", 4000);
    declare_parameter<int>("fast_profile_acceleration", 8000);
    declare_parameter<int>("fast_profile_deceleration", 8000);
    declare_parameter<int>("linear_fast_profile_velocity", 10000);
    declare_parameter<int>("linear_fast_profile_acceleration", 16000);
    declare_parameter<int>("linear_fast_profile_deceleration", 16000);

    declare_parameter<int>("state_period_ms", 30);
    declare_parameter<int>("status_period_ms", 500);

    ready_ = OpenAndConfigure();
    if (!ready_)
    {
      RCLCPP_ERROR(get_logger(), "Failed to open/configure EPOS");
      return;
    }

    auto topic_cmd_theta = get_parameter("topic_cmd_theta").as_string();
    theta_sub_ = create_subscription<std_msgs::msg::Float64>(
      topic_cmd_theta, 10, std::bind(&EposMotionBridgeNode::OnTargetDeg, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Listening cmd theta on %s", topic_cmd_theta.c_str());

    auto topic_state_theta = get_parameter("topic_state_theta").as_string();
    theta_state_pub_ = create_publisher<std_msgs::msg::Float64>(topic_state_theta, 10);
    RCLCPP_INFO(get_logger(), "Publishing state theta on %s", topic_state_theta.c_str());

    auto topic_cmd_theta_velocity = get_parameter("topic_cmd_theta_velocity").as_string();
    theta_velocity_sub_ = create_subscription<std_msgs::msg::Float64>(
      topic_cmd_theta_velocity, 10,
      std::bind(&EposMotionBridgeNode::OnThetaVelocity, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Listening theta velocity on %s", topic_cmd_theta_velocity.c_str());

    auto topic_execute_fast_mode = get_parameter("topic_execute_fast_mode").as_string();
    execute_mode_sub_ = create_subscription<std_msgs::msg::Bool>(
      topic_execute_fast_mode, 10,
      std::bind(&EposMotionBridgeNode::OnExecuteFastMode, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Listening execute mode on %s", topic_execute_fast_mode.c_str());

    if (enable_linear_)
    {
      auto topic_cmd_linear = get_parameter("topic_cmd_linear").as_string();
      linear_sub_ = create_subscription<std_msgs::msg::Float64>(
        topic_cmd_linear, 10, std::bind(&EposMotionBridgeNode::OnTargetLinear, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Listening cmd linear on %s", topic_cmd_linear.c_str());

      auto topic_state_linear = get_parameter("topic_state_linear").as_string();
      linear_state_pub_ = create_publisher<std_msgs::msg::Float64>(topic_state_linear, 10);
      RCLCPP_INFO(get_logger(), "Publishing state linear on %s", topic_state_linear.c_str());

      auto topic_cmd_linear_velocity = get_parameter("topic_cmd_linear_velocity").as_string();
      linear_velocity_sub_ = create_subscription<std_msgs::msg::Float64>(
        topic_cmd_linear_velocity, 10,
        std::bind(&EposMotionBridgeNode::OnLinearVelocity, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Listening linear velocity on %s", topic_cmd_linear_velocity.c_str());
    }

    int state_period_ms = get_parameter("state_period_ms").as_int();
    if (state_period_ms > 0)
    {
      state_timer_ = create_wall_timer(
        std::chrono::milliseconds(state_period_ms),
        std::bind(&EposMotionBridgeNode::PublishState, this));
    }

    int status_period_ms = get_parameter("status_period_ms").as_int();
    if (status_period_ms > 0)
    {
      status_timer_ = create_wall_timer(
        std::chrono::milliseconds(status_period_ms),
        std::bind(&EposMotionBridgeNode::LogStatus, this));
    }
  }

  ~EposMotionBridgeNode() override
  {
    unsigned int errorCode = 0;
    if (key_handle_)
    {
      VCS_SetDisableState(key_handle_, theta_.node_id, &errorCode);
      if (enable_linear_)
      {
        VCS_SetDisableState(key_handle_, linear_.node_id, &errorCode);
      }
      VCS_CloseDevice(key_handle_, &errorCode);
      key_handle_ = nullptr;
    }
  }

  bool IsReady() const
  {
    return ready_;
  }

private:
  struct AxisConfig
  {
    unsigned short node_id = 0;
    int encoder_cpr = 0;
    double gear_ratio = 1.0;
    bool invert_direction = false;
    bool absolute = true;
    int profile_velocity = 0;
    int profile_acceleration = 0;
    int profile_deceleration = 0;
    double units_per_rev = 1.0;
    bool define_zero_on_start = true;
    const char * name = "";
  };

  bool OpenAndConfigure()
  {
    device_name_ = get_parameter("device_name").as_string();
    protocol_stack_name_ = get_parameter("protocol_stack_name").as_string();
    interface_name_ = get_parameter("interface_name").as_string();
    port_name_ = get_parameter("port_name").as_string();
    baudrate_ = get_parameter("baudrate").as_int();

    theta_.node_id = static_cast<unsigned short>(get_parameter("node_id_theta").as_int());
    theta_.gear_ratio = get_parameter("gear_ratio").as_double();
    theta_.encoder_cpr = get_parameter("encoder_cpr").as_int();
    theta_.invert_direction = get_parameter("invert_direction").as_bool();
    theta_.absolute = get_parameter("absolute").as_bool();
    theta_.profile_velocity = get_parameter("profile_velocity").as_int();
    theta_.profile_acceleration = get_parameter("profile_acceleration").as_int();
    theta_.profile_deceleration = get_parameter("profile_deceleration").as_int();
    theta_.define_zero_on_start = get_parameter("define_zero_on_start").as_bool();
    theta_.units_per_rev = 360.0;
    theta_.name = "theta";

    enable_linear_ = get_parameter("enable_linear").as_bool();
    linear_.node_id = static_cast<unsigned short>(get_parameter("node_id_linear").as_int());
    linear_.gear_ratio = get_parameter("linear_gear_ratio").as_double();
    linear_.encoder_cpr = get_parameter("linear_encoder_cpr").as_int();
    linear_.invert_direction = get_parameter("linear_invert_direction").as_bool();
    linear_.absolute = get_parameter("linear_absolute").as_bool();
    linear_.profile_velocity = get_parameter("linear_profile_velocity").as_int();
    linear_.profile_acceleration = get_parameter("linear_profile_acceleration").as_int();
    linear_.profile_deceleration = get_parameter("linear_profile_deceleration").as_int();
    linear_.define_zero_on_start = get_parameter("linear_define_zero_on_start").as_bool();
    linear_.units_per_rev = get_parameter("linear_mm_per_rev").as_double();
    linear_.name = "linear";

    fast_profile_velocity_ = get_parameter("fast_profile_velocity").as_int();
    fast_profile_acceleration_ = get_parameter("fast_profile_acceleration").as_int();
    fast_profile_deceleration_ = get_parameter("fast_profile_deceleration").as_int();
    linear_fast_profile_velocity_ = get_parameter("linear_fast_profile_velocity").as_int();
    linear_fast_profile_acceleration_ = get_parameter("linear_fast_profile_acceleration").as_int();
    linear_fast_profile_deceleration_ = get_parameter("linear_fast_profile_deceleration").as_int();

    char deviceName[255] = {0};
    char protocolStackName[255] = {0};
    char interfaceName[255] = {0};
    char portName[255] = {0};
    std::strncpy(deviceName, device_name_.c_str(), sizeof(deviceName) - 1);
    std::strncpy(protocolStackName, protocol_stack_name_.c_str(), sizeof(protocolStackName) - 1);
    std::strncpy(interfaceName, interface_name_.c_str(), sizeof(interfaceName) - 1);
    std::strncpy(portName, port_name_.c_str(), sizeof(portName) - 1);

    unsigned int errorCode = 0;
    key_handle_ = VCS_OpenDevice(deviceName, protocolStackName, interfaceName, portName, &errorCode);
    if (!key_handle_ || errorCode != 0)
    {
      RCLCPP_ERROR(get_logger(), "OpenDevice failed: 0x%08x %s", errorCode, ErrorInfo(errorCode).c_str());
      return false;
    }

    unsigned int currentBaudrate = 0;
    unsigned int timeout = 0;
    if (VCS_GetProtocolStackSettings(key_handle_, &currentBaudrate, &timeout, &errorCode) == 0)
    {
      RCLCPP_ERROR(get_logger(), "GetProtocolStackSettings failed: 0x%08x %s",
                   errorCode, ErrorInfo(errorCode).c_str());
      return false;
    }
    if (VCS_SetProtocolStackSettings(key_handle_, baudrate_, timeout, &errorCode) == 0)
    {
      RCLCPP_ERROR(get_logger(), "SetProtocolStackSettings failed: 0x%08x %s",
                   errorCode, ErrorInfo(errorCode).c_str());
      return false;
    }

    if (!PrepareAxis(theta_) || !ConfigureAxis(theta_))
    {
      return false;
    }

    if (enable_linear_)
    {
      if (linear_.units_per_rev <= 0.0)
      {
        RCLCPP_ERROR(get_logger(), "linear_mm_per_rev must be > 0");
        return false;
      }
      if (!PrepareAxis(linear_) || !ConfigureAxis(linear_))
      {
        RCLCPP_ERROR(get_logger(), "Linear axis not ready; disabling linear control");
        enable_linear_ = false;
      }
    }

    RCLCPP_INFO(get_logger(), "EPOS bridge ready");
    return true;
  }

  bool PrepareAxis(const AxisConfig & axis)
  {
    unsigned int errorCode = 0;
    int isFault = 0;
    if (VCS_GetFaultState(key_handle_, axis.node_id, &isFault, &errorCode) == 0)
    {
      RCLCPP_ERROR(get_logger(), "%s GetFaultState failed: 0x%08x %s",
                   axis.name, errorCode, ErrorInfo(errorCode).c_str());
      return false;
    }
    if (isFault)
    {
      if (VCS_ClearFault(key_handle_, axis.node_id, &errorCode) == 0)
      {
        RCLCPP_ERROR(get_logger(), "%s ClearFault failed: 0x%08x %s",
                     axis.name, errorCode, ErrorInfo(errorCode).c_str());
        return false;
      }
    }

    int isEnabled = 0;
    if (VCS_GetEnableState(key_handle_, axis.node_id, &isEnabled, &errorCode) == 0)
    {
      RCLCPP_ERROR(get_logger(), "%s GetEnableState failed: 0x%08x %s",
                   axis.name, errorCode, ErrorInfo(errorCode).c_str());
      return false;
    }
    if (!isEnabled)
    {
      if (VCS_SetEnableState(key_handle_, axis.node_id, &errorCode) == 0)
      {
        RCLCPP_ERROR(get_logger(), "%s SetEnableState failed: 0x%08x %s",
                     axis.name, errorCode, ErrorInfo(errorCode).c_str());
        return false;
      }
    }

    return true;
  }

  bool ConfigureAxis(const AxisConfig & axis)
  {
    unsigned int errorCode = 0;
    if (VCS_ActivateProfilePositionMode(key_handle_, axis.node_id, &errorCode) == 0)
    {
      RCLCPP_ERROR(get_logger(), "%s ActivateProfilePositionMode failed: 0x%08x %s",
                   axis.name, errorCode, ErrorInfo(errorCode).c_str());
      return false;
    }

    if (VCS_SetPositionProfile(
          key_handle_, axis.node_id, axis.profile_velocity, axis.profile_acceleration,
          axis.profile_deceleration, &errorCode) == 0)
    {
      RCLCPP_ERROR(get_logger(), "%s SetPositionProfile failed: 0x%08x %s",
                   axis.name, errorCode, ErrorInfo(errorCode).c_str());
      return false;
    }

    if (axis.define_zero_on_start)
    {
      if (VCS_DefinePosition(key_handle_, axis.node_id, 0, &errorCode) == 0)
      {
        RCLCPP_ERROR(get_logger(), "%s DefinePosition failed: 0x%08x %s",
                     axis.name, errorCode, ErrorInfo(errorCode).c_str());
        return false;
      }
      RCLCPP_INFO(get_logger(), "%s position defined as zero", axis.name);
    }

    return true;
  }

  long TargetCounts(const AxisConfig & axis, double target_units) const
  {
    const double counts_per_rev = static_cast<double>(axis.encoder_cpr) * axis.gear_ratio;
    double target_counts = target_units * counts_per_rev / axis.units_per_rev;
    if (axis.invert_direction)
    {
      target_counts = -target_counts;
    }
    return static_cast<long>(std::llround(target_counts));
  }

  double CountsToUnits(const AxisConfig & axis, int counts) const
  {
    const double counts_per_rev = static_cast<double>(axis.encoder_cpr) * axis.gear_ratio;
    if (counts_per_rev == 0.0)
    {
      return 0.0;
    }
    double units = static_cast<double>(counts) * axis.units_per_rev / counts_per_rev;
    if (axis.invert_direction)
    {
      units = -units;
    }
    return units;
  }

  int LinearSpeedMmPerSecToProfileVelocity(double speed_mm_per_sec) const
  {
    if (linear_.units_per_rev <= 0.0 || linear_.gear_ratio <= 0.0)
    {
      return 0;
    }

    const double rpm = speed_mm_per_sec * 60.0 * linear_.gear_ratio / linear_.units_per_rev;
    return static_cast<int>(std::llround(rpm));
  }

  int ThetaSpeedTipRpmToProfileVelocity(double tip_rpm) const
  {
    if (theta_.gear_ratio <= 0.0)
    {
      return 0;
    }

    const double motor_rpm = tip_rpm * theta_.gear_ratio;
    return static_cast<int>(std::llround(motor_rpm));
  }

  void OnTargetDeg(const std_msgs::msg::Float64::SharedPtr msg)
  {
    int velocity = theta_.profile_velocity;
    int acceleration = theta_.profile_acceleration;
    int deceleration = theta_.profile_deceleration;
    // Fast mode is intentionally applied to linear axis only.
    if (!ApplyAxisProfile(theta_, velocity, acceleration, deceleration))
    {
      return;
    }

    long target = TargetCounts(theta_, msg->data);
    unsigned int errorCode = 0;
    int absolute = theta_.absolute ? 1 : 0;
    if (VCS_MoveToPosition(key_handle_, theta_.node_id, target, absolute, 1, &errorCode) == 0)
    {
      RCLCPP_ERROR(get_logger(), "theta MoveToPosition failed: 0x%08x %s",
                   errorCode, ErrorInfo(errorCode).c_str());
      return;
    }
  }

  void OnTargetLinear(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!enable_linear_)
    {
      return;
    }
    int velocity = linear_.profile_velocity;
    int acceleration = linear_.profile_acceleration;
    int deceleration = linear_.profile_deceleration;
    if (execute_fast_mode_)
    {
      velocity = linear_fast_profile_velocity_;
      acceleration = linear_fast_profile_acceleration_;
      deceleration = linear_fast_profile_deceleration_;
    }
    if (!ApplyAxisProfile(linear_, velocity, acceleration, deceleration))
    {
      return;
    }

    long target = TargetCounts(linear_, msg->data);
    unsigned int errorCode = 0;
    int absolute = linear_.absolute ? 1 : 0;
    if (VCS_MoveToPosition(key_handle_, linear_.node_id, target, absolute, 1, &errorCode) == 0)
    {
      RCLCPP_ERROR(get_logger(), "linear MoveToPosition failed: 0x%08x %s",
                   errorCode, ErrorInfo(errorCode).c_str());
      return;
    }
  }

  void PublishAxisState(
    const AxisConfig & axis,
    const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr & pub)
  {
    if (!pub)
    {
      return;
    }

    int positionIs = 0;
    unsigned int errorCode = 0;
    if (VCS_GetPositionIs(key_handle_, axis.node_id, &positionIs, &errorCode) == 0)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "%s GetPositionIs failed: 0x%08x %s",
                           axis.name, errorCode, ErrorInfo(errorCode).c_str());
      return;
    }

    std_msgs::msg::Float64 msg;
    msg.data = CountsToUnits(axis, positionIs);
    pub->publish(msg);
  }

  bool ApplyAxisProfile(const AxisConfig & axis, int velocity, int acceleration, int deceleration)
  {
    unsigned int errorCode = 0;
    if (VCS_SetPositionProfile(
          key_handle_, axis.node_id, static_cast<unsigned int>(velocity),
          static_cast<unsigned int>(acceleration), static_cast<unsigned int>(deceleration),
          &errorCode) == 0)
    {
      RCLCPP_ERROR(get_logger(), "%s SetPositionProfile failed: 0x%08x %s",
                   axis.name, errorCode, ErrorInfo(errorCode).c_str());
      return false;
    }
    return true;
  }

  void OnExecuteFastMode(const std_msgs::msg::Bool::SharedPtr msg)
  {
    execute_fast_mode_ = msg->data;
    RCLCPP_INFO(get_logger(), "Execute fast mode: %s", execute_fast_mode_ ? "ON" : "OFF");
  }

  void OnThetaVelocity(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (msg->data <= 0.0)
    {
      RCLCPP_WARN(get_logger(), "Ignoring non-positive theta velocity %.3f tip rpm", msg->data);
      return;
    }
    int velocity = ThetaSpeedTipRpmToProfileVelocity(msg->data);
    if (velocity <= 0)
    {
      RCLCPP_WARN(get_logger(), "Theta velocity %.3f tip rpm converted to invalid profile velocity %d rpm",
                  msg->data, velocity);
      return;
    }
    theta_.profile_velocity = velocity;
    RCLCPP_INFO(get_logger(), "Theta profile velocity set to %d rpm (from %.3f tip rpm)",
                theta_.profile_velocity, msg->data);
  }

  void OnLinearVelocity(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (msg->data <= 0.0)
    {
      RCLCPP_WARN(get_logger(), "Ignoring non-positive linear velocity %.3f mm/s", msg->data);
      return;
    }
    int velocity = LinearSpeedMmPerSecToProfileVelocity(msg->data);
    if (velocity <= 0)
    {
      RCLCPP_WARN(get_logger(), "Linear velocity %.3f mm/s converted to invalid profile velocity %d rpm",
                  msg->data, velocity);
      return;
    }
    linear_.profile_velocity = velocity;
    RCLCPP_INFO(get_logger(), "Linear profile velocity set to %d rpm (from %.3f mm/s)",
                linear_.profile_velocity, msg->data);
  }

  void PublishState()
  {
    if (!key_handle_)
    {
      return;
    }
    PublishAxisState(theta_, theta_state_pub_);
    if (enable_linear_)
    {
      PublishAxisState(linear_, linear_state_pub_);
    }
  }

  void LogStatus()
  {
    if (!key_handle_)
    {
      return;
    }

    LogAxisStatus(theta_);
    if (enable_linear_)
    {
      LogAxisStatus(linear_);
    }
  }

  void LogAxisStatus(const AxisConfig & axis)
  {
    unsigned int errorCode = 0;
    int isEnabled = 0;
    int isFault = 0;
    int positionIs = 0;
    int velocityIs = 0;
    int currentIs = 0;

    if (VCS_GetEnableState(key_handle_, axis.node_id, &isEnabled, &errorCode) == 0)
    {
      RCLCPP_WARN(get_logger(), "%s GetEnableState failed: 0x%08x %s",
                  axis.name, errorCode, ErrorInfo(errorCode).c_str());
      return;
    }
    if (VCS_GetFaultState(key_handle_, axis.node_id, &isFault, &errorCode) == 0)
    {
      RCLCPP_WARN(get_logger(), "%s GetFaultState failed: 0x%08x %s",
                  axis.name, errorCode, ErrorInfo(errorCode).c_str());
      return;
    }
    if (VCS_GetPositionIs(key_handle_, axis.node_id, &positionIs, &errorCode) == 0)
    {
      RCLCPP_WARN(get_logger(), "%s GetPositionIs failed: 0x%08x %s",
                  axis.name, errorCode, ErrorInfo(errorCode).c_str());
      return;
    }
    if (VCS_GetVelocityIs(key_handle_, axis.node_id, &velocityIs, &errorCode) == 0)
    {
      RCLCPP_WARN(get_logger(), "%s GetVelocityIs failed: 0x%08x %s",
                  axis.name, errorCode, ErrorInfo(errorCode).c_str());
      return;
    }
    if (VCS_GetCurrentIsEx(key_handle_, axis.node_id, &currentIs, &errorCode) == 0)
    {
      RCLCPP_WARN(get_logger(), "%s GetCurrentIsEx failed: 0x%08x %s",
                  axis.name, errorCode, ErrorInfo(errorCode).c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), "%s status enabled=%d fault=%d pos=%d vel=%d current=%d",
                axis.name, isEnabled, isFault, positionIs, velocityIs, currentIs);
  }

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr execute_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr theta_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr linear_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr theta_velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr linear_velocity_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr theta_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr linear_state_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  void * key_handle_ = nullptr;
  std::string device_name_;
  std::string protocol_stack_name_;
  std::string interface_name_;
  std::string port_name_;
  int baudrate_ = 0;
  bool enable_linear_ = false;
  bool execute_fast_mode_ = false;
  int fast_profile_velocity_ = 0;
  int fast_profile_acceleration_ = 0;
  int fast_profile_deceleration_ = 0;
  int linear_fast_profile_velocity_ = 0;
  int linear_fast_profile_acceleration_ = 0;
  int linear_fast_profile_deceleration_ = 0;
  AxisConfig theta_;
  AxisConfig linear_;
  bool ready_ = false;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EposMotionBridgeNode>();
  if (!node->IsReady())
  {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
