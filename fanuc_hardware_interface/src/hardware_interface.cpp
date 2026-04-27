// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include "fanuc_robot_driver/hardware_interface.hpp"

#include <atomic>
#include <charconv>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "fanuc_client/gpio_buffer.hpp"
#include "fanuc_robot_driver/constants.hpp"
#include "gpio_config/gpio_config.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

namespace fanuc_robot_driver
{
namespace
{
using CommandGPIOTypes = ::fanuc_client::GPIOBuffer::CommandGPIOTypes;
using StatusGPIOTypes = ::fanuc_client::GPIOBuffer::StatusGPIOTypes;

constexpr auto kFRHWInterface = "FR_HW_Interface";
constexpr int kNumberConnectionAttempts = 5;

enum class LogLevel
{
  kDebug,
  kWarn,
  kError,
};

struct FailureCounters
{
  std::atomic<uint64_t> timeout{ 0 };
  std::atomic<uint64_t> transport{ 0 };
  std::atomic<uint64_t> gpio{ 0 };
  std::atomic<uint64_t> rmi{ 0 };
  std::atomic<uint64_t> invalid_argument{ 0 };
};

FailureCounters g_failure_counters;

const char* StatusToString(fanuc_client::OperationStatus status)
{
  switch (status)
  {
    case fanuc_client::OperationStatus::kOk:
      return "ok";
    case fanuc_client::OperationStatus::kAlreadyStreaming:
      return "already_streaming";
    case fanuc_client::OperationStatus::kNotStreaming:
      return "not_streaming";
    case fanuc_client::OperationStatus::kInvalidArgument:
      return "invalid_argument";
    case fanuc_client::OperationStatus::kRmiError:
      return "rmi_error";
    case fanuc_client::OperationStatus::kTimeout:
      return "timeout";
    case fanuc_client::OperationStatus::kGpioError:
      return "gpio_error";
    case fanuc_client::OperationStatus::kTransportError:
      return "transport_error";
    case fanuc_client::OperationStatus::kUnknownError:
      return "unknown_error";
  }
  return "unknown_status";
}

const char* StatusActionHint(fanuc_client::OperationStatus status)
{
  switch (status)
  {
    case fanuc_client::OperationStatus::kOk:
      return "none";
    case fanuc_client::OperationStatus::kAlreadyStreaming:
      return "skip duplicate start request";
    case fanuc_client::OperationStatus::kNotStreaming:
      return "start stream before read/write";
    case fanuc_client::OperationStatus::kInvalidArgument:
      return "verify configuration and command dimensions";
    case fanuc_client::OperationStatus::kRmiError:
      return "check TP state, alarms, and RMI setup";
    case fanuc_client::OperationStatus::kTimeout:
      return "check controller run state and network latency";
    case fanuc_client::OperationStatus::kGpioError:
      return "validate GPIO config against controller setup";
    case fanuc_client::OperationStatus::kTransportError:
      return "check robot connectivity and transport channels";
    case fanuc_client::OperationStatus::kUnknownError:
      return "inspect detailed logs and robot diagnostics";
  }
  return "inspect detailed logs and robot diagnostics";
}

void IncrementFailureCounter(fanuc_client::OperationStatus status)
{
  // Metrics hook point for future telemetry export.
  switch (status)
  {
    case fanuc_client::OperationStatus::kTimeout:
      g_failure_counters.timeout.fetch_add(1, std::memory_order_relaxed);
      return;
    case fanuc_client::OperationStatus::kTransportError:
      g_failure_counters.transport.fetch_add(1, std::memory_order_relaxed);
      return;
    case fanuc_client::OperationStatus::kGpioError:
      g_failure_counters.gpio.fetch_add(1, std::memory_order_relaxed);
      return;
    case fanuc_client::OperationStatus::kRmiError:
      g_failure_counters.rmi.fetch_add(1, std::memory_order_relaxed);
      return;
    case fanuc_client::OperationStatus::kInvalidArgument:
      g_failure_counters.invalid_argument.fetch_add(1, std::memory_order_relaxed);
      return;
    default:
      return;
  }
}

void LogOperationStatus(LogLevel level, const char* operation, fanuc_client::OperationStatus status,
                        const std::string& detail)
{
  IncrementFailureCounter(status);
  const auto* status_text = StatusToString(status);
  const auto* action = StatusActionHint(status);

  switch (level)
  {
    case LogLevel::kDebug:
      RCLCPP_DEBUG(rclcpp::get_logger(kFRHWInterface),
                   "hw_interface | op=%s | status=%s | detail=%s | action_hint=%s", operation, status_text,
                   detail.c_str(), action);
      return;
    case LogLevel::kWarn:
      RCLCPP_WARN(rclcpp::get_logger(kFRHWInterface),
                  "hw_interface | op=%s | status=%s | detail=%s | action_hint=%s", operation, status_text,
                  detail.c_str(), action);
      return;
    case LogLevel::kError:
      RCLCPP_ERROR(rclcpp::get_logger(kFRHWInterface),
                   "hw_interface | op=%s | status=%s | detail=%s | action_hint=%s", operation, status_text,
                   detail.c_str(), action);
      return;
  }
}

bool StringToInt(const std::string& param_name, const std::string& param_value, int& out_value)
{
  const char* begin = param_value.data();
  const char* end = begin + param_value.size();
  auto result = std::from_chars(begin, end, out_value);

  if (result.ec == std::errc::invalid_argument || result.ptr != end)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRHWInterface), "Invalid integer parameter `%s` for parameter named `%s`",
                 param_value.c_str(), param_name.c_str());
    return false;
  }

  if (result.ec == std::errc::result_out_of_range)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRHWInterface), "Integer parameter out of range: `%s` for parameter named `%s`",
                 param_value.c_str(), param_name.c_str());
    return false;
  }

  return true;
}
}  // namespace

struct IOCommandInterface
{
  // Get the state interface name;
  virtual std::string name() const = 0;
  // Update internals based on value.
  virtual void updateBuffer() = 0;
  // The GPIO index.
  uint32_t index;
  // The value as a double (set by ROS 2 control).
  double value = 0;
};

template <CommandGPIOTypes type, typename T>
struct IOCommand : IOCommandInterface
{
  IOCommand(int i, fanuc_client::GPIOBuffer::CommandBlock<type, T>& b) : block(b)
  {
    index = i;
  }

  std::string name() const override
  {
    return std::string(block.type());
  }

  void updateBuffer() override
  {
    block.set(index, static_cast<T>(value));
  }

  // The block in the GPIO command buffer that this index belongs to.
  fanuc_client::GPIOBuffer::CommandBlock<type, T>& block;
};

struct IOStateInterface
{
  virtual std::string name() const = 0;
  // Update value based on internal sources.
  virtual void updateValue() = 0;
  // The GPIO index.
  uint32_t index;
  // The value as a double (read by ROS 2 control).
  double value = 0;
};

template <StatusGPIOTypes type, typename T>
struct IOState : IOStateInterface
{
  IOState(int i, fanuc_client::GPIOBuffer::StatusBlock<type, T>& b) : block(b)
  {
    index = i;
  }

  std::string name() const override
  {
    return std::string(block.type());
  }

  void updateValue() override
  {
    value = static_cast<double>(block.get(index));
  }

  // The block in the GPIO status buffer that this index belongs to.
  fanuc_client::GPIOBuffer::StatusBlock<type, T>& block;
};

struct IOInterfaces
{
  std::vector<std::unique_ptr<IOCommandInterface>> commands;
  std::vector<std::unique_ptr<IOStateInterface>> states;
  fanuc_client::GPIOBuffer buffer;
};

namespace
{

template <CommandGPIOTypes type>
void AppendBoolCmdInterfaces(uint32_t start, uint32_t length, fanuc_client::GPIOBuffer::Builder& builder,
                             std::vector<std::unique_ptr<IOCommandInterface>>& interfaces)
{
  auto& block = builder.addCommandConfig<type, bool>(start, length);
  for (uint32_t index = start; index < start + length; ++index)
  {
    interfaces.emplace_back(std::make_unique<IOCommand<type, bool>>(index, block));
  }
}

template <StatusGPIOTypes type>
void AppendBoolStateInterfaces(uint32_t start, uint32_t length, fanuc_client::GPIOBuffer::Builder& builder,
                               std::vector<std::unique_ptr<IOStateInterface>>& interfaces)
{
  auto& block = builder.addStatusConfig<type, bool>(start, length);
  for (uint32_t index = start; index < start + length; ++index)
  {
    interfaces.emplace_back(std::make_unique<IOState<type, bool>>(index, block));
  }
}

template <CommandGPIOTypes type>
void AppendAnalogCmdInterfaces(uint32_t start, uint32_t length, fanuc_client::GPIOBuffer::Builder& builder,
                               std::vector<std::unique_ptr<IOCommandInterface>>& interfaces)
{
  auto& block = builder.addCommandConfig<type, uint16_t>(start, length);
  for (uint32_t index = start; index < start + length; ++index)
  {
    interfaces.emplace_back(std::make_unique<IOCommand<type, uint16_t>>(index, block));
  }
}

template <StatusGPIOTypes type>
void AppendAnalogStateInterfaces(uint32_t start, uint32_t length, fanuc_client::GPIOBuffer::Builder& builder,
                                 std::vector<std::unique_ptr<IOStateInterface>>& interfaces)
{
  auto& block = builder.addStatusConfig<type, uint16_t>(start, length);
  for (uint32_t index = start; index < start + length; ++index)
  {
    interfaces.emplace_back(std::make_unique<IOState<type, uint16_t>>(index, block));
  }
}

IOInterfaces GPIOConfigToInterfaces(const gpio_config::GPIOTopicConfig& config)
{
  std::vector<std::unique_ptr<IOCommandInterface>> commands;
  std::vector<std::unique_ptr<IOStateInterface>> states;
  fanuc_client::GPIOBuffer::Builder buf_builder{};

  using ::gpio_config::BoolIOCmdType;
  if (config.io_cmd.has_value())
  {
    for (const gpio_config::BoolIOCmdConfig& bool_cmd : *config.io_cmd)
    {
      switch (bool_cmd.type)
      {
        case BoolIOCmdType::DO:
          AppendBoolCmdInterfaces<CommandGPIOTypes::DO>(bool_cmd.start, bool_cmd.length, buf_builder, commands);
          break;
        case BoolIOCmdType::RO:
          AppendBoolCmdInterfaces<CommandGPIOTypes::RO>(bool_cmd.start, bool_cmd.length, buf_builder, commands);
          break;
        case BoolIOCmdType::F:
          AppendBoolCmdInterfaces<CommandGPIOTypes::F>(bool_cmd.start, bool_cmd.length, buf_builder, commands);
          break;
      }
    }
  }

  using ::gpio_config::BoolIOStateType;
  if (config.io_state.has_value())
  {
    for (const gpio_config::BoolIOStateConfig& bool_state : *config.io_state)
    {
      switch (bool_state.type)
      {
        case BoolIOStateType::DO:
          AppendBoolStateInterfaces<StatusGPIOTypes::DO>(bool_state.start, bool_state.length, buf_builder, states);
          break;
        case BoolIOStateType::DI:
          AppendBoolStateInterfaces<StatusGPIOTypes::DI>(bool_state.start, bool_state.length, buf_builder, states);
          break;
        case BoolIOStateType::RO:
          AppendBoolStateInterfaces<StatusGPIOTypes::RO>(bool_state.start, bool_state.length, buf_builder, states);
          break;
        case BoolIOStateType::RI:
          AppendBoolStateInterfaces<StatusGPIOTypes::RI>(bool_state.start, bool_state.length, buf_builder, states);
          break;
        case BoolIOStateType::F:
          AppendBoolStateInterfaces<StatusGPIOTypes::F>(bool_state.start, bool_state.length, buf_builder, states);
          break;
      }
    }
  }

  using ::gpio_config::AnalogIOCmdType;
  if (config.analog_io_cmd.has_value())
  {
    for (const gpio_config::AnalogIOCmdConfig& analog_cmd : *config.analog_io_cmd)
    {
      if (analog_cmd.type == AnalogIOCmdType::AO)
      {
        AppendAnalogCmdInterfaces<CommandGPIOTypes::AO>(analog_cmd.start, analog_cmd.length, buf_builder, commands);
      }
    }
  }

  using ::gpio_config::AnalogIOStateType;
  if (config.analog_io_state.has_value())
  {
    for (const gpio_config::AnalogIOStateConfig& analog_state : *config.analog_io_state)
    {
      switch (analog_state.type)
      {
        case AnalogIOStateType::AO:
          AppendAnalogStateInterfaces<StatusGPIOTypes::AO>(analog_state.start, analog_state.length, buf_builder, states);
          break;
        case AnalogIOStateType::AI:
          AppendAnalogStateInterfaces<StatusGPIOTypes::AI>(analog_state.start, analog_state.length, buf_builder, states);
          break;
      }
    }
  }

  if (config.num_reg_cmd.has_value())
  {
    for (const gpio_config::NumRegConfig& num_reg_cmd : *config.num_reg_cmd)
    {
      const uint32_t start = num_reg_cmd.start;
      const uint32_t length = num_reg_cmd.length;
      auto& block = buf_builder.addCommandConfig<CommandGPIOTypes::FloatReg, float>(start, length);
      for (uint32_t index = start; index < start + length; ++index)
      {
        commands.emplace_back(std::make_unique<IOCommand<CommandGPIOTypes::FloatReg, float>>(index, block));
      }
    }
  }

  if (config.num_reg_state.has_value())
  {
    for (const gpio_config::NumRegConfig& num_reg_state : *config.num_reg_state)
    {
      const uint32_t start = num_reg_state.start;
      const uint32_t length = num_reg_state.length;
      auto& block = buf_builder.addStatusConfig<StatusGPIOTypes::FloatReg, float>(start, length);
      for (uint32_t index = start; index < start + length; ++index)
      {
        states.emplace_back(std::make_unique<IOState<StatusGPIOTypes::FloatReg, float>>(index, block));
      }
    }
  }

  return { std::move(commands), std::move(states), buf_builder.build() };
}

}  // namespace

FanucHardwareInterface::FanucHardwareInterface()
  : fr_joint_pos_{ Eigen::VectorXd::Zero(9) }
  , fr_prev_joint_pos_{ Eigen::VectorXd::Zero(9) }
  , fr_joint_vel_{ Eigen::VectorXd::Zero(9) }
  , joint_targets_{ Eigen::VectorXd::Zero(9) }
  , joint_targets_degrees_{ Eigen::VectorXd::Zero(9) }
  , stream_motion_port_(60015)
  , rmi_port_(1600)
{
}

FanucHardwareInterface::~FanucHardwareInterface() = default;

hardware_interface::CallbackReturn FanucHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  info_ = info;

  // Parse and configure cyclic GPIO from the yaml config.
  const auto config_path_it = info_.hardware_parameters.find("gpio_configuration");
  if (config_path_it != info_.hardware_parameters.end() && !config_path_it->second.empty())
  {
    gpio_config::GPIOConfig gpio_config;
    RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface),
                       "Loading GPIO configuration file: " << config_path_it->second);
    try
    {
      gpio_config = gpio_config::ParseGPIOConfig(std::filesystem::path(config_path_it->second));
    }
    catch (std::runtime_error& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger(kFRHWInterface), "Failed to parse gpio_config: %s", e.what());
      return CallbackReturn::ERROR;
    }

    IOInterfaces interfaces = GPIOConfigToInterfaces(gpio_config.gpio_topic_config);
    io_commands_ = std::move(interfaces.commands);
    io_state_ = std::move(interfaces.states);
    gpio_buffer_ = std::make_shared<fanuc_client::GPIOBuffer>(std::move(interfaces.buffer));
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
FanucHardwareInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "Preparing FANUC ROS2 HW interface");
  ip_address_ = info_.hardware_parameters["robot_ip"];
  int parsed_rmi_port = 0;
  int parsed_stream_motion_port = 0;
  int parsed_payload_schedule = 0;
  int parsed_out_cmd_interp_buff_target = 0;
  int parsed_force_sensor_type = 0;

  if (!StringToInt("rmi_port", info_.hardware_parameters["rmi_port"], parsed_rmi_port) ||
      !StringToInt("stream_motion_port", info_.hardware_parameters["stream_motion_port"], parsed_stream_motion_port) ||
      !StringToInt("payload_schedule", info_.hardware_parameters["payload_schedule"], parsed_payload_schedule) ||
      !StringToInt("out_cmd_interp_buff_target", info_.hardware_parameters["out_cmd_interp_buff_target"],
                   parsed_out_cmd_interp_buff_target) ||
      !StringToInt("force_sensor_type", info_.hardware_parameters["force_sensor_type"], parsed_force_sensor_type))
  {
    return CallbackReturn::ERROR;
  }

  rmi_port_ = static_cast<uint16_t>(parsed_rmi_port);
  stream_motion_port_ = static_cast<uint16_t>(parsed_stream_motion_port);
  payload_schedule_ = parsed_payload_schedule;
  out_cmd_interp_buff_target_ = static_cast<uint32_t>(parsed_out_cmd_interp_buff_target);
  force_sensor_type_ = static_cast<uint32_t>(parsed_force_sensor_type);

  RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "payload_schedule: " << payload_schedule_);
  RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "Starting RMI with: " << ip_address_);

  // Initialize the driver client
  for (int i = 0; i < kNumberConnectionAttempts; i++)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "Connecting to the robot: attempt: " << i);
    fanuc_client_.reset();
    std::string create_error;
    fanuc_client_ = fanuc_client::FanucClient::tryCreate(ip_address_, stream_motion_port_, rmi_port_, create_error);
    if (fanuc_client_ == nullptr)
    {
      RCLCPP_WARN(rclcpp::get_logger(kFRHWInterface), "%s", create_error.c_str());
      rclcpp::sleep_for(std::chrono::milliseconds(3000));
      continue;
    }

    fanuc_client_->setOutCmdInterpBuffTarget(out_cmd_interp_buff_target_);
    fanuc_client_->setForceSensorType(force_sensor_type_);

    const auto start_rmi_status = fanuc_client_->tryStartRMI();
    if (start_rmi_status != fanuc_client::OperationStatus::kOk)
    {
      LogOperationStatus(LogLevel::kWarn, "on_configure.tryStartRMI", start_rmi_status, fanuc_client_->lastError());
      rclcpp::sleep_for(std::chrono::milliseconds(3000));
      continue;
    }
    const auto set_payload_status = fanuc_client_->trySetPayloadSchedule(payload_schedule_);
    if (set_payload_status != fanuc_client::OperationStatus::kOk)
    {
      LogOperationStatus(LogLevel::kWarn, "on_configure.trySetPayloadSchedule", set_payload_status,
                         fanuc_client_->lastError());
      rclcpp::sleep_for(std::chrono::milliseconds(3000));
      continue;
    }
    const auto validate_gpio_status = fanuc_client_->tryValidateGPIOBuffer(gpio_buffer_);
    if (validate_gpio_status != fanuc_client::OperationStatus::kOk)
    {
      LogOperationStatus(LogLevel::kWarn, "on_configure.tryValidateGPIOBuffer", validate_gpio_status,
                         fanuc_client_->lastError());
      rclcpp::sleep_for(std::chrono::milliseconds(3000));
      continue;
    }

      RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "Successfully connected to the robot.");
      RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface),
                         "FANUC ROS2 HW interface is ready with client version: " << fanuc_client_->getClientVersion());
      return CallbackReturn::SUCCESS;
  }

  RCLCPP_ERROR(rclcpp::get_logger(kFRHWInterface), "Failed to connect to the robot.");
  return CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn FanucHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "activating hardware interface");

  if (fanuc_client_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRHWInterface), "Cannot activate: FANUC client is not initialized.");
    return CallbackReturn::ERROR;
  }

  const auto start_stream_status = fanuc_client_->tryStartRealtimeStream(gpio_buffer_);
  if (start_stream_status != fanuc_client::OperationStatus::kOk)
  {
    robot_status_.is_connected = 0.0;
    LogOperationStatus(LogLevel::kError, "on_activate.tryStartRealtimeStream", start_stream_status,
                       fanuc_client_->lastError());
    return CallbackReturn::ERROR;
  }

  const auto read_initial_status = fanuc_client_->tryReadJointAngles();
  if (read_initial_status != fanuc_client::OperationStatus::kOk)
  {
    robot_status_.is_connected = 0.0;
    LogOperationStatus(LogLevel::kError, "on_activate.tryReadJointAngles", read_initial_status,
                       fanuc_client_->lastError());
    return CallbackReturn::ERROR;
  }

  joint_targets_degrees_ = fanuc_client_->latestJointAngles();
  joint_targets_.array() = M_PI / 180.0 * joint_targets_degrees_.array();
  robot_status_.is_connected = 1.0;
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FanucHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "deactivating stream motion");

  if (fanuc_client_ != nullptr)
  {
    const auto stop_status = fanuc_client_->tryStopRealtimeStream();
    if (stop_status != fanuc_client::OperationStatus::kOk)
    {
      LogOperationStatus(LogLevel::kWarn, "on_deactivate.tryStopRealtimeStream", stop_status,
                         fanuc_client_->lastError());
    }
  }

  robot_status_.is_connected = 0.0;
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FanucHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  robot_status_.is_connected = 0.0;
  fanuc_client_.reset();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FanucHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(2 * info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &fr_joint_pos_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &fr_joint_vel_[i]);
  }

  for (const auto& io_state : io_state_)
  {
    state_interfaces.emplace_back(io_state->name(), std::to_string(io_state->index), &io_state->value);
  }

  state_interfaces.emplace_back(kRobotStatusInterfaceName, kStatusInErrorType, &robot_status_.in_error);
  state_interfaces.emplace_back(kRobotStatusInterfaceName, kStatusTPEnabledType, &robot_status_.tp_enabled);
  state_interfaces.emplace_back(kRobotStatusInterfaceName, kStatusEStoppedType, &robot_status_.e_stopped);
  state_interfaces.emplace_back(kRobotStatusInterfaceName, kStatusMotionPossibleType, &robot_status_.motion_possible);
  state_interfaces.emplace_back(kRobotStatusInterfaceName, kStatusContactStopModeType, &robot_status_.contact_stop_mode);
  state_interfaces.emplace_back(kRobotStatusInterfaceName, kStatusCollaborativeSpeedScalingType,
                                &robot_status_.collaborative_speed_scaling);

  state_interfaces.emplace_back(kConnectionStatusName, kIsConnectedType, &robot_status_.is_connected);

  state_interfaces.emplace_back(kForceInterfaceName, kForceXType, &force_sensor_.force_x);
  state_interfaces.emplace_back(kForceInterfaceName, kForceYType, &force_sensor_.force_y);
  state_interfaces.emplace_back(kForceInterfaceName, kForceZType, &force_sensor_.force_z);
  state_interfaces.emplace_back(kForceInterfaceName, kMomentXType, &force_sensor_.moment_x);
  state_interfaces.emplace_back(kForceInterfaceName, kMomentYType, &force_sensor_.moment_y);
  state_interfaces.emplace_back(kForceInterfaceName, kMomentZType, &force_sensor_.moment_z);
  state_interfaces.emplace_back(kForceInterfaceName, kForceSensorType, &force_sensor_.fs_type);

  // For force_torque_sensor_broadcaster (geometry_msgs/WrenchStamped)
  state_interfaces.emplace_back("ft_sensor", "force.x", &force_sensor_.force_x);
  state_interfaces.emplace_back("ft_sensor", "force.y", &force_sensor_.force_y);
  state_interfaces.emplace_back("ft_sensor", "force.z", &force_sensor_.force_z);
  state_interfaces.emplace_back("ft_sensor", "torque.x", &force_sensor_.moment_x);
  state_interfaces.emplace_back("ft_sensor", "torque.y", &force_sensor_.moment_y);
  state_interfaces.emplace_back("ft_sensor", "torque.z", &force_sensor_.moment_z);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FanucHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(info_.joints.size()); ++i)
  {
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_targets_[i]);
  }

  for (const auto& io_command : io_commands_)
  {
    command_interfaces.emplace_back(io_command->name(), std::to_string(io_command->index), &io_command->value);
  }

  return command_interfaces;
}

hardware_interface::return_type FanucHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                             const rclcpp::Duration& period)
{
  robot_status_.is_connected = fanuc_client_ != nullptr && fanuc_client_->isStreaming();
  if (!robot_status_.is_connected)
  {
    if (fanuc_client_ != nullptr)
    {
      fanuc_client_->tryStopRealtimeStream();
    }

    static auto last_log_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count() > 5000)
    {
      RCLCPP_WARN(rclcpp::get_logger(kFRHWInterface),
                  "FANUC ROS2 HW no longer streaming (this is normal during shutdown).");
      last_log_time = now;
    }
    return hardware_interface::return_type::ERROR;
  }

  const auto read_status = fanuc_client_->tryReadJointAngles();
  if (read_status != fanuc_client::OperationStatus::kOk)
  {
    LogOperationStatus(LogLevel::kDebug, "read.tryReadJointAngles", read_status, fanuc_client_->lastError());
    return hardware_interface::return_type::ERROR;
  }

  fr_prev_joint_pos_ = fr_joint_pos_;
  const Eigen::Ref<const Eigen::VectorXd> joint_angles = fanuc_client_->latestJointAngles();
  for (Eigen::Index i = 0; i < joint_angles.size(); ++i)
  {
    fr_joint_pos_[i] = M_PI / 180.0 * joint_angles[i];
  }
  if ((fr_prev_joint_pos_.array() != fr_joint_pos_.array()).any())
  {
    const double dt = static_cast<double>(fanuc_client_->getControlPeriod()) / 1000.0;
    fr_joint_vel_ = (fr_joint_pos_ - fr_prev_joint_pos_) / dt;
  }

  for (const auto& io_state : io_state_)
  {
    io_state->updateValue();
  }

  robot_status_.in_error = fanuc_client_->robot_status().in_error;
  robot_status_.tp_enabled = fanuc_client_->robot_status().tp_enabled;
  robot_status_.e_stopped = fanuc_client_->robot_status().e_stopped;
  robot_status_.motion_possible = fanuc_client_->robot_status().motion_possible;
  robot_status_.contact_stop_mode = static_cast<double>(fanuc_client_->robot_status().contact_stop_mode);
  robot_status_.collaborative_speed_scaling = static_cast<double>(fanuc_client_->robot_status().safety_scale);

  force_sensor_.force_x = static_cast<double>(fanuc_client_->force_sensor().force_x);
  force_sensor_.force_y = static_cast<double>(fanuc_client_->force_sensor().force_y);
  force_sensor_.force_z = static_cast<double>(fanuc_client_->force_sensor().force_z);
  force_sensor_.moment_x = static_cast<double>(fanuc_client_->force_sensor().moment_x);
  force_sensor_.moment_y = static_cast<double>(fanuc_client_->force_sensor().moment_y);
  force_sensor_.moment_z = static_cast<double>(fanuc_client_->force_sensor().moment_z);
  force_sensor_.fs_type = static_cast<double>(fanuc_client_->force_sensor().fs_type);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FanucHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  robot_status_.is_connected = fanuc_client_ != nullptr && fanuc_client_->isStreaming();
  if (!robot_status_.is_connected)
  {
    if (fanuc_client_ != nullptr)
    {
      fanuc_client_->tryStopRealtimeStream();
    }
    // Throttle to avoid spam during shutdown
    static auto last_log_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count() > 5000)
    {
      RCLCPP_WARN(rclcpp::get_logger(kFRHWInterface),
                  "FANUC ROS2 HW no longer streaming (this is normal during shutdown).");
      last_log_time = now;
    }
    return hardware_interface::return_type::ERROR;
  }

  joint_targets_degrees_.array() = 180.0 / M_PI * joint_targets_.array();

  const auto write_status = fanuc_client_->tryWriteJointTarget(joint_targets_degrees_);
  if (write_status != fanuc_client::OperationStatus::kOk)
  {
    LogOperationStatus(LogLevel::kDebug, "write.tryWriteJointTarget", write_status, fanuc_client_->lastError());
    return hardware_interface::return_type::ERROR;
  }

  for (const auto& io_command : io_commands_)
  {
    io_command->updateBuffer();
  }
  fanuc_client_->sendIOCommand();

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn FanucHardwareInterface::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  if (fanuc_client_ != nullptr)
  {
    const auto stop_status = fanuc_client_->tryStopRealtimeStream();
    if (stop_status != fanuc_client::OperationStatus::kOk)
    {
      LogOperationStatus(LogLevel::kDebug, "on_shutdown.tryStopRealtimeStream", stop_status,
                         fanuc_client_->lastError());
    }
  }
  robot_status_.is_connected = 0.0;
  fanuc_client_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}
}  // namespace fanuc_robot_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(fanuc_robot_driver::FanucHardwareInterface, hardware_interface::SystemInterface)
