#include <controller_interface/controller_interface.hpp>
#include "dz_minimal_controller/dz_minimal_controller.hpp"

#include <stddef.h>
#include <chrono>
#include <functional>
#include <memory>
#include <ostream>
#include <ratio>
#include <string>
#include <vector>

// we need to include what we use here, rclcpp etc :/ 

namespace dz_minimal_controller
{

DzMinimalController::DzMinimalController()
{
}

controller_interface::CallbackReturn DzMinimalController::on_init()
{
  try //see JTC which is also using generate_parameter_library
  {
    param_listener_ = std::make_shared<dz_minimal_controller::ParamListener>(this->get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what()); // cargo-culted, should use logging macro
    return controller_interface::CallbackReturn::ERROR;
  }
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration DzMinimalController::command_interface_configuration()
  const
{
  std::vector<std::string> command_interfaces_config_names;
  for (const auto & interface_name : params_.command_interfaces)
  {
    for (const auto & joint_name : params_.joints)
    {
      auto full_name = joint_name + "/" + interface_name; // we may want to extend later. 
      command_interfaces_config_names.push_back(full_name);
    }
  }
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    command_interfaces_config_names
  };
}

controller_interface::InterfaceConfiguration DzMinimalController::state_interface_configuration()
  const
{
  std::vector<std::string> state_interfaces_config_names;
  for (const auto & interface_name : params_.state_interfaces)
  {
    for (const auto & joint_name : params_.joints)
    {
      auto full_name = joint_name + "/" + interface_name;
      state_interfaces_config_names.push_back(full_name);
    }
  }
  // we may have a reason here to claim the F/T sensor state interface 
  // like admittance_controller does, 
  // but this is next steps
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    state_interfaces_config_names
  };
}

controller_interface::CallbackReturn DzMinimalController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), 
      "The parameter listener was never initialized, probably because init failed.");
    return controller_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(this->get_node()->get_logger(), "on_configure() returns SUCCESS");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DzMinimalController::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_node()->get_logger(), "on_activate() returns SUCCESS");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DzMinimalController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_node()->get_logger(), "on_deactivate() returns SUCCESS");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type DzMinimalController::update(
  const rclcpp::Time & /* time */, const rclcpp::Duration & period)
{
  auto clk = *this->get_node()->get_clock(); // probably reconsider for RT but also we won't be logging
  RCLCPP_INFO_THROTTLE(this->get_node()->get_logger(),
    clk,
    period.nanoseconds()/(50*1'000'000), // yet again https://github.com/ros2/rclcpp/issues/1929
    "update() returns OK at 1/50th the loop rate"
  );
  //https://github.com/ros2/ros2_documentation/pull/3143

  return controller_interface::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dz_minimal_controller::DzMinimalController, controller_interface::ControllerInterface)





}  // namespace dz_minimal_controller
