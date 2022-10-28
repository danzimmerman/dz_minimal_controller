/***************************************************************************************************
 * A minimal "new style" ROS2 Control controller using PickNik's generate_parameter_library
 * 
 * Author: Dan Zimmerman
 * Date: 2022-10-28
 * 
 **************************************************************************************************/

#ifndef DZ_MINIMAL_CONTROLLER__DZ_MINIMAL_CONTROLLER_HPP_
#define DZ_MINIMAL_CONTROLLER__DZ_MINIMAL_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "dz_minimal_controller/visibility_control.h"
#include <controller_interface/controller_interface.hpp>
#include <controller_interface/helpers.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>


/***************************************************************************************************
 * We're using https://github.com/PickNikRobotics/generate_parameter_library for parameters.
 * 
 * The header file below does not exist until build time.
 * 
 * The resulting Params, ParamListener, etc. go into the project's normal namespace 
 * 
 * For example:
 * namespace dz_minimal_controller {
 *  struct Params {...};
 * ...
 * }
 * generate_parameter_library does not actually nest the .hpp file in a folder like normal includes
 **************************************************************************************************/
#include "dz_minimal_controller_generated_parameter_library.hpp" // code-gen

namespace dz_minimal_controller
{

class DzMinimalController : public controller_interface::ControllerInterface
{
public:
  DZ_MINIMAL_CONTROLLER_PUBLIC
  DzMinimalController();

  DZ_MINIMAL_CONTROLLER_PUBLIC 
  virtual ~DzMinimalController() = default; 

  DZ_MINIMAL_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  DZ_MINIMAL_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  
  DZ_MINIMAL_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  DZ_MINIMAL_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  
  DZ_MINIMAL_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  DZ_MINIMAL_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  DZ_MINIMAL_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // we could eventually also add lifecycle methods on_cleanup() and on_error() here
  // also some controllers have on_export_reference_interfaces()

protected: 
  // Params and ParamListener are from generate_parameter_library so the code 
  // for these classes does not exist until build time 
  dz_minimal_controller::Params params_; 
  std::shared_ptr<dz_minimal_controller::ParamListener> param_listener_;

  trajectory_msgs::msg::JointTrajectoryPoint last_reference_;
};


}  // namespace dz_minimal_controller

#endif  // DZ_MINIMAL_CONTROLLER__DZ_MINIMAL_CONTROLLER_HPP_
