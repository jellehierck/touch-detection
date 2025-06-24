#include "linear_velocity_controller/linear_velocity_controller.hpp"

#include <cstdio>
#include <exception>
#include <memory>
#include <string>

#include <Eigen/src/Core/Matrix.h>

#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <controller_interface/controller_interface_base.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include <franka_semantic_components/franka_cartesian_velocity_interface.hpp>

#include "linear_velocity_controller/linear_trajectory_generator.hpp"

namespace linear_velocity_controller {

controller_interface::InterfaceConfiguration LinearVelocityController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_velocity_->get_command_interface_names();
  return config;
}

controller_interface::InterfaceConfiguration LinearVelocityController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::CallbackReturn LinearVelocityController::on_init() {
  try {
    auto_declare<double>("x_vel", 0.0);
    auto_declare<double>("y_vel", 0.0);
    auto_declare<double>("z_vel", 0.0);
    auto_declare<double>("distance", 0.0);
    auto_declare<double>("duration", 0.0);
  } catch (const std::exception &e) {
    (void)fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LinearVelocityController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/
) {
  // Get the desired velocities
  x_vel_ = get_node()->get_parameter("x_vel").as_double();
  y_vel_ = get_node()->get_parameter("y_vel").as_double();
  z_vel_ = get_node()->get_parameter("z_vel").as_double();

  target_distance_ = get_node()->get_parameter("distance").as_double();
  target_duration_ = get_node()->get_parameter("duration").as_double();
  if (target_distance_ > 0.0 && target_duration_ > 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot set both parameters 'distance' and 'duration'.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (target_distance_ < 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'distance' cannot be negative (got %f).", target_distance_);
    return controller_interface::CallbackReturn::ERROR;
  }
  if (target_duration_ < 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'distance' cannot be negative (got %f).", target_duration_);
    return controller_interface::CallbackReturn::ERROR;
  }
  if (target_distance_ == 0.0 && target_duration_ == 0.0) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Parameters 'distance' and 'duration' are both zero, the resulting motion will ramp up and down immediately."
    );
  }

  // Set up the semantic component
  franka_cartesian_velocity_ = std::make_unique<franka_semantic_components::FrankaCartesianVelocityInterface>(
    franka_semantic_components::FrankaCartesianVelocityInterface(false  // No elbow commands
    )
  );

  // TODO: Setup default collision behaviour?

  RCLCPP_INFO(get_node()->get_logger(), "Configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LinearVelocityController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/
) {
  // Reset parameters
  x_vel_ = 0.0;
  y_vel_ = 0.0;
  z_vel_ = 0.0;
  target_distance_ = 0.0;
  target_duration_ = 0.0;

  // Remove semantic component
  franka_cartesian_velocity_.reset();

  RCLCPP_INFO(get_node()->get_logger(), "Cleanup successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LinearVelocityController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/
) {
  // Compute trajectory
  const LinearVelocityTrajectoryGenerator::DimVector target_velocity(x_vel_, y_vel_, z_vel_);
  if (target_distance_ > 0.0) {
    trajectory_generator_ = std::make_shared<LinearVelocityTrajectoryGenerator>(
      LinearVelocityTrajectoryGenerator::with_target_distance(target_velocity, target_distance_)
    );
  } else if (target_duration_ > 0.0) {
    trajectory_generator_ = std::make_shared<LinearVelocityTrajectoryGenerator>(
      LinearVelocityTrajectoryGenerator::with_target_duration(target_velocity, target_duration_)
    );
  } else {
    trajectory_generator_ =
      std::make_shared<LinearVelocityTrajectoryGenerator>(LinearVelocityTrajectoryGenerator(target_velocity));
  }
  const std::string trajectory_description = trajectory_generator_->describe();
  RCLCPP_INFO(get_node()->get_logger(), "%s", trajectory_description.c_str());
  elapsed_time_ = rclcpp::Duration(0, 0);

  // Set command interfaces
  franka_cartesian_velocity_->assign_loaned_command_interfaces(command_interfaces_);

  // // Reset command buffer in case a command came in when controller was inactive
  // rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdMsg>>(nullptr);

  RCLCPP_INFO(get_node()->get_logger(), "Activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LinearVelocityController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/
) {
  // Reset internal variables
  trajectory_generator_.reset();

  // Release command interfaces
  franka_cartesian_velocity_->release_interfaces();

  // // Reset command buffer
  // rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdMsg>>(nullptr);

  RCLCPP_INFO(get_node()->get_logger(), "Deactivate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LinearVelocityController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration &period
) {
  elapsed_time_ = elapsed_time_ + period;

  // Send the output commands
  const Eigen::Vector3d cartesian_linear_velocity = trajectory_generator_->velocity_at_time(elapsed_time_.seconds());
  const Eigen::Vector3d cartesian_angular_velocity(0.0, 0.0, 0.0);
  if (!franka_cartesian_velocity_->setCommand(cartesian_linear_velocity, cartesian_angular_velocity)) {
    RCLCPP_FATAL(get_node()->get_logger(), "Set command failed. Did you activate the elbow command interface?");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

}  // namespace linear_velocity_controller

// Export the controller as a pluginlib plugin

#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(linear_velocity_controller::LinearVelocityController, controller_interface::ControllerInterface)