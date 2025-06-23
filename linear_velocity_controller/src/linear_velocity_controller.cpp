#include "franka_semantic_components/franka_cartesian_velocity_interface.hpp"
#include "linear_velocity_controller/generate_trajectory.hpp"
#include <linear_velocity_controller/linear_velocity_controller.hpp>

#include <cstdio>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <controller_interface/controller_interface_base.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include <ruckig/ruckig.hpp>

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
    auto_declare<double>("x_vel", IGNORE_DIRECTION);
    auto_declare<double>("x_dist", IGNORE_DIRECTION);
    auto_declare<double>("y_vel", IGNORE_DIRECTION);
    auto_declare<double>("y_dist", IGNORE_DIRECTION);
    auto_declare<double>("z_vel", IGNORE_DIRECTION);
    auto_declare<double>("z_dist", IGNORE_DIRECTION);

    // auto_declare<std::string>("cmd_topic", "~/cmd_pose");
    // auto_declare<std::string>("state_topic", "~/commanded_state");
  } catch (const std::exception &e) {
    (void)fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LinearVelocityController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/
) {
  // Get the desired velocity and distance in the x-direction
  x_vel_ = get_node()->get_parameter("x_vel").as_double();
  x_dist_ = get_node()->get_parameter("x_dist").as_double();
  if (x_vel_ == IGNORE_DIRECTION && x_dist_ == IGNORE_DIRECTION) {
    RCLCPP_INFO(get_node()->get_logger(), "Will not move in x-direction.");
  } else if (x_vel_ != IGNORE_DIRECTION && x_dist_ != IGNORE_DIRECTION) {
    RCLCPP_INFO(get_node()->get_logger(), "Will move in x-direction with %.3f m/s for up to %0.3f m.", x_vel_, x_vel_);
  } else {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameters 'x_vel' and 'x_dist' must both be nonzero, or both be %.1f. Got x_vel = %.3f and x_dist = %.3f",
      IGNORE_DIRECTION, x_vel_, x_dist_
    );
    return controller_interface::CallbackReturn::ERROR;
  }

  // Get the desired velocity and distance in the y-direction
  y_vel_ = get_node()->get_parameter("y_vel").as_double();
  y_dist_ = get_node()->get_parameter("y_dist").as_double();
  if (y_vel_ == IGNORE_DIRECTION && y_dist_ == IGNORE_DIRECTION) {
    RCLCPP_INFO(get_node()->get_logger(), "Will not move in y-direction.");
  } else if (y_vel_ != IGNORE_DIRECTION && y_dist_ != IGNORE_DIRECTION) {
    RCLCPP_INFO(get_node()->get_logger(), "Will move in y-direction with %.3f m/s for up to %0.3f m.", y_vel_, y_vel_);
  } else {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameters 'y_vel' and 'y_dist' must both be nonzero, or both be %.1f. Got y_vel = %.3f and y_dist = %.3f",
      IGNORE_DIRECTION, y_vel_, y_dist_
    );
    return controller_interface::CallbackReturn::ERROR;
  }

  // Get the desired velocity and distance in the z-direction
  z_vel_ = get_node()->get_parameter("z_vel").as_double();
  z_dist_ = get_node()->get_parameter("z_dist").as_double();
  if (z_vel_ == IGNORE_DIRECTION && z_dist_ == IGNORE_DIRECTION) {
    RCLCPP_INFO(get_node()->get_logger(), "Will not move in z-direction.");
  } else if (z_vel_ != IGNORE_DIRECTION && z_dist_ != IGNORE_DIRECTION) {
    RCLCPP_INFO(get_node()->get_logger(), "Will move in z-direction with %.3f m/s for up to %0.3f m.", z_vel_, z_vel_);
  } else {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameters 'z_vel' and 'z_dist' must both be nonzero, or both be %.1f. Got z_vel = %.3f and z_dist = %.3f",
      IGNORE_DIRECTION, z_vel_, z_dist_
    );
    return controller_interface::CallbackReturn::ERROR;
  }

  // // Command subscriber configuration
  // try {
  //   auto cmd_topic_param = get_node()->get_parameter("cmd_topic").as_string();
  //   // Register commanded state publisher
  //   subscriber_ = get_node()->create_subscription<CmdMsg>(
  //     cmd_topic_param, rclcpp::SystemDefaultsQoS(),
  //     [this](const CmdMsg::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); }
  //   );
  // } catch (const std::exception &e) {
  //   // NOLINTNEXTLINE(cert-err33-c)
  //   fprintf(
  //     stderr,
  //     "Exception thrown during subscription creation at configure stage "
  //     "with message : %s \n",
  //     e.what()
  //   );
  //   return controller_interface::CallbackReturn::ERROR;
  // }

  // // State publisher configuration
  // try {
  //   auto state_topic_param = get_node()->get_parameter("state_topic").as_string();
  //   // Register commanded state publisher
  //   publisher_ = get_node()->create_publisher<StateMsg>(state_topic_param, rclcpp::SystemDefaultsQoS());
  //   realtime_publisher_ = std::make_unique<RtStatePublisher>(publisher_);
  // } catch (const std::exception &e) {
  //   // NOLINTNEXTLINE(cert-err33-c)
  //   fprintf(
  //     stderr,
  //     "Exception thrown during publisher creation at configure stage "
  //     "with message : %s \n",
  //     e.what()
  //   );
  //   return controller_interface::CallbackReturn::ERROR;
  // }

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
  x_dist_ = 0.0;
  y_vel_ = 0.0;
  y_dist_ = 0.0;
  z_vel_ = 0.0;
  z_dist_ = 0.0;

  // Remove semantic component
  franka_cartesian_velocity_.reset();

  // // Remove subscriber
  // subscriber_.reset();
  // // Remove publisher
  // publisher_.reset();
  // realtime_publisher_.reset();

  RCLCPP_INFO(get_node()->get_logger(), "Cleanup successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LinearVelocityController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/
) {
  // Reset internal variables
  ramp_up_trajectory_ = trajectory::generate_acceleration_phase({x_vel_, y_vel_, z_vel_});

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
  ramp_up_trajectory_.clear();

  // Release command interfaces
  franka_cartesian_velocity_->release_interfaces();

  // // Reset command buffer
  // rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdMsg>>(nullptr);

  RCLCPP_INFO(get_node()->get_logger(), "Deactivate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LinearVelocityController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/
) {
  // // Read the commands from the real-time buffer
  // std::shared_ptr<CmdMsg> *twist_command_msg_ptr = rt_command_ptr_.readFromRT();

  // // Check if a command is received yet
  // if ((twist_command_msg_ptr == nullptr) || !(*twist_command_msg_ptr)) {
  //   // No command received yet, continue the controller without setting an
  //   // output
  //   return controller_interface::return_type::OK;
  // }

  // // Check if the received command has the correct number of values
  // auto twist_command_msg = (*twist_command_msg_ptr);

  // Also publish the commanded state to the real time publisher
  // if (realtime_publisher_ && realtime_publisher_->trylock()) {
  //   realtime_publisher_->msg_.header.stamp = time;
  //   realtime_publisher_->msg_.data = std::vector<double>(N_JOINTS, 0.0);
  //   for (size_t index = 0ul; index < N_JOINTS; ++index) {
  //     realtime_publisher_->msg_.data[index] = commanded_joint_torques(static_cast<int>(index));
  //   }
  //   realtime_publisher_->unlockAndPublish();
  // }

  // -----------------------------------------

  double x_vel_target;
  double y_vel_target;
  double z_vel_target;
  if (loop_count_ < ramp_up_trajectory_.size()) {
    // Still ramping up
    auto ramp_up_velocities = ramp_up_trajectory_[loop_count_];
    x_vel_target = ramp_up_velocities[0];
    y_vel_target = ramp_up_velocities[1];
    z_vel_target = ramp_up_velocities[2];
  } else {
    // In constant velocity
    x_vel_target = x_vel_;
    y_vel_target = y_vel_;
    z_vel_target = z_vel_;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Sending velocities %f, %f, %f", x_vel_target, y_vel_target, z_vel_target);

  // Send the output commands
  Eigen::Vector3d cartesian_linear_velocity(x_vel_target, y_vel_target, z_vel_target);
  Eigen::Vector3d cartesian_angular_velocity(0.0, 0.0, 0.0);

  loop_count_++;

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