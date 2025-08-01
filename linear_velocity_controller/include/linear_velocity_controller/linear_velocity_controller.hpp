#ifndef LINEAR_VELOCITY_CONTROLLER_LINEAR_VELOCITY_CONTROLLER
#define LINEAR_VELOCITY_CONTROLLER_LINEAR_VELOCITY_CONTROLLER

#include <memory>

#include <Eigen/Eigen>
#include <Eigen/src/Core/IO.h>
#include <Eigen/src/Core/Matrix.h>

#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <controller_interface/controller_interface.hpp>
#include <controller_interface/controller_interface_base.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include <franka_semantic_components/franka_cartesian_velocity_interface.hpp>

#include "linear_velocity_controller/linear_trajectory_generator.hpp"

namespace linear_velocity_controller {

/**
 *
 */
class LinearVelocityController : public controller_interface::ControllerInterface {
 protected:
  /**
   * Specify which hardware command interfaces are used.
   *
   * This method is called in the `active` or `inactive` state. This means
   * that the configuration can be changed during the `on_configure` state
   * but not afterwards.
   */
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * Specify which hardware state interfaces are used.
   *
   * This method is called in the `active` or `inactive` state. This means
   * that the configuration can be changed during the `on_configure` state
   * but not afterwards.
   */
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  /**
   * Declare all parameters.
   */
  CallbackReturn on_init() override;

  /**
   * Perform tasks that must be performed once during the node life time.

   * The tasks to perform can be e.g. obtaining parameter values and
   * setting up topic publications/subscriptions that do not change.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * Clear all state and return the node to a functionally
   * equivalent state as when first created.
   *
   * This should revert all changes during the configure
   * transition, e.g. destroying all managed
   * objects transition.
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * Do any final preparations to start executing.
   *
   * This may include acquiring resources that are only held while the node is
   * actually active, such as access to hardware and setting up real-time
   * buffers.
   *
   * Ideally, no preparation that requires significant time (such as
   * lengthy hardware initialisation) should be performed in this callback.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * Do any cleanup after executing is stopped.
   *
   * This should reverse all changes during the activate transition.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * Perform a single controller update step.
   *
   * This function should ONLY execute real-time capable code.
   *
   * @param time Current time.
   * @param period Time since the last controller update step.
   */
  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

 private:
  std::unique_ptr<franka_semantic_components::FrankaCartesianVelocityInterface> franka_cartesian_velocity_;

  // Parameter values
  double x_vel_ = 0.0;
  double y_vel_ = 0.0;
  double z_vel_ = 0.0;
  double target_distance_ = 0.0;
  double target_duration_ = 0.0;

  // Internal variables
  std::shared_ptr<LinearVelocityTrajectoryGenerator> trajectory_generator_;
  rclcpp::Duration                                   elapsed_time_ = rclcpp::Duration(0, 0);
};
}  // namespace linear_velocity_controller

#endif  // LINEAR_VELOCITY_CONTROLLER_LINEAR_VELOCITY_CONTROLLER