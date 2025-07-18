#ifndef LINEAR_VELOCITY_CONTROLLER_GENERATE_TRAJECTORY
#define LINEAR_VELOCITY_CONTROLLER_GENERATE_TRAJECTORY

#include <cstddef>
#include <sstream>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <Eigen/src/Core/Matrix.h>

#include <ruckig/ruckig.hpp>
#include <ruckig/input_parameter.hpp>
#include <ruckig/output_parameter.hpp>
#include <ruckig/utils.hpp>
#include <ruckig/trajectory.hpp>

namespace linear_velocity_controller {

class LinearVelocityTrajectoryGenerator {
 public:
  static constexpr size_t N_DIMS = 3;
  using DimVector = Eigen::Matrix<double, N_DIMS, 1>;
  using SubTrajectory = ruckig::Trajectory<N_DIMS, ruckig::EigenVector>;

  enum class TargetMode { Distance, Duration };

  /// Maximum allowed acceleration in all dimensions
  static const DimVector& max_acceleration() {
    static const DimVector vec(1.0, 1.0, 1.0);
    return vec;
  }

  /// Maximum allowed deceleration in all dimensions
  static const DimVector& max_jerk() {
    static const DimVector vec(2.0, 2.0, 2.0);
    return vec;
  }

  /**
   * Construct a new Linear Velocity Trajectory which will accelerate and immediately decelerate.
   *
   * @param target_velocity Target velocity in the constant velocity phase.
   * @throws std::runtime_error if any subtrajectory could not be generated.
   */
  // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
  explicit LinearVelocityTrajectoryGenerator(DimVector target_velocity)
      : constant_velocity_(target_velocity), constant_velocity_duration_(0.0), constant_velocity_distance_(0.0) {
    LinearVelocityTrajectoryGenerator::generate_acceleration_subtrajectory(target_velocity);
    LinearVelocityTrajectoryGenerator::generate_deceleration_subtrajectory(target_velocity);
  }

  /**
   * Calculate the distance and duration during the constant velocity phase to obtain the total target distance.
   *
   * @param target_velocity Target velocity in the constant velocity phase.
   * @param target_distance The total distance to travel during the trajectory.
   *
   * @throws std::invalid_argument if the target_distance is too short to accomodate acceleration and deceleration.
   * @throws std::runtime_error if any subtrajectory could not be generated.
   */
  static LinearVelocityTrajectoryGenerator with_target_distance(DimVector target_velocity, double target_distance) {
    LinearVelocityTrajectoryGenerator trajectory_generator(target_velocity);

    // Calculate the distance left to travel in constant velocity after acceleration and deceleration
    trajectory_generator.constant_velocity_distance_ =
      target_distance - trajectory_generator.acceleration_distance_ - trajectory_generator.deceleration_distance_;

    // Ensure that the distance is realizable
    if (trajectory_generator.constant_velocity_distance_ < 0.0) {
      throw std::invalid_argument("Target distance is too short to accommodate acceleration and deceleration.");
    }

    // Calculate how long the constant velocity phase will take
    trajectory_generator.constant_velocity_duration_ =
      trajectory_generator.constant_velocity_distance_ / target_velocity.norm();

    return trajectory_generator;
  }

  /**
   * Calculate the distance and duration during the constant velocity phase to obtain the total target duration.
   *
   * @param target_velocity Target velocity in the constant velocity phase.
   * @param target_duration The total duration of the trajectory.
   *
   * @throws std::invalid_argument if the target_duration is too short to accomodate acceleration and deceleration.
   * @throws std::runtime_error if any subtrajectory could not be generated.
   */
  static LinearVelocityTrajectoryGenerator with_target_duration(DimVector target_velocity, double target_duration) {
    LinearVelocityTrajectoryGenerator trajectory_generator(target_velocity);

    // Calculate the duration left to travel in constant velocity after acceleration and deceleration
    trajectory_generator.constant_velocity_duration_ =
      target_duration - trajectory_generator.acceleration_duration_ - trajectory_generator.deceleration_duration_;

    // Ensure that the duration is realizable
    if (trajectory_generator.constant_velocity_duration_ < 0.0) {
      throw std::invalid_argument("Target duration is too short to accommodate acceleration and deceleration.");
    }

    // Calculate how far to travel during the constant velocity phase
    trajectory_generator.constant_velocity_distance_ =
      target_velocity.norm() * trajectory_generator.constant_velocity_duration_;

    return trajectory_generator;
  }

  /**
   * Get the velocity at a specified time.
   * @param time_sec
   * @return `DimVector`
   */
  DimVector velocity_at_time(double time_sec) {
    DimVector pos;  // Unused
    DimVector vel;  // Will contain the velocity at a specified time
    DimVector acc;  // Unused

    // Before acceleration phase
    const double t_start = 0.0;
    if (time_sec < t_start) {
      return DimVector::Constant(0.0);
    }

    // Acceleration phase
    const double t_acc_start = t_start;
    const double t_acc_end = t_acc_start + acceleration_duration();
    if (time_sec >= t_acc_start && time_sec < t_acc_end) {
      acceleration_subtrajectory_.at_time(time_sec - t_acc_start, pos, vel, acc);
      return vel;
    }

    // Constant velocity phase
    const double t_const_vel_start = t_acc_end;
    const double t_const_vel_end = t_const_vel_start + constant_vel_duration();
    if (time_sec >= t_const_vel_start && time_sec < t_const_vel_end) {
      return constant_velocity_;
    }

    // Deceleration phase
    const double t_dec_start = t_const_vel_end;
    const double t_dec_end = t_dec_start + deceleration_duration();
    if (time_sec >= t_dec_start && time_sec < t_dec_end) {
      deceleration_subtrajectory_.at_time(time_sec - t_dec_start, pos, vel, acc);
      return vel;
    }

    // After deceleration phase
    return DimVector::Constant(0.0);
  }

  /// Get the acceleration subtrajectory.
  SubTrajectory acceleration_subtrajectory() const { return acceleration_subtrajectory_; }

  /// Get the duration of the acceleration phase.
  double acceleration_duration() const { return acceleration_duration_; }

  /// Get the distance travelled during the acceleration phase.
  double acceleration_distance() const { return acceleration_distance_; }

  /// Get the duration of the deceleration phase.
  double constant_vel_duration() const { return constant_velocity_duration_; }

  /// Get the distance travelled during the deceleration phase.
  double constant_vel_distance() const { return constant_velocity_distance_; }

  /// Get the deceleration subtrajectory.
  SubTrajectory deceleration_subtrajectory() const { return deceleration_subtrajectory_; }

  /// Get the duration of the deceleration phase.
  double deceleration_duration() const { return deceleration_duration_; }

  /// Get the distance travelled during the deceleration phase.
  double deceleration_distance() const { return deceleration_distance_; }

  std::string describe() const {
    std::stringstream description;
    description << "\nAcceleration: duration = " << acceleration_duration()
                << " s, distance = " << acceleration_distance() << " m";
    description << "\nConstant vel: duration = " << constant_vel_duration()
                << " s, distance = " << constant_vel_distance() << " m";
    description << "\nDeceleration: duration = " << deceleration_duration()
                << " s, distance = " << deceleration_distance() << " m";
    return description.str();
  }

 private:
  /**
   * Generate the acceleration phase up to a constant target velocity.
   *
   * @throws std::runtime_error if the subtrajectory could not be generated.
   */
  void generate_acceleration_subtrajectory(DimVector target_velocity) {
    ruckig::Ruckig<N_DIMS, ruckig::EigenVector>               generator;
    const ruckig::InputParameter<N_DIMS, ruckig::EigenVector> input = generator_input(
      DimVector::Constant(0.0),  // Assuming we are already at zero speed
      target_velocity            // We want to do up to the target speed speed
    );

    // Generate the result and store the resulting subtrajectory
    const ruckig::Result result = generator.calculate(input, acceleration_subtrajectory_);
    if (result < 0) {
      throw std::runtime_error("Acceleration subtrajectory could not be computed");
    }
    acceleration_duration_ = acceleration_subtrajectory_.get_duration();
    acceleration_distance_ = subtrajectory_distance(acceleration_subtrajectory_);
  }

  /**
   * Generate the deceleration phase up to a constant target velocity.
   *
   * @throws std::runtime_error if the subtrajectory could not be generated.
   */
  void generate_deceleration_subtrajectory(DimVector target_velocity) {
    ruckig::Ruckig<N_DIMS, ruckig::EigenVector>               generator;
    const ruckig::InputParameter<N_DIMS, ruckig::EigenVector> input = generator_input(
      target_velocity,          // Assuming we are already at constant speed
      DimVector::Constant(0.0)  // We want to do down to zero speed
    );

    // Generate the result and store the resulting subtrajectory
    const ruckig::Result result = generator.calculate(input, deceleration_subtrajectory_);
    if (result < 0) {
      throw std::runtime_error("Acceleration subtrajectory could not be computed");
    }
    deceleration_duration_ = deceleration_subtrajectory_.get_duration();
    deceleration_distance_ = subtrajectory_distance(deceleration_subtrajectory_);
  }

  /**
   * Helper function to construct the Ruckig generator inputs with some sensible default values.
   * @param starting_velocity Velocity at the start of the trajectory to be generated.
   * @param ending_velocity Velocity at the end of the trajectory to be generated.
   * @return `ruckig::InputParameter<N_DIMS, ruckig::EigenVector>`
   */
  // NOLINT(bugprone-easily-swappable-parameters)
  static ruckig::InputParameter<N_DIMS, ruckig::EigenVector> generator_input(
    DimVector starting_velocity, DimVector ending_velocity  // NOLINT(bugprone-easily-swappable-parameters)
  ) {
    ruckig::InputParameter<N_DIMS, ruckig::EigenVector> input;

    // Set velocity control interface
    input.control_interface = ruckig::ControlInterface::Velocity;

    // Use the maximum values as specified in this class
    input.max_acceleration = LinearVelocityTrajectoryGenerator::max_acceleration();
    input.max_jerk = LinearVelocityTrajectoryGenerator::max_jerk();

    // We assume 0 position at the start of this subtrajectory.
    input.current_position = DimVector::Constant(0.0);

    // For this subtrajectory, we assume to already be at a constant target velocity
    input.current_velocity = starting_velocity;
    input.current_acceleration = DimVector::Constant(0.0);

    // We want to ramp down to zero velocity and have no more motion after that
    input.target_velocity = ending_velocity;
    input.target_acceleration = DimVector::Constant(0.0);

    return input;
  }

  /**
   * Get the distance travelled in a subtrajectory. This assumes the subtrajectory contains only linear motion.
   */
  static double subtrajectory_distance(ruckig::Trajectory<N_DIMS, ruckig::EigenVector> subtrajectory) {
    const double ramp_up_time = subtrajectory.get_duration();
    DimVector    ramp_up_final_position;
    DimVector    ramp_up_final_velocity;
    DimVector    ramp_up_final_acceleration;
    subtrajectory.at_time(ramp_up_time, ramp_up_final_position, ramp_up_final_velocity, ramp_up_final_acceleration);
    return ramp_up_final_position.norm();
  }

  // Acceleration phase subtrajectory
  SubTrajectory acceleration_subtrajectory_;
  double        acceleration_duration_;
  double        acceleration_distance_;

  // Constant speed subtrajectory
  DimVector constant_velocity_;
  double    constant_velocity_duration_;
  double    constant_velocity_distance_;

  // Deceleration phase subtrajectory
  SubTrajectory deceleration_subtrajectory_;
  double        deceleration_duration_;
  double        deceleration_distance_;
};

static constexpr double CONTROLLER_PERIOD = 0.001;

}  // namespace linear_velocity_controller

#endif  // LINEAR_VELOCITY_CONTROLLER_GENERATE_TRAJECTORY
