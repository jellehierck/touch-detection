#ifndef LINEAR_VELOCITY_CONTROLLER_GENERATE_TRAJECTORY
#define LINEAR_VELOCITY_CONTROLLER_GENERATE_TRAJECTORY

#include <array>
#include <cstddef>
#include <vector>

#include <ruckig/ruckig.hpp>
#include <ruckig/input_parameter.hpp>
#include <ruckig/output_parameter.hpp>
#include <ruckig/utils.hpp>

namespace linear_velocity_controller::trajectory {

static constexpr size_t N_DIMS = 3;
static constexpr double CONTROLLER_PERIOD = 0.001;

std::vector<std::array<double, N_DIMS>> generate_acceleration_phase(std::array<double, N_DIMS> target_vel) {
  // Create instances: the Ruckig OTG as well as input and output parameters
  ruckig::Ruckig<N_DIMS>          generator(CONTROLLER_PERIOD);  // control cycle
  ruckig::InputParameter<N_DIMS>  generator_input;
  ruckig::OutputParameter<N_DIMS> generator_output;

  std::vector<std::array<double, N_DIMS>> velocities_result = {};

  // Set input parameters and velocity control interface
  generator_input.control_interface = ruckig::ControlInterface::Velocity;

  generator_input.current_position = {0.0, 0.0, 0.0};
  generator_input.current_velocity = {0.0, 0.0, 0.0};
  generator_input.current_acceleration = {0.0, 0.0, 0.0};

  generator_input.target_velocity = target_vel;
  generator_input.target_acceleration = {0.0, 0.0, 0.0};

  generator_input.max_velocity = {0.5, 0.5, 0.5};
  generator_input.max_acceleration = {1.0, 1.0, 1.0};
  generator_input.max_jerk = {2.0, 2.0, 2.0};

  // Generate the trajectory within the control loop
  // std::cout << "t | position | velocity" << std::endl;
  while (generator.update(generator_input, generator_output) == ruckig::Result::Working) {
    // std::cout << generator_output.time << " | " << ruckig::join(generator_output.new_position, N_DIMS) << " | "
              // << ruckig::join(generator_output.new_velocity, N_DIMS) << std::endl;
    generator_output.pass_to_input(generator_input);
    velocities_result.emplace_back(generator_output.new_velocity);
  }

  std::cout << "Trajectory duration: " << generator_output.trajectory.get_duration() << " [s]." << std::endl;
  return velocities_result;
}

}  // namespace linear_velocity_controller::trajectory

#endif  // LINEAR_VELOCITY_CONTROLLER_GENERATE_TRAJECTORY
