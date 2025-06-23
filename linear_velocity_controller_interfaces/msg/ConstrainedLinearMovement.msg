# Constrained Linear Movement message.
# This message is meant as command input to the LinearVelocityController.
# It is used to let the controller move linearly until a specified distance is travelled.
# The directions and velocities are expressed in the frame of the end effector.
# For each cardinal direction (x,y,z), both the *_vel and *_max_dist field must be NOT_SET, or both must be nonzero.
# Multiple cardinal directions can be specified to allow diagonal movement.

# Metadata
std_msgs/Header header


# Setting a field to this value means it is not set and that cardinal direction is ignored.
# Note that for a cardinal direction (x,y,z), both the *_vel and *_max_dist field must be NOT_SET, or both must be nonzero.
float64 IGNORED=0.0

# Velocity (m/s) in the x-direction.
float64 x_vel 0.0
# Absolute distance (m) in the x-direction. Must be >= 0.0.
float64 x_max_dist 0.0

# Velocity (m/s) in the y-direction.
float64 y_vel 0.0
# Absolute distance (m) in the y-direction. Must be >= 0.0.
float64 y_max_dist 0.0

# Velocity (m/s) in the z-direction.
float64 z_vel 0.0
# Absolute distance (m) in the z-direction. Must be >= 0.0.
float64 z_max_dist 0.0

