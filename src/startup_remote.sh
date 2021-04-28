#!/bin/bash
EIGENVALUE_TOPIC='damping_eigenvalues'

# ros2 param set /robot_interface $EIGENVALUE_TOPIC "[50.0, 12.0, 12.0, 2.0, 1.0, 1.0]"
# ros2 param set /robot_interface $EIGENVALUE_TOPIC "[35.0, 12.0, 12.0, 2.0, 1.0, 1.0]"
# ros2 param set /robot_interface $EIGENVALUE_TOPIC "[25.0, 12.0, 12.0, 2.0, 1.0, 1.0]"
# ros2 param set /robot_interface $EIGENVALUE_TOPIC "[35.0, 12.0, 12.0, 2.0, 1.0, 1.0]"

# Simulation gains
# ros2 param set /robot_interface $EIGENVALUE_TOPIC "[3.0, 3.0, 3.0, 2.0, 1.0, 1.0]"

# ros2 param set /robot_interface orientation_controller_damping "[0.0, 0.0, 0.0, 1.0, 1.0, 1.0]"
# ros2 param set /robot_interface orientation_controller_inertia
# ros2 param set /robot_interface orientation_controller_stiffness


# Gain Working [2021-03
ros2 param set /robot_interface orientation_controller_damping "[40.0, 40.0, 40.0, 3.0, 3.0, 3.0]"
ros2 param set /robot_interface orientation_controller_stiffness "[30.0, 30.0, 30.0, 1.0, 1.0, 1.0]"


ros2 param set /motion_generator clamping_values "[0.25, 0.25, 0.01, 0.01]"

ros2 param set /motion_generator distance_tolerance 0.01

ros2 lifecycle set /ft_sensor_interface configure
ros2 lifecycle set /robot_interface configure
ros2 lifecycle set /motion_generator configure

ros2 lifecycle set /ft_sensor_interface activate
ros2 lifecycle set /robot_interface activate
ros2 lifecycle set /motion_generator activate





