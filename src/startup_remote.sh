#!/bin/bash
# ros2 param set /robot_interface force_controller_damping_eigenvalues "[25.0, 12.0, 12.0, 2.0, 1.0, 1.0]"
# ros2 param set /robot_interface force_controller_damping_eigenvalues "[25.0, 12.0, 12.0, 4.0, 2.0, 2.0]"

# Simulation gains
# ros2 param set /robot_interface force_controller_damping_eigenvalues "[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]"
# ros2 param set /robot_interface force_controller_damping_eigenvalues "[3.0, 3.0, 3.0, 2.0, 1.0, 1.0]"

ros2 param set /robot_interface damping_eigenvalues "[3.0, 3.0, 3.0, 2.0, 1.0, 1.0]"

ros2 param set /motion_generator distance_tolerance 0.05

ros2 lifecycle set /ft_sensor_interface configure
ros2 lifecycle set /robot_interface configure
ros2 lifecycle set /motion_generator configure

ros2 lifecycle set /ft_sensor_interface activate
ros2 lifecycle set /robot_interface activate
ros2 lifecycle set /motion_generator activate





