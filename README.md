The double pendulum, a classic example of a chaotic system, exhibits intricate and unpredictable behavior that is highly sensitive to its initial conditions. This system consists of two pendulums attached end to end, where the motion of the second pendulum depends on the first. One of the most fascinating aspects of a double pendulum is its sensitivity to initial parameters, including the starting angles and velocities of the pendulums.

Small changes in these initial parameters can lead to vastly different trajectories, a phenomenon known as sensitivity to initial conditions. This sensitivity is a hallmark of chaotic systems, where predictability is extremely limited over long time periods. For example, if one double pendulum starts at a slightly different angle than an identical pendulum, their motions will diverge significantly over time, making long-term prediction impossible.

The dependence on initial conditions is not just limited to angles but also includes the initial angular velocities. Even if two double pendulums start at the same angle but with slightly different angular velocities, their paths will diverge dramatically. This unpredictability is due to the nonlinear dynamics governing the motion of the pendulums, where the equations of motion do not yield simple, predictable solutions.

Moreover, the energy in a double pendulum system can transfer between the two pendulums in complex ways, depending on these initial conditions. This transfer of energy can lead to scenarios where one pendulum is momentarily stationary while the other swings wildly, and vice versa. The study of double pendulums not only offers insights into the behavior of chaotic systems but also has applications in various fields, including physics, engineering, and even biology, where similar sensitive dependencies on initial conditions are observed.

rm -rf install/ log/ build/
colcon build
source install/local_setup.bash
ros2 launch double_pendulum double_pendulum.launch.py
ros2 launch system_with_spring system_with_spring.launch.py
