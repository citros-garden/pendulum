# General Information 🌍
Write a few lines just to explain the project. 
You can add equations with the markdown syntax:

$$ x(t) = \int(v(t))dt + x_0 $$

# Installation 🛫
minimal installation commands.


# Build :tractor:
build commands. Preferred to point to VSCode task instead of CLI commands.


# Run 🚀
Run commands. Preferred to point to VSCode task instead of CLI commands


# Develop :bulb:
instruction to further develop the simulation.

# Extras :eyes:
Images / Videos from Foxglove

rm -rf install/ log/ build/
colcon build
source install/local_setup.bash
ros2 launch double_pendulum double_pendulum.launch.py
ros2 launch system_with_spring system_with_spring.launch.py
