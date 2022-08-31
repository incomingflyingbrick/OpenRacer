source /opt/ros/galactic/setup.bash
cd ~/ros2_ws/
sudo colcon build --packages-select ai_racer engine_node
. install/setup.bash
ros2 launch /home/jetson/ros2_ws/src/ai_racer/launch/ai_racer_launch.xml