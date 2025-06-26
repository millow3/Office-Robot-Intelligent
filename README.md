# Office-Robot-Intelligent
Office Robot Intelligent – TTTC2343 Project

👨‍💻 Contributors
Anas Aeyman bin Nagur Aziz (A204323)
Mohammad Farihin bin Roslan (A201909)

✅ Features
- Office environment simulation in Gazebo
- SLAM mapping using gmapping
- Navigation using move_base with RRT planner
- Terminal input for destination rooms
- Auto-return to origin after task
- RViz used for map visualization and 2D Nav Goal

🚀 How to Run
1. Launch Office World:
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_house.launch

2. Run SLAM + RViz:
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

3. Teleoperate to Explore:
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

4. Save Map:
rosrun map_server map_saver -f ~/map/office_map

5. Launch Navigation with Saved Map:
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/farihin/map/office_map.yaml

6. Run Multi-Point Navigation Script:
rosrun my_office_world multi_nav.py
