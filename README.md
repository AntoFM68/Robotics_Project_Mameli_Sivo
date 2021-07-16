# Robotics_Project_Mameli_Sivo

**robot_model:** "uv", "drug", "drug1"

**World launch:** roslaunch robotics world.launch

**Robot Rviz launch:** roslaunch robotics rviz_robot.launch robot:="robot_model"

**Robot Gazebo launch:** roslaunch robotics robot.launch robot:="robot_model"

**Robot Gazebo RQT Steering launch:** roslaunch robotics gazebo_rqt_steering.launch robot:="robot_model"

**Teleop Keyboard launch:** roslaunch robotics teleop_key.launch robot:="robot_model"

**Slam launch:** roslaunch robotics slam_gmapping.launch robot:="robot_model"

**Navigation launch:** roslaunch robotics navigation.launch robot:="robot_model"

**Multi Navigation:**

roslaunch robotics map_rviz_multi.launch

roslaunch robotics navigation_multi.launch

roslaunch robotics navigation_multi.launch robot:="drug" x:="-5" y:="-1"

roslaunch robotics navigation_multi.launch robot:="drug1" x:="-4" y:="-1"

roslaunch robotics TF_multi.launch

rosrun robotics uv_goal.py

rosrun robotics drug_LF.py
