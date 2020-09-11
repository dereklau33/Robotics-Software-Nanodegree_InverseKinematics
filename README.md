# Robotics-Software-Nanodegree_InverseKinematics

# Goal
Use the Kuka Arm, provided by Udacity's Pick and Place project, to implement forwards and inverse kinematics to actuate the robotic arm to pick up and place an object into a bin given the desired locations. 

# Project Requirements
ROS Kinetic
Gazebo >= 7.0  

# Project Setup
Run Update On Linux Command Line:   
```bash
$ sudo apt-get update && sudo apt-get upgrade -y
``` 

# Run Instructions
Create a workspace:    
```bash
$ mkdir -p ~/home/workspace/
$ cd /home/workspace/
```   

Copy code from previous project WhereAmI:
```bash
$ git clone https://github.com/dereklau33/Robotics-Software-Nanodegree_InverseKinematics.git
```

Build package:  
```bash
$ cd ..
$ source devel/setup.bash
$ catkin_make
```

Launch the Simulator:
```bash
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh
```   
Ensure that in the inverse_kinematics.launch file under /RoboND-Kinematics-Project/kuka_arm/launch that the demo flag is labeled "false" 

To run Inverse Kinematics: 
Run the following in a new terminal
```bash
rosrun kuka_arm IK_server.py
```  

Follow the instructions on Rviz to see the Inverse Kinematics code in action
