# Fixed Wing UAV Mission Flight Simulation Environment
2022 yılı Teknofest Yarışması Sabit Kanat İHA kategorisi görev yazılımı reposudur.
## Setup

Create the new catkin package
```bash
cd ~/catkin_ws/src
catkin_create_pkg roscamera
mkdir roscamera/scripts
```

Run in the project directory
```bash
cp CMakeLists.txt ~/catkin_ws/src/roscamera/scripts
cp mission.py ~/catkin_ws/src/roscamera/scripts
cp roscamera.py ~/catkin_ws/src/roscamera/scripts
cp utils.py ~/catkin_ws/src/roscamera/scripts
```

Set the main script <roscamera.py> as executable
```bash
cd ~/catkin_ws/src/roscamera/scripts
chmod +x roscamera.py
```

## Running

All lines in different terminals

Run ROS core:
```bash
roscore
```

Run gazebo with ROS:
```bash 
rosrun gazebo_ros --verbose <.word file location>
```
<.word file location> = ~/Downloads/pist2.world for my case

Export API with Python socket:
```bash
cd ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --map
```

Run our code with ROS:
```bash
rosrun roscamera roscamera.py
```

## Editting
For my case Python script are in:

/home/dad/
  - FixedWing/ardupilot
  - catkin_ws/src/roscamera/scripts  # Path for roscamera.py and utils.py

You can modify the roscore.py script for differing behaviour.

  
