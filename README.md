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
