# tasc_autonomy_2027

This is the ROS 2 Jazzy workspace for the TASC Autonomous Navigation Subteam, made to be used in CIRC 2027. 

# Current Progress / Snapshot

Just beginning:
- converting Humble -> Jazzy
- implement URDF and sensors

# Workspace Layout

**YOU MUST ADHERE TO THIS STRUCTURE. DO NOT CREATE EXTRA PACKAGES OR SUB FOLDERS WITHOUT CONSULTING THE LEADS**

Recall, these should all be C++ packages. 
```
~/<your_workspace_name> # this if your main workspace folder, name it as you please in your PC
  ├── build
  ├── install
  ├── log
  └── src
      └── tasc_autonomy_2027
          ├── autonav_bringup # launch files per CIRC tasc and other mass tests
          ├── autonav_navigation # main Nav2 folder
          ├── autonav_sensors # to hold sensor implementation and related code
          ├── autonomy_vision # main computer vision and streaming folder
          ├── autonav_description # urdf files and related TF
```

# Installation 

## Download and link repo

1) Create your workspace folders and cd to the src folder
```bash
mkdir -p ~/<your_folder_name>/src
cd ~/<your_folder_name>/src
```
2) Git clone this main branch
```bash
git clone https://github.com/Combat-Craft/tasc_autonomy_2027.git
```
3) Use Git status to ensure you're on main
```bash
git status
```

## Build
Go to workspace root i.e. ```cd ~/<your_folder_name>```
**ONLY BUILD IN YOUR ROOT WORKSPACE FOLDER** It ensures the build, install, and log folders and respective files are all within ~/<your_folder_name> and NOT in src or any other folder!

1) Source ROS 2 Jazzy
```bash
source /opt/ros/jazzy/setup.bash

```
2) Get all dependencies installed, such as rplidar.
```bash
rosdep install --from-paths src -y --ignore-src -r
```

3) Build tasc_autonomy_2027

symlink should allow changes to go through without having to re-colcon build
```bash
colcon build --symlink-install 
```
To rebuild one specific package
```bash
colcon build --packages-select <package> 
```
To build all except a certain package
```bash
colcon build --packages-ignore <name-of-package>
```

4) Source the packages
```bash
source install/setup.bash
```

Notes:
- Use `source install/setup.bash` (not `source install/bash`).
- If you also use a virtual environment, activate it before build/run and ensure ROS Python packages are available there.
