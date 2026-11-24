# This sample package

This package is an example on how to use and maintain your ROS2 code to fit the CMake structure.

This includes where to place your files, key functions in C++, and how to add your executables to the package.xml and CMakelist.txt files.

This packages contains several example nodes from the official ROS 2 Jazzy documentation, including:
- basic publisher and subsriber
- basic publisher with a parameter
- a hello world example
- a launch file

## The Folder Structure

Keep the package folder structure to match this!

```
<your_workspace_folder>/
├── build/
├── install/
├── log/
└── src/
    └── tasc_autonomy_2027/
        ...
        └── sampleCppPackage/
            ├── config   # any config files, such as .yaml, go here
            ├── firmware # any firmware code goes here
            ├── include
            │   └── sampleCppPackage # your header files go here
            ├── launch # all launch folders go here
            ├── src  # your main node and related code goes here
            ├── package.xml  # list dependencies
            └── CMakeLists.txt # list dependicies, your nodes and link headers here
```
