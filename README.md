# Drone Repository
This repository contains the code for the autonomous drones for the XPRIZE Snowflake team. Code scope covers subsystems of Communication, Detection, and Navigation. Steps have been taken to make sure all the necessary code is contained in this repository. If an onboard computer is corrupted, or more drones are added to the fleet, setting up the workspace is very simple.

## Setup:
1. Clone the repository into the home directory (`~/`) of the Raspberry Pi or similar onboard computer using `git clone git@github.com:XPRIZE-Snowflake/high_alt_drone.git`.

    - If you are cloning on a new onboard computer and receive an error similar to `Permission denied (publickey).`, follow [these steps](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) to generate a new ssh key-value pair and give you permission to perform Git actions.

2. Once the repository is cloned, it will be located in a directory named `~/high_alt_drone`. We now want to rename that directory to be called **ros2_ws** to be consistent with the rest of our workspaces. We can use `mv ~/high_alt_drone ~/ros2_ws` to rename it. Now go into that directory.

3. Once in that directory, we will install the Seek Thermal SDK files so we can communicate with the thermal camera. Use `sudo apt-get install ./seekthermal-sdk-dev-{version}_arm64.deb` to install the files. Then run `sudo udevadm control --reload` to reload the udev rules as per the Seek Thermal Quick Start Guide.
    - As a precaution, download this extra dependency for the Seek Thermal SDK. It is sometimes pre-installed, but not always. `sudo apt-get install libsdl2-dev`
  
4. Now try and build the workspace. Make sure you are in the main directory of the workspace (`~/ros2_ws`) and run `colcon build`. Due to the size of the **px4_msgs** package, this will take about 15-20 minutes. After you successfully build the first time you can build individual packages to save time. 

# Code Organization: 

## Src:
All current and future packages will be found in the **/src** folder.

### custom_msgs
This folder houses the custom message files for the repository. This will to publish custom message types and recieve the necessary data.

### img_capture
Contains the ROS2 nodes and python scripts for the subteams. The **/scripts** folder contains the python scripts to receive and send camera data. The **/src** folder contains the C++ seekcamera_publisher.cpp. This publisher sends the camera images to the ROS network so that our python scripts can filter and gather data.

### new_px4_msgs & new_px4_ros_com
These use to be nested repositories, but caused extra issues when cloning. We now only use the files themselves. In these folders are everything we need to communicate with the PX4. We can send commands via preset messages.
