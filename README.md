# Jacob's Ladder
 Jacob’s Ladder is a modular, system-agnostic UAV command-and-control (C2) framework designed to operate seamlessly across varied hardware platforms running PX4. By abstracting communication between the flight computer and flight controller, it enables the same autonomy logic to run on different UAV configurations without hardware-specific rewrites. Building on the original "tracktor-beam" repository (https://github.com/ARK-Electronics/tracktor-beam), Jacob’s Ladder incorporates the PX4 ROS 2 message translation node, Docker-based development environments, and streamlined launch scripts to create a more versatile, reliable, modifiable, and shareable ROS 2 workspace for drone autonomy. This packaged approach eliminates the need for users to manually configure their machine environment, set up ROS 2 workspaces, or troubleshoot PX4 message compatibility and simulation setup issues—allowing them to focus entirely on developing autonomy algorithms and mission logic. The current implementation demonstrates this capability through a precision landing system, integrating a ROS 2–based perception and control pipeline with a hardware–software bridge for detecting, tracking, and autonomously landing on stationary or moving targets. The framework’s flexible architecture supports custom landing patterns, target tracking algorithms, and diverse sensor configurations, enabling rapid adaptation to a wide range of mission scenarios.

### Prerequisites
* Ubuntu Linux Machine (any version)
* Docker Engine
* PX4 Autopilot (any version)
* Micro XRCE-DDS Agent
* QGroundControl Daily Build

You can find the required instructions collected below
##### PX4
Run the following command lines within your host terminal:
```
mkdir src
cd src
git clone https://github.com/PX4/PX4-Autopilot.git
```
The latest PX4-Autopilot is recommended, but this repository contains a message translation node that allows for compatibility with any version of PX4. However, some versions of PX4 do not contain the gazebo worlds and models necessary for simulation. If this is the case for your PX4 version, refer to the README.md contained within the gazebo folder of this repository for further instructions. 

##### QGC
https://docs.qgroundcontrol.com/master/en/qgc-user-guide/releases/daily_builds.html

##### Using Jacob's Ladder
Navigate to the directory you would like to place the workspace ans then run the following command:
```
git clone https://github.com/Jacob-Safeer/Jacob_Ladder.git
```
Then navigate into the workspace:
```
cd Jacob_Ladder
```
Install Submodules:
```
git submodule update --init --recursive
```
The rest of the workspace setup will be completed within the docker container.

##### Docker
Install the docker engine from the following instructions:
https://docs.docker.com/engine/install/ubuntu/ 
Setup and run the docker container:
```
# enable access to xhost from the container
xhost +

# Run docker and open bash shell
docker run -it --privileged \
--env=LOCAL_USER_ID="$(id -u)" \
-v ~/src/PX4-Autopilot:/src/PX4-Autopilot/:rw \
-v ~/path/to/Jacob_Ladder:/path/to/Jacob_Ladder/:rw \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-e DISPLAY=:0 \
--network host \
--name=<local_container_name> jacobsafeer/px4-dev-harmonic-jammy-humbl-opencv-rqt:latest bash
```
Replace "/path/to" with the directory path to where you cloned Jacob_Ladder on your host machine. Replace "<local_container_name>" with whatever you would like to name your container.

After running these command lines, your docker container will now open. Run the following commands within the docker container to complete the setup of your ros2 ws:
```
cd /path/to/Jacob_Ladder
source /opt/ros/humble/setup.bash
colcon build
```
Now everything you need to run your simulation is set up. Exit the container and continue to the steps of running the example to verify that your setup was successful.


### Running the example simulation
Customize your launch bash script using the template provided:
```
#!/bin/bash

container_name="<local_container_name>"
user="user"

# Tab names
tab_names=("PX4-SITL" "RQT-Image" "Translation-Node" "Aruco-Tracker" "Precision-Land")

# Commands to run in each tab
commands=(
    "cd /src/PX4-Autopilot && make px4_sitl gz_x500_mono_cam_down_aruco; exec bash"
    "cd /path/to/Jacob_Ladder && source install/setup.bash && ros2 run rqt_image_view rqt_image_view; exec bash"
    "cd /path/to/Jacob_Ladder && source install/setup.bash && ros2 run translation_node translation_node_bin; exec bash"
    "cd /path/to/Jacob_Ladder && source install/setup.bash && ros2 launch aruco_tracker v1_16_tracker.launch.py; exec bash"
    "cd /path/to/Jacob_Ladder && source install/setup.bash && ros2 launch precision_land precision_land.launch.py; exec bash"
)

# Start gnome-terminal with the first tab
docker_cmd="docker exec -it --user ${user} ${container_name} bash -c \"${commands[0]}\""
gnome-terminal --tab --title="${tab_names[0]}" -- bash -c "${docker_cmd}"

# Open the rest of the tabs
for i in "${!commands[@]}"; do
    if [ $i -eq 0 ]; then
        continue
    fi

    # Add delay only for Precision-Land
    if [ $i -eq 4 ]; then
        docker_cmd="docker exec -it --user ${user} ${container_name} bash -c \"sleep 10; ${commands[$i]}\""
    else
        docker_cmd="docker exec -it --user ${user} ${container_name} bash -c \"${commands[$i]}\""
    fi

    gnome-terminal --tab --title="${tab_names[$i]}" -- bash -c "${docker_cmd}"
    sleep 1
done
```
Change the <local_container_name> to match the one you created. Change the command lines to fit the px4 version you are using and mission you are aiming to execute. 
* precision_land folder 
- in the default 'aruco' world, use precision_land.launch.py
- in the custom 'moving_platform' world, use track_follow.launch.py
* aruco_tracker folder
- for px4 1.15.x, use aruco_tracker.launch.py 
- for px4 1.16.x, 
-- default 'aruco' world: use v1_16_tracker.launch.py 
-- custom 'moving_platform' world, use moving_aruco.launch.py

Launch QGC:
```
./QGroundControl.AppImage
```
Launch micro dds:
```
MicroXRCEAgent udp4 -p 8888
```
Start the docker container that corresponds to your launch bash script:
```
docker start <local_container_name>
```
Navigate to the Jacob_Ladder repo from your host terminal and run the launch bash script:
```
./<launch_script_name>.sh
```
This will open separate docker container terminals to launch each of the different programs necessary to run the simulation. 


![](Precision.png)


## Questions
email me at jsafeer1@terpmail.umd.edu

## Additional resources
[LinuxCheatSheet](https://www.geeksforgeeks.org/linux-commands-cheat-sheet/)

[ROS2CheatSheet](https://www.theconstruct.ai/wp-content/uploads/2021/10/ROS2-Command-Cheat-Sheets-updated.pdf)

[CMakeBasics](https://nu-msr.github.io/navigation_site/lectures/cmake_basics.html)

### Hardware
#### Prerequisites
* Ubuntu 22.04
* ROS2 Humble
* Drone with downward facing camera
* Micro XRCE-DDS Agent
* QGroundControl
* OpenCV 4.10.0
* Camera node:
I use an usb camera, there is a ROS2 package already out there for it:
https://github.com/ros-drivers/usb_cam.git

You can either run the the nodes or turn them into a service, that starts at boot:
#### Normal run

##### Service

First, you need to move your service file to the /etc/systemd/system/ directory, where systemd can find it. Replace myservice.service with the actual name of your service file.

Ensure that the service file has the correct permissions. Typically, it should be readable by all users:
```
sudo chmod 644 /etc/systemd/system/myservice.service

```
After copying the service file, reload the systemd daemon to recognize the new service:

```
sudo systemctl daemon-reload

```
Start the service using systemctl:

```
sudo systemctl start myservice

```
If you want the service to start automatically on boot, enable it:

```
sudo systemctl enable myservice

```
Verify that the service is running correctly:
```
sudo systemctl status myservice

```
