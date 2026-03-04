# Jacob's Ladder
 Jacob’s Ladder is a modular, system-agnostic UAV command-and-control (C2) framework designed to operate seamlessly across varied hardware platforms running PX4. By abstracting communication between the flight computer and flight controller, it enables the same autonomy logic to run on different UAV configurations without hardware-specific rewrites. Building on the original "tracktor-beam" repository (https://github.com/ARK-Electronics/tracktor-beam), Jacob’s Ladder incorporates the PX4 ROS 2 message translation node, Docker-based development environments, and streamlined launch scripts to create a more versatile, reliable, modifiable, and shareable ROS 2 workspace for drone autonomy. This packaged approach eliminates the need for users to manually configure their machine environment, set up ROS 2 workspaces, or troubleshoot PX4 message compatibility and simulation setup issues—allowing them to focus entirely on developing autonomy algorithms and mission logic. The current implementation demonstrates this capability through a precision landing system, integrating a ROS 2–based perception and control pipeline with a hardware–software bridge for detecting, tracking, and autonomously landing on stationary or moving targets. The framework’s flexible architecture supports custom landing patterns, target tracking algorithms, and diverse sensor configurations, enabling rapid adaptation to a wide range of mission scenarios.

## Prerequisites
* Ubuntu Linux Machine (any version)
* Docker Engine
* PX4 Autopilot (any version)
* QGroundControl Daily Build

You can find the required instructions collected below
### PX4
Run the following command lines within your host terminal:
```
mkdir src
cd src
git clone https://github.com/PX4/PX4-Autopilot.git
```
The v1.16.0 tag of PX4-Autopilot is recommended, but this repository contains a message translation node that allows for compatibility with most versions of PX4 from 1.16 and forward. However, some versions of PX4 do not contain the gazebo worlds and models necessary for simulation. If this is the case for your PX4 version, refer to PX4's official worlds and models documentation within the user guide.The gazebo folder of this repository contains the custom world that I created for simulation of maritime application autonomous precision landing.

### QGC
Install using the link below:
https://docs.qgroundcontrol.com/master/en/qgc-user-guide/releases/daily_builds.html

### Using Jacob's Ladder
Navigate to the directory you would like to place the workspace and then run the following command:
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

### Docker
Within the /docker directory of this repository, you’ll find a stack of Dockerfiles designed to streamline setup and ensure a consistent development environment. The base image is built from px4io/px4-dev-base-jammy, with successive layers adding all dependencies required to run the full Jacob_Ladder workspace. By using these prebuilt images, available on my Docker Hub (https://hub.docker.com/repositories/jacobsafeer), users can skip tedious installation steps, avoid ROS 2 message compatibility pitfalls, and bypass common simulation setup headaches. The environment is fully encapsulated, so you can focus entirely on developing autonomy algorithms and mission logic—without worrying about local machine configuration or dependency management. Follow the instructions below to properly utilize my docker image:

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
--name=<local_container_name> jacobsafeer/px4-dev-harmonic-jammy-humble-opencv-rqt:latest bash
```
Replace "/path/to" with your directory path to where you cloned Jacob_Ladder on your host machine. Replace "<local_container_name>" with whatever you would like to name your container.

After running these command lines, your docker container will now open. Run the following commands within the docker container to complete the setup of your ros2 ws:
```
cd /path/to/Jacob_Ladder
source /opt/ros/humble/setup.bash
colcon build
```
Now everything you need to run your simulation is set up. Exit the container and continue to the steps of running the example to verify that your setup was successful.


## Running the example simulation
Customize your launch bash script using the template provided:
```
#!/bin/bash

container_name="<local_container_name>"
user="user"

# Tab names
tab_names=("PX4-SITL" "DDS-Agent" "RQT-Image" "Translation-Node" "Aruco-Tracker" "Precision-Land")

# Commands to run in each tab
commands=(
    "cd /src/PX4-Autopilot && make px4_sitl gz_x500_mono_cam_down_aruco; exec bash"
    "cd /path/to/Jacob_Ladder && source install/setup.bash && MicroXRCEAgent udp4 -p 8888; exec bash"
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
        docker_cmd="docker exec -it --user ${user} ${container_name} bash -c \"sleep 15; ${commands[$i]}\""
    else
        docker_cmd="docker exec -it --user ${user} ${container_name} bash -c \"${commands[$i]}\""
    fi

    gnome-terminal --tab --title="${tab_names[$i]}" -- bash -c "${docker_cmd}"
    sleep 1
done
```
Change the <local_container_name> to match the one you created. Change the command lines to fit the px4 version you are using and mission you are aiming to execute. 
* precision_land tab 
    - in the default 'aruco' world, use precision_land.launch.py
    - in the custom 'moving_platform' world, use track_follow.launch.py
* aruco_tracker tab
    - for px4 1.15.x, use aruco_tracker.launch.py 
    - for px4 1.16.x, 
        + default 'aruco' world, use v1_16_tracker.launch.py 
        + custom 'moving_platform' world, use moving_aruco.launch.py

Launch QGC:
```
./QGroundControl.AppImage
```
Start the docker container that corresponds to your launch bash script:
```
docker start <local_container_name>
```
Navigate to the Jacob_Ladder repo from your host terminal and run the launch bash script:
```
./<launch_script_name>.sh
```
This will open separate docker container terminals to launch each of the different programs necessary to run the simulation. If you previously ran make px4_sitl in your local repo, you may need to run the following line in the px4-Autopilot directory within the running container:
```
rm -rf build/px4_sitl_default
```
After doing this, stop the container and repeat the steps to start the container and run the launch script:
```
docker stop <local_container_name>
```
Activate the precision landing custom mode by selecting it from the dropdown bar in QGC as pictured below:

![](Precision.png)


## Questions
email me at jsafeer1@terpmail.umd.edu

## Additional resources
[LinuxCheatSheet](https://www.geeksforgeeks.org/linux-commands-cheat-sheet/)

[ROS2CheatSheet](https://www.theconstruct.ai/wp-content/uploads/2021/10/ROS2-Command-Cheat-Sheets-updated.pdf)

[CMakeBasics](https://nu-msr.github.io/navigation_site/lectures/cmake_basics.html)

## Real Flight Testing
### Hardware Requirements
* Nvidia Jetson Orin Nano Super Developer Kit
* Pixhawk Orange Cube Plus
* USB to TTL converter
* Optical Flow Sensor or other pose estimation source

### Step 1: Set up drone for manual flight
Assemble the drone with everything necessary to fly properly in both Stabilized and Position flight modes before evaluating any autonomous missions. This step is crucial for verifying that the airframe, propulsion, and flight controller operate reliably independent of the Jacob’s Ladder autonomy stack.

Required actions:

* Flash the Cube flight controller with a supported PX4 firmware version

* Perform full sensor calibration (accelerometer, gyroscope, magnetometer, level horizon)

* Calibrate ESCs and verify proper motor direction and response

* Configure RC transmitter and verify manual control authority

* Set up telemetry communication with QGroundControl

* Verify stable hover performance in:
    - Stabilized mode
    - Position mode (requires GPS or optical flow / external pose estimation)

The vehicle must demonstrate stable and repeatable manual flight before proceeding.

### Step 2: Set up Jetson onboard computer
Flash the Jetson with JetPack and install Ubuntu 22.04.

Due to compute architecture differences between x86 (PC) and ARM64 (Jetson), software packages and containers built for simulation may not work directly on the Jetson. A separate Docker image is available on DockerHub for ARM64 compatibility, but initial setup and debugging are easier without Docker.

Install and build the ROS 2 workspace directly on the Jetson.

#### Step 2.1: Verify offboard nodes using networked simulation
Before connecting to real hardware, verify the autonomy stack using a networked PX4 SITL simulation.

On the PC in two separate terminals:
```
cd ~/src/PX4-Autopilot && make px4_sitl gz_x500_mono_cam_down_aruco; exec bash
```
```
cd ~/path/to/Jacob_Ladder && source install/setup.bash && MicroXRCEAgent udp4 -p 8888
```

On the Jetson:
```
cd ~/path/to/Jacob_Ladder && source install/setup.bash && ros2 run translation_node translation_node_bin
```
```
cd ~/path/to/Jacob_Ladder && source install/setup.bash && ros2 launch aruco_tracker v1_16_tracker.launch.py
```
```
cd ~/path/to/Jacob_Ladder && source install/setup.bash && ros2 launch precision_land precision_land.launch.py
```
Verify:

* ROS nodes run without errors
* DDS communication is functioning
* Commands are received by PX4
* Vehicle responds correctly in simulation

This step confirms the software stack is functioning properly on Jetson before hardware integration.

#### Step 2.2: Camera Node Setup

I use a usb camera, there is a ROS2 package already out there for it:
https://docs.luxonis.com/software-v3/depthai/ros/driver/

Run the ArUco tracker with topic remapping:

```
ros2 run aruco_tracker aruco_tracker \
  --ros-args \
  -r /camera:=/oak/rgb/image_raw \
  -r /camera_info:=/oak/rgb/camera_info
```

Verify correct operation using:
```
ros2 run rqt_image_view rqt_image_view
```
Confirm that:

* Camera images are streaming
* ArUco markers are detected correctly
* Detection behavior matches simulation

#### Step 2.3: Add startup script for Jetson hotspot connection
Step 2.3: Add startup script for Jetson hotspot connection

In order to connect to the Jetson during flight, you must be able to SSH into it. Many institutional or enterprise networks prevent peer-to-peer SSH connections or block device discovery. To ensure reliable access in all environments, configure the Jetson to automatically create its own Wi-Fi hotspot at boot.

This allows a laptop to connect directly to the Jetson without relying on external network infrastructure.


### Step 3: Connecting the Jetson to the Cube

##### The physical connection
Use a USB-to-TTL converter to connect: 
Jetson USB port → Cube Orange+ TELEM2 port
Typical wiring:
* Jetson TX → Cube RX
* Jetson RX → Cube TX
* Jetson GND → Cube GND
Do NOT connect 5V power between devices.
After connecting, verify the serial device appears on the Jetson:
```
ls /dev/ttyUSB*
```
#### Step 3.1: PX4 Parameter Configuration
The parameters file located within this repository matches the original experiments of this research exactly. You may not want to copy all the parameters exactly depending on how you plan to implement pose estimation and other features within your system. In fact, the only parameters that you really need to be concerned with to set up the Jacob's Ladder system are the MAVLink communications and UXRCE_DDS configuration. 

Required MAVLink parameters:
```
MAV_1_CONFIG = TELEM1
MAV_1_RATE   = 57600
```
Required uXRCE-DDS parameters:
```
UXRCE_DDS_CFG = TELEM2
UXRCE_DDS_BAUD = 921600
```
Make sure there are no MAVLink instances running on TELEM2
After setting parameters:
* Reboot the flight controller
* Verify no parameter errors in QGroundControl

#### Step 3.2: Start the Micro XRCE-DDS Agent
On the Jetson, start the DDS agent:
```
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```
You should see output indicating successful connection that mirrors the output you see in the XRCE-DDS window during simulation.

### Step 4: Verify ROS-PX4 Communication
Source the workspace:
```
source ~/jacobs_ladder_ws/install/setup.bash
```
Verify PX4 topics are visible:
```
ros2 topic list
```
Expected topics include:
```
/fmu/out/vehicle_odometry
/fmu/out/vehicle_status
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
```
Confirm data is publishing:
```
ros2 topic echo /fmu/out/vehicle_odometry
```
### Step 5: Offboard mode testing (propellers removed)

Before flight testing, verify offboard control safely without propellers.

Run:
```
ros2 run offboard_ladder velocity_test_node
```
Verify:

* No communication errors
* PX4 enters Offboard mode
* Setpoints are received correctly

Use QGroundControl to monitor vehicle state.