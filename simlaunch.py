import subprocess
import time

container_name = "px4-rqt"
user = "user"

commands = [
    # Run the build, then drop into a shell
    "cd /src/PX4-Autopilot && make px4_sitl gz_x500_mono_cam_down_aruco; exec bash",
    "cd /aruco_land/Jacob_Ladder && source install/setup.bash && ros2 run rqt_image_view rqt_image_view; exec bash",
    "cd /aruco_land/Jacob_Ladder && source install/setup.bash && ros2 launch aruco_tracker aruco_tracker.launch.py; exec bash",
    "cd /aruco_land/Jacob_Ladder && source install/setup.bash && ros2 launch precision_land precision_land.launch.py; exec bash"
]

for command in commands:
    docker_command = f"docker exec -it --user {user} {container_name} bash -c \"{command}\""
    subprocess.run([
        "gnome-terminal", "--tab", "--", "bash", "-c", docker_command
    ])
    time.sleep(1)