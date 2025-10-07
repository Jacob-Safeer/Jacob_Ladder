# Gazebo Asset Installation for PX4

This directory holds custom Gazebo models (`./models`) and a world (`./worlds`)
that extend the default PX4 SITL environment. Follow the steps below to copy
these assets into your local `PX4-Autopilot` checkout so they are available to
the simulator.

## Prerequisites

- A local clone of `PX4-Autopilot`
- Write access to `PX4-Autopilot/Tools/simulation/gz`

## Copying the models

1. Set an environment variable pointing at your `PX4-Autopilot` directory:
   ```bash
   export PX4_DIR=~/src/PX4-Autopilot
   ```
   Adjust the path as needed for your setup.
2. Copy the models into the PX4 Gazebo model directory:
   ```bash
   cp -r ./models/ "$PX4_DIR/Tools/simulation/gz/models/"
   ```
3. Copy the world files:
   ```bash
   cp -r ./worlds/ "$PX4_DIR/Tools/sitl_gazebo/worlds/"
   ```

## Using the assets

- Launch PX4 SITL with the new world, for example:
  ```bash
  cd "$PX4_DIR"
  make px4_sitl gz_x500_dual_cam_aruco_dual_ids
  ```
- If the world uses custom models, Gazebo will load them automatically from the
  `models` directory populated above.

Re-run the copy steps whenever the assets in this repository change.
