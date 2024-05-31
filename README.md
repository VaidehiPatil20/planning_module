# Planning Module

## Overview

This `planning_module` package marks the beginning of the new restructured planner. The planning module contains 4 important nodes: Plan Interface, Plan Manager, Collision Detection, and Plan Optimizer. The Planning Interface interacts with the orchestration piece using the ROS Interface and interacts with all other nodes via Python services. The Plan Manager has access to all the planners under it.

## Install Required Dependencies

```bash
sudo apt-get install ros-noetic-trac-ik
pip install -r planning_module/requirements.txt
```
## Setup and Usage
Step 1: Launch the ROS Interface
Open a terminal and run the following command to start the ROS interface:

```bash
roslaunch planning_module planning_module.launch
```

Step 2: Start the Python Nodes

Open separate terminals for each of the following nodes and run:

Planning Interface:

```bash

python3 ~/catkin_ws/src/planning_module/nodes/planning_interface.py
```
Plan Manager:

```bash

python3 ~/catkin_ws/src/planning_module/nodes/plan_manager.py
```
Linear Planner:

```bash

python3 ~/catkin_ws/src/planning_module/nodes/linear_planner.py
```
Polar Planner:

```bash

python3 ~/catkin_ws/src/planning_module/nodes/polar_planner.py
```
Test Service: To test the setup with a dummy service, run:

```bash

python3 ~/catkin_ws/src/planning_module/nodes/test_planning_client.py
```

Configuration

Ensure that the config.yaml file is in the correct path and properly formatted.
Ensure that the URDF/ parameters are launched so the IK can access them.
Make sure the RWS/EGM services of the robot are on and the robot can listen to joint states.
