# Autonomous Systems 2025 - ROS 2 Architecture

This workspace (`ros2_ws`) contains the core software components and simulation bridge used for the autonomous control, perception, and simulation of a multirotor Unmanned Aerial Vehicle (UAV).

## System Architecture Overview

The system architecture revolves around a Unity-based physics and rendering simulation bridged to a set of ROS 2 nodes handling control, state estimation spoofing, and perception. Communication between the nodes uses standard ROS 2 messaging paradigms alongside custom UAV messages.

### Packages

<!-- #### 1. `simulation`
The bridge and entry point connecting the ROS 2 software stack with the Unity physics engine.
- **`Simulation.x86_64`**: The precompiled Unity environment executable. 
- **`unity_ros` (C++)**: Acts as the interface between Unity and ROS 2 over local TCP/IP sockets (using `libsocket`). It processes raw bytes from Unity corresponding to simulated IMU, pose, and stereo/depth cameras, unpacking them into standard ROS 2 `sensor_msgs`, `nav_msgs`, and `geometry_msgs`.
- **`state_estimate_corruptor_node` (C++)**: To closely mimic a real-world scenario where state estimation is imperfect, this node subscribes to the pristine `/true_pose` and `/true_twist` topics from Unity. It systematically injects white noise, sporadic jump errors, and random walk drift, publishing the resultant noisy odometry to `/current_state_est` and issuing the `world` $\rightarrow$ `body` TF.
- **`w_to_unity` (C++)**: Relays motor/actuator commands from the ROS 2 controller back to the Unity simulation.
- **Launch scripts**: `simulation.launch.py` brings up the entire system, calling `unity_ros.launch.py`, initializing the aforementioned nodes, spinning up static TF broadcasters for the sensor suite (`sim_true_body`, `sim_rgb_camera`, `sim_depth_camera`), and initializing the controller.

#### 2. `controller_pkg`
Handles the low-level flight control and stabilization.
- **`controller_node` (C++)**: A geometric tracking controller implementing a control loop over `mav_msgs::msg::Actuators`. It subscribes to the UAV's current state (`/current_state_est` via remapping) along with a target trajectory (`trajectory_msgs::msg::MultiDOFJointTrajectory`), computes the necessary wrench and rotor speeds to align the error, and publishes the respective motor commands. Configuration and controller gains are tuned via `config/controller_params.yaml`.

#### 3. `perception_pkg`
Processes raw sensor data into actionable environmental formats.
- **`depth_to_pointcloud_node` (Python)**: Subscribes to the raw `/realsense/depth/image` and camera info. It employs a fully vectorized approach using OpenCV and Numpy to unproject the 2D depth image into a 3D coordinate space mapping. Applying rigid scaling and distance filtering, it generates an unstructured `sensor_msgs/PointCloud2` mapped to exactly align with the drone's sensory frame (`Quadrotor/DepthCamera`), making it highly optimized for real-time robotic perception.

#### 4. `mission_pkg` & `planning_pkg`
- Currently, these act as foundational skeletons for future development. `mission_pkg` provides a Python-based template structure, whilst `planning_pkg` establishes a C++ build environment pre-linked against necessary standard messaging types (`octomap_msgs`, `nav_msgs`, `trajectory_msgs`). They are reserved for downstream high-level behavioral logic and obstacle-avoidance trajectory generation.

#### 5. `mav_msgs` & `mav_planning_msgs`
- Custom ROS 2 message definitions specifically curated for multirotor platforms, originally stemming from the ETH ASL (Autonomous Systems Lab) MAV ecosystem. They encompass core definitions like motor `Actuators` alongside generic multi-degree-of-freedom trajectory directives.

---
## Data Flow Summary
1. **Unity $\rightarrow$ ROS 2**: The `Simulation` executable sends perfect state and camera data via TCP to `unity_ros`, converting it to ROS 2 topics.
2. **State Corruption**: The `/true_pose` is intercepted by `state_estimate_corruptor_node` and degraded before being exposed to the controller as `/current_state_est`.
3. **Control Output**: `controller_node` calculates required rotor velocities based on the drift-impacted state estimate and pushes these `Actuators` commands to `w_to_unity`.
4. **ROS 2 $\rightarrow$ Unity**: `w_to_unity` pushes these actuator states back via TCP to the simulation core to apply physical torques.
5. **Perception**: Concurrently, `depth_to_pointcloud_node` consumes simulated RealSense outputs to stream dense 3D point clouds (`/points`) to be consumed by the (to be implemented) planning modules. -->

### useage of perception_pkg 
#### 1. `cd\ros2_ws` build `colcon build`
#### 2. source the setup `source install/setup.bash`
#### 3. launch simulation `ros2 launch simulation simulation.launch.py`
#### 4. Run the depth to pointcloud node ` ros2 run perception_pkg depth_to_pointcloud_node `