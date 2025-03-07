# Trajectory Visualization and Capture

This package contains two ROS nodes:

1. **capture_trajectory_service**: Captures the trajectory of a robot by recording its position and orientation over a specified duration and saves it to a CSV file with the help of tf2_ros::TransformListener that captures the pose of base_link wrt to Odom .
2. **vizualise_trajectory_service**: Visualizes the captured trajectory by reading the CSV file and publishing markers to RViz.

These nodes facilitate the recording and visualization of robot

## Usage 
bas
e on the following steps:

### Capturing the Trajectory

1. Launch the ROS master:
    ```sh
    roscore
    ```

2. Start the `capture_trajectory_service` node:
    ```sh
    rosrun your_package_name capture_trajectory_service
    ```

3. Call the service to start capturing the trajectory:
    ```sh
    rosservice call /capture_trajectory "duration: 10.0"
    ```
    Replace `10.0` with the desired duration in seconds.

### Visualizing the Trajectory

1. Start the `vizualise_trajectory_service` node:
    ```sh
    rosrun your_package_name vizualise_trajectory_service
    ```

2. Open RViz:
    ```sh
    rviz
    ```

3. Add a `Marker` display in RViz and set the topic to `/visualization_marker`.

4. Call the service to visualize the trajectory:
    ```sh
    rosservice call /vizualise_trajectory "file_path: '/path/to/your/trajectory.csv'"
    ```
    Replace `'/path/to/your/trajectory.csv'` with the actual path to your CSV file.