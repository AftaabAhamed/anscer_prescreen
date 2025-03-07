# Trajectory Visualization and Capture

This package contains two ROS nodes:

1. **capture_trajectory_service**: Captures the trajectory of a robot by recording its position and orientation over a specified duration and saves it to a CSV file with the help of tf2_ros::TransformListener that captures the pose of base_link wrt to Odom .
2. **vizualise_trajectory_service**: Visualizes the captured trajectory by reading the CSV file and publishing markers to RViz.

These nodes facilitate the recording and visualization of robot

## Usage 

### Starting the Simulation

1. Start the `AR 100`simulation packages
   ```sh
   roslaunch start_anscer start_anscer.launch
   ```
2. Start the navigation packages
   ```sh
   roslaunch anscer_navigation anscer_navigation.launch map_name:="map"
   ```
3. Add a `Marker_array` display in RViz and set the topic to `/visualization_marker`.

### Capturing the Trajectory

1. Start the `capture_trajectory_service` node:
    ```sh
    rosrun traj_viz capture_service
    ```

2. Call the service to start capturing the trajectory:
    ```sh
    rosservice call /capture_trajectory "{"filename": "trajectory" , "duration" : "30"}"
    ```
    Replace `30` with the desired duration in seconds (NOTE: Enter a integer).
    Change `trajectory` to desired filename

### Visualizing the Trajectory

1. Start the `vizualise_trajectory_service` node:
    ```sh
    rosrun traj_viz viz_service
    ```

2. Call the visualisation service:
    ```sh
    rosservice call /visualize_trajectory "filename: 'trajectory'"
    ```
    Replace trajectory with name of file to be visualised


