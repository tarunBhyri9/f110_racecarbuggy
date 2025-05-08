## Issues Faced & Fixes

### 1. robot_state_publisher crash due to SDF
- *Cause:* robot_state_publisher only works with URDF, but the project originally used an SDF (f110_car.sdf).
- *Fix:* 
  - Commented out the robot_state_publisher node in the launch file.
  - Let *Gazebo handle the SDF model directly*, avoiding the need for URDF parsing.


### 2. model.config XML Parsing Error
- *Cause:*  Missing </model> closing tag in the model.config file.
- *Fix:* Manually edited the file and added missing closing tags to resolve PARSING errors.


### 3. Incorrect File Referencing in model.config and .sdf
- *Cause:*  
  While spawning models like yellow_cone or blue_cone, Gazebo uses a model.config file to locate the .sdf. Errors occurred due to:
  - Improper folder structure (e.g., .sdf file not placed alongside model.config)
  - Incorrect URI usage inside the world file (e.g., circle.sdf) where .sdf file names were referenced directly instead of folder names.
- *Fix:*
  - Make sure each model (e.g., yellow_cone/) contains *both*:
    - model.config
    - model.sdf (or named appropriately)
  - Ensure *the folder name and file structure match*, like this:
    ~/.gazebo/models/yellow_cone/
        ├── model.config
        └── model.sdf
  - Inside the *world file* (circle.sdf), use the *model folder name*, not the .sdf file name directly:
    xml
    <include>
      <uri>model://yellow_cone</uri>  <!-- NOT model://yellow_cone/model.sdf -->
    </include>


### 4. No cmd_vel Output – SLAM Toolbox Not Publishing Pose
 *Cause:*  
  The final motion topic cmd_vel (published by ackermann_to_twist) was inactive. On tracing the dependency chain, it was discovered that:
  - ackermann_to_twist subscribes to the drive topic.
  - The drive node requires both pose and target_point to compute motion.
  - The pose topic should be published by slam_toolbox, based on incoming scan data.
  - While scan was being correctly published, **slam_toolbox failed to convert scan to pose**.
  Therefore, **the issue originated in the slam_toolbox node**, which acted as a silent failure point in the data pipeline.

 *Fix:*
- Verified that scan was active via ros2 topic echo /scan.
- Confirmed that slam_toolbox was running but not publishing pose.
- Restarted the slam_toolbox node and ensured it subscribed correctly to scan.
- Once pose began publishing, the downstream nodes (drive, ackermann_to_twist) resumed normal behavior.

 *Debugging Insight:*
- The root issue wasn’t at cmd_vel or drive, but **deep upstream at slam_toolbox**.


### 5. Missing TF Link Between base_link and lidar_link – TF Tree Incomplete

- *Cause:*  
  While debugging Issue 4 (SLAM Toolbox not publishing pose), it was found that the *TF tree was broken*. Specifically:
  - lidar_link (connected to CPU_lidar) was not linked to the main TF chain (odom → base_link → camera_link).
  - As a result, slam_toolbox could not associate scan data with the robot's pose frame, preventing it from publishing pose.

- *TF Visualization Insight:*
  When running:
  ```bash
  ros2 run tf2_tools view_frames

 *Fix:*
  The missing transform between base_link and lidar_link was added explicitly. Two valid methods were used:

  *Method 1: Static Transform Publisher (Quick Fix)*  
  Added a static transform between base_link and lidar_link using:
  ```bash
  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link lidar_link



