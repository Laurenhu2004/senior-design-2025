# senior-design
This repository contains resources and scripts for working with the LIMO Pro Cobot using ROS 1 Noetic. Follow the instructions below to set up and execute the project workflows.


## Prerequisites
- LIMO Pro Cobot
- ROS 1 Noetic installed on your system
- Python 3.9

### Installation & Setup

- [ ] Create virtual environment
- [ ] Run `brew install portaudio` or similar package
- [ ] Run `pip install -r requirements.txt`
- [ ] Download your Google Speech-To-Text API as a JSON file

1. Clone the repository:
   ```bash
   git clone <repository_url>
   cd <repository_name>
   ```
- Create virtual environment
- Run `brew install portaudio` or similar package
- Run `pip install -r requirements.txt`
- 
1. Ensure that the `ros_scripts` folder in this repository is accessible in your ROS workspace. Adjust the `ROS_PACKAGE_PATH` if needed.

## Moving from Depth found at given XY-coordinate Execution Steps
Run the following commands in the terminal in the specified sequence:

1. Launch the Astra camera:
   ```bash
   roslaunch astra_camera dabai_u3.launch
   ```

2. Start the LIMO base with odometry publishing disabled:
   ```bash
   roslaunch limo_bringup limo_start.launch pub_odom_tf:=false
   ```

3. Launch the gmapping module for SLAM:
   ```bash
   roslaunch limo_bringup limo_gmapping.launch
   ```

4. Initialize the move base for navigation:
   ```bash
   roslaunch limo_bringup limo_move_base.launch
   ```

5. Run the RRT exploration module:
   ```bash
   roslaunch rrt_exploration simple.launch
   ```

6. Execute the custom SLAM and movement script:
   ```bash
   rosrun "script_folder" slam_depth_move.py
   ```

7. Run the depth publisher script:
   ```bash
   rosrun "script_folder" depth_publisher.py
   ```

## Notes
- Replace `"script_folder"` with the actual path or package name containing your custom scripts.
- Ensure all required ROS packages are installed before running the commands.

- In full flow, initializing the models take about 9 seconds.

## Acknowledgments
- AgileX Robotics for the LIMO Pro Cobot
- ROS Community for the support and tools used in this project