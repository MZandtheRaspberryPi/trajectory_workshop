# Developer Notes

These are notes on what setup and configuration the real robots need to support the assignments. Students don't have to read this.

## building and installing the python traj_lib library

Within the root folder of the repo...

```
python3 -m pip install build
python3 -m build --wheel
python3 -m pip install dist/traj_lib-1.0.0-py3-none-any.whl
```

## Docker stuff
```Bash
git submodule update --init --recursive
docker build -f docker/Dockerfile -t traj --progress plain .
mkdir $HOME/docker_mount
chmod -R 777 $HOME/docker_mount
export REPO_PATH=/home/data/projects/trajectory_workshop
# export REPO_PATH=/home/$USER/trajectory_workshop
docker run -it --rm --network host --volume /home/$USER/docker_mount:/docker_mount --volume $REPO_PATH/dev/ros2_ws/scripts:/home/student/ros_ws/scripts --volume $REPO_PATH/dev/ros2_ws/src/traj_helper:/home/student/ros_ws/src/traj_helper --volume $REPO_PATH/dev/ros2_ws/src/livox_ros_driver2:/home/student/ros_ws/src/livox_ros_driver2 --volume $REPO_PATH/dev/ros2_ws/src/FAST_LIO_ROS2:/home/student/ros_ws/src/FAST_LIO_ROS2 traj

cd ros_ws
source install/setup.bash
colcon build --symlink-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Network Connection 
# For Go2W - wire connection. Set up a ethernet interface using Ubuntu settings GUI like "unitree Go2W", IP address need to be set under "192.168.123.xxx" with net mask "255.255.255.0"
# For Go2W w/ Backpack - do ssh connection.  

# DDS setup
source scripts/unitree_cyclonedds_vars.sh   # modify the name of "NetworkInterface" to yours. 
```

```Bash
# Test
ros2 topic echo /utlidar/robot_odom --field pose.pose
ros2 topic echo /lf/sportmodestate --field position
ros2 topic echo /lf/sportmodestate --field imu_state.rpy


# Run
ros2 run unitree_examples_py cmd_vel_to_unitree 
ros2 run traj_helper trajectory_follower
```

To bringup with fastlio for odom...
```Bash
ros2 launch traj_helper system.launch.py --ros_args -p odom_fastlio:="true" 
```

## Real Robot Network Configuration

Connect with network cable on the laptop
Configure IP & Test Ping

```Bash
cd {path-to-this-repo}
cd dev/ros2_ws/

sudo apt update
sudo apt install -y ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl

source /opt/ros/humble/setup.bash # source ROS2 environment
colcon build # Compile all packages in the workspace
```

**Connect with network cable on the laptop**

1. Connect the laptop with Ethernet Cable, run `ifconfig` to find the ethernet connection IP.
  ```Bash
  export IFACE="eno1"                 # modify here
  export PC_IP="192.168.123.222/24"   # modify here
  ```
2. Manually Set Ip in `./scripts/unitree_net_setup.sh`. And execute it.
```Bash
sudo chmod +x scripts/unitree_net_setup.sh
./scripts/unitree_net_setup.sh
```

3. Source workspace, export DDS and net-interface. 

```Bash
# source ROS2 & ws
# cd /path/to/project/trajectory_workshop/dev/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash  

# Export DDS and Network Interface
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="eno1" priority="default" multicast="default" />
      </Interfaces>
    </General>
  </Domain>
  <DDSI2E>
    <Internal>
      <MaxSampleSize>2MB</MaxSampleSize>
    </Internal>
  </DDSI2E>
</CycloneDDS>'

```
4. Test if things work:
```Bash
ros2 topic list

ros2 run unitree_ros2_example read_motion_state


```


---
## Toubleshooting

1. DDS Buffer Too small

```Bash
./install/unitree_ros2_example/bin/read_motion_state
2026-01-21 15:09:53.605 [RTPS_READER_HISTORY Error] Change payload size of '516' bytes is larger than the history payload size of '239' bytes and cannot be resized. -> Function can_change_be_added_nts

# Solution: Export CycloneDDS configuration with larger buffer
export CYCLONEDDS_URI='<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="eno1" priority="default" multicast="default" />
      </Interfaces>
    </General>
  </Domain>
  <DDSI2E>
    <Internal>
      <MaxSampleSize>2MB</MaxSampleSize>
    </Internal>
  </DDSI2E>
</CycloneDDS>'
```

2. Not consistant message recieved

When trying to echo data via `ros2 topic echo /utlidar/robot_odom --field pose.pose`, the returned message was "piece-by-piece" instead of being "always comming-in". 

- `sudo ethtool eno1 | grep Speed` -> `Speed: 1000Mb/s`
- `sudo apt install nload` -> ![alt text](image.png)

Solve: Reboot the robot and Re-run docker container. It should be working now.

## Notes

1. The official `unitree_ros2` README isn't clear to understand. It supports ROS2 Foxy & Humble, but only states clearly how to do the former. So I need to fork it and add some useful notes for my own (and others) easy-to-use purpose on Humble.
2. Add my forked submodule
   git submodule add -b master git@github.com:JLCucumber/unitree_ros2.git dev/ros2_ws/src/unitree_ros2
   git submodule update --init --recursive

3. Little Demos
   1. `rviz2` visualize lidar point cloud
      1. Add PointCloud2 (topic:utlidar/could)
      2. Change `worldframe` to `utlidar_lidar`
      3. For better visualization, change `decay_time` to 5  