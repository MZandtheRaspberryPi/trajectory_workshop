# Developer Notes

These are notes on what setup and configuration the real robots need to support the assignments. Students don't have to read this.

To setup a computer to run on the real robot from scratch you will need to clone the repo, build the docker image, connect the computer to the robot, setup your computer's network interface, run the docker image, and run the code inside the docker image.

Complicating matters, the GO2W does not publish odometry automatically. Hence to use that robot odometry has to be published. We implement FASTLIO2 for that, and the launch file has an argument to toggle this on. For the GO2W we hence run on the onboard computer in the "sensor backpack" we have attached.

## running from scratch

### initial setup

Setup steps, do this once per computer:
```Bash
cd $HOME
git clone https://github.com/MZandtheRaspberryPi/trajectory_workshop --recurse-submodules
cd trajectory_workshop
git submodule update --init --recursive
docker build --network host -f docker/Dockerfile -t traj --progress plain .
mkdir $HOME/docker_mount
chmod -R 777 $HOME/docker_mount
```

### running the lab

Run steps, change the `REPO_PATH` variable to be the path to where you downloaded the git repo. Additionally, For Go2 - wire connection. Set up a ethernet interface using Ubuntu settings GUI like "unitree Go2W", IP address need to be set under "192.168.123.xxx" with net mask "255.255.255.0". For Go2W w/ Backpack - do ssh connection to the jetson. You may need to plug the jetson into a monitor to connect it to the lab WIFI and figure out what IP address it is for the WIFI Interface. From there just check the other network interfaces are on different subnets (192.168.2.50, the 2 is the subnet) and the WIFI interface is on the 192.168.1.xxx subnet. From here you should be able to SSH into it. We set the en01 interface to static ip 192.168.2.50 and netmask 255.255.255.0, which talks to the livox. We set the enxc84d44298f99 interface to 192.168.123.222 with netmask 255.255.255.0.

```Bash
# export REPO_PATH=/home/data/projects/trajectory_workshop
export REPO_PATH=/home/$USER/trajectory_workshop

docker run -it --rm --network host --volume /home/$USER/docker_mount:/docker_mount --volume $REPO_PATH/dev/ros2_ws/scripts:/home/student/ros_ws/scripts --volume $REPO_PATH/dev/ros2_ws/src/traj_helper:/home/student/ros_ws/src/traj_helper --volume $REPO_PATH/dev/ros2_ws/src/livox_ros_driver2:/home/student/ros_ws/src/livox_ros_driver2 --volume $REPO_PATH/dev/ros2_ws/src/FAST_LIO_ROS2:/home/student/ros_ws/src/FAST_LIO_ROS2 --volume $REPO_PATH/dev/ros2_ws/src/traj_helper_msgs:/home/student/ros_ws/src/traj_helper_msgs traj

cd ros_ws
source install/setup.bash
colcon build --symlink-install --parallel-workers 3 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# DDS setup, put in the name of the interface you are using here
export IP_INTERFACE=eno1  # enp13s0f1 (Michael), eno1 (Hongbo), enxc84d44298f99 (Jetson BackPack)
source scripts/unitree_cyclonedds_setup.sh $IP_INTERFACE 
```

Test if things work. On GO2W you should see topics returned by the topic list, and the read motion state will print zeros, but it will recieve messages (Go2W does not run odometry automatically). GO2 topics should print and odometry should print.
Test if things work:
```Bash
ros2 topic list

ros2 run unitree_ros2_example read_motion_state
ros2 topic echo /utlidar/robot_odom --field pose.pose
ros2 topic echo /lf/sportmodestate --field position
ros2 topic echo /lf/sportmodestate --field imu_state.rpy
```

To bringup with fastlio for odom...
```Bash
ros2 launch traj_helper system.launch.py odom_fastlio:=false  use_open_loop:=true
```

scp student@192.168.1.99:/home/student/docker_mount/traj_data/traj_2026-01-29_09-57-28.png .




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

3. Building the docker file takes forever, it is hanging on something like `#21 49.72 Get:11 http://ports.ubuntu.com/ubuntu-ports jammy/main arm64 iproute2 arm64 5.15.0-1ubuntu2 [1080 kB]`. 

Not sure what the fix is here....

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


## building and installing the python traj_lib library

Within the root folder of the repo...

```
python3 -m pip install build
python3 -m build --wheel
python3 -m pip install dist/traj_lib-1.0.0-py3-none-any.whl
```