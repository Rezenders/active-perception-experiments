# active-perception-experiments
Experiments for [active perception](https://github.com/Rezenders/jason-active-perception)

# run bare metal

Terminal 1:
```
$ sim_vehicle.py -v ArduCopter --console --map -L UFSC --out mavros:14551
```

For the terminals that runs ros commands you must first source ros
```
$ source /opt/ros/melodic/setup.bash
```
Substitute melodic with your ros version

Terminal 2:
```
$ roscore
```

Terminal 3:
```
$ roslaunch mavros apm.launch fcu_url:="udp://:14551@:14555"
```

For terminal 4 and 5 you must first download [jason_ros](https://github.com/jason-lang/jason_ros), use ```catkin_make``` to build it, and source the workspace.

```
$ mkdir -p ~/jason_ros_ws/src
$ cd ~/jason_ros_ws
$ catkin_init_workspace src
$ catkin_make
$ source devel/setup.bash
```

Terminal 4:
```
$ rosrun hw_bridge hw_bridge.py -a actions_manifest -p perceptions_manifest
```
Use the correct path for actions_manifest and perceptions_manifest

Terminal 5:
```
$ cd uav_ap/
$ gradle
```

# using docker

```
$ sudo docker build -t uav_ap .
```

```
$ sudo docker run -it --rm uav_ap
```

```
$ gradle
```
