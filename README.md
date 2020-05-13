# active-perception-experiments
Experiments for [active perception](https://github.com/Rezenders/jason-active-perception)

# compile

```
$ mkdir -p ~/jason_ros_ws/src
$ cd ~/jason_ros_ws
$ catkin_init_workspace src
```

Then, you must first download [jason_ros](https://github.com/jason-lang/jason_ros) and [this repo](https://github.com/Rezenders/active-perception-experiments) , use ```catkin_make``` to build it, and source the workspace.

```
$ cd ~/jason_ros_ws/src
$ git clone https://github.com/jason-lang/jason_ros.git
$ git clone https://github.com/Rezenders/active-perception-experiments.git
$ cd ~/jason_ros_ws
$ catkin_make
$ source devel/setup.bash
```

# run using roslaunch

Terminal 1:
```
$ sim_vehicle.py -v ArduCopter --console --map -L UFSC
```

Terminal 2:
```
$ roslaunch active-perception-experiments uav_ap.launch
```

# run one by one

Terminal 1:
```
$ sim_vehicle.py -v ArduCopter --console --map -L UFSC
```

Terminal 2:
```
$ roscore
```

Terminal 3:
```
$ roslaunch mavros apm.launch fcu_url:="udp://:14551@:14555"
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
$ sudo docker network create ros_net
```

```
$ sudo docker build -t ap_experiment .
```

```
$ xhost +local:root # for the lazy and reckless
```

```
$ sudo docker run -it --rm --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --name ardupilot --net ros_net rezenders/ardupilot-ubuntu sim_vehicle.py -v ArduCopter --console --map -L UFSC --out mavros:14551
```

```
$ sudo docker run -it --rm --net ros_net  --name mavros --env ROS_HOSTNAME=mavros ap_experiment roslaunch active-perception-experiments uav_ap.launch fcu_url:="udp://:14551@ardupilot:14555"
```

```
$ gradle
```
