This directory is about how to use darknet ROS with our project.

1-First create catkin_ws folder and inside of this folder create src folder and then use this order.

```
cd catkin_ws/src
git clone --recursive git@github.com:leggedrobotics/darknet_ros.git
cd ../
catkin build darknet_ros -DCMAKE_BUILD_TYPE=Release
```

after download darknet we need to set to parameter in darknet_ros.launch, ros.yaml and yolov3.yaml file change in darknet_ros folder.

Then, we need to add our drone dataset in our darknet in 

https://github.com/chuanenlin/drone-net

Download this repository and copy yolo-drone.cfg to darknet_ros/yolo_network_config/cfg
and copy yolo-drone.weights to darknet_ros/yolo_network_config/weights.

Finally open a terminal in catkin_ws and configure source in terminal write;
```
source devel/setup.bash
roslaunch darknet_ros darknet_ros.launch
```

