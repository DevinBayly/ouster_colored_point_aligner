## Setup instructions
Ensure that you have several files provided in the folder one `-pc.bag` and one `-transforms.bag` then use the name param to correctly process them
```
catkin_make
source devel/setup.bash
rosrun lidar_basic process_lidar _param=" <name>"
```
Note the space within the quotation is important because if the name is a number it won't be parsed correctly anyways. 