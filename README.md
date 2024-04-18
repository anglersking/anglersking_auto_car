- 主节点启动所有传感器以及底盘链接

``` bash
roslaunch anglersking_auto_car_start anglersking_auto_car_start.launch
```

```bash
roslaunch turtlebot_teleop keyboard_teleop.launch
```

```bash
rostopic echo /odom
```