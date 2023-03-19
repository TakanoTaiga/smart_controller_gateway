# Smart Controller Gateway
## Use Local ROS
### Prerequstments
- Ubuntu 20
- ROS2 Humble or Foxy
- git
### Build and Install
```
mkdir -p ~/ros_ws/src
cd ros_ws/src
git clone https://github.com/hakoroboken/smart_controller_gateway.git
git clone https://github.com/hakoroboken/AdaOS_Message.git
cd ~ros_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Launch
```
source ~/ros_ws/install/setup.sh
ros2 launch smart_controller_gateway demo.py
```

## Parameter

|ROS Parameter |Type |Default |Description |
|---|---|---|---|
|`network_interface` |`String` |`eno1` | Network interface for communication
|`is_publish_twist` |`Bool` |`false` | Select whether to publish Twist type.
|`is_publish_rc_rover` |`Bool` |`false` | WIP
|`button_a_label` |`String` |`butoon a` | Label for Button A
|`button_b_label` |`String` |`butoon b` |Label for Button B
|`slider_label` |`String` |`slider` | Label for slider
