
## Useful Commands
```
Within src/
ros2 pkg create --build-type ament_python py_pubsub
ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy example_interfaces


Within dev_ws/
rosdep install -i --from-path src --rosdistro foxy -y

colcon build --packages-select py_pubsub

. install/setup.bash
ros2 run py_pubsub talker


$ sudo apt-get install qemu qemu-user qemu-user-static

docker buildx build --platform=linux/arm64,linux/amd64 -t ros2 .


docker run ghcr.io/martysweet/lawn-mower:dev -- ros2 topic pub /i2c_drive_cmd robomower_interfaces/I2CDrive "{pwr_left: 0, pwr_right: 0}" --once
```

## Docker Cheat Sheet
`--privileged` Access to /dev

`--network host` Allows cross host development (useful for monitoring tools on dev machine)

`-it` Interactive, allows CTRL+C to be captured

```bash
# Drive Control
export IMAGE=ghcr.io/martysweet/lawn-mower:dev
docker pull $IMAGE && docker run -it --privileged --network host $IMAGE ros2 run drive_i2c drive_control
```

```bash
# Odometry
export IMAGE=ghcr.io/martysweet/lawn-mower:dev
docker pull $IMAGE && docker run -it --privileged --network host $IMAGE ros2 run drive_i2c odometry_feedback
```

You need to keep the service discovery open, for example:
```bash
ros2 topic pub /i2c_drive_cmd robomower_interfaces/I2CDrive "{pwr_left: 0, pwr_right: 0}"
ros2 topic echo /i2c_drive_cmd
ros2 topic echo /i2c_drive_cmd  
```

Then start sending --once commands
```bash
ros2 topic pub /i2c_drive_cmd robomower_interfaces/I2CDrive "{pwr_left: 255, pwr_right: 255}"  --once
ros2 topic pub /i2c_drive_cmd robomower_interfaces/I2CDrive "{pwr_left: 0, pwr_right: 0}"  --once
```

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

# DS4 Controller

On the RPI:
```bash
sudo bluetoothctl
# Hold PS + Share until blinking
devices
pair <MacADDR>
# Authorise yes
```

```bash
# RPI (Bluetooth to controller messages)
export IMAGE=ghcr.io/martysweet/lawn-mower:dev
docker pull $IMAGE && docker run -it --privileged --network host $IMAGE -- ros2 run joy_linux joy_linux_node
 

# Local
ros2 topic echo /joy

# RPI (translates controller commands to Twist)
export IMAGE=ghcr.io/martysweet/lawn-mower:dev
docker pull $IMAGE && docker run -it --privileged --network host $IMAGE -- ros2 run teleop_twist_joy teleop_node --ros-args -p require_enable_button:="false" -p axis_linear.x:="1" -p axis_angular.yaw:="0" -p scale_linear.x:="3.0" -p scale_angular.yaw:="3.0"

export IMAGE=ghcr.io/martysweet/lawn-mower:dev
docker pull $IMAGE && docker run -it --privileged --network host $IMAGE -- ros2 run demo_nodes_cpp listener


export IMAGE=ghcr.io/martysweet/lawn-mower:dev
docker pull $IMAGE && docker run -it --privileged --network host $IMAGE -- ros2 run demo_nodes_cpp talker
```
