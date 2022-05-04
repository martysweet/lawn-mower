


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
```