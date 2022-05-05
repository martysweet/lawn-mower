FROM arm64v8/ros:foxy-ros-base

# Install deps
RUN apt update && apt install python3-pip -y

# Copy Source
WORKDIR /root/dev_ws/src
COPY src .

# Get Deps and Build
WORKDIR /root/dev_ws
RUN rosdep update && rosdep install -i --from-path src --rosdistro foxy -y
RUN . /opt/ros/foxy/setup.sh && colcon build

# Ensure extra python dependencies are installed # TODO: Is there a way of doing pip installs with rosdep?
RUN pip3 install -r src/requirements.txt

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh

# Default Entrypoint
CMD ["bash"]