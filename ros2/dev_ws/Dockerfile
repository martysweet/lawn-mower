FROM arm64v8/ros:foxy-ros-base
# FROM osrf/ros:foxy-desktop

# Install deps
RUN apt update && apt install python3-pip python3-rpi.gpio -y
RUN rosdep update # Get package names

# Copy Source
WORKDIR /root/dev_ws/src

# Ensure extra python dependencies are installed # TODO: Is there a way of doing pip installs with rosdep?
COPY src/requirements.txt .
RUN pip3 install -r requirements.txt

# Copy the entrypoint
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh


# Get Deps and Build
# If anything in src/ changes, docker will rebuild from this point
COPY src .
WORKDIR /root/dev_ws
RUN rosdep install -i --from-path src --rosdistro foxy -y
RUN . /opt/ros/foxy/setup.sh && colcon build

# Default Entrypoint
CMD ["bash"]