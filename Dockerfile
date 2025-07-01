FROM osrf/ros:foxy-desktop

SHELL ["/bin/bash", "-c"]

WORKDIR /app

COPY install_ros2_foxy_dependencies.sh /install_deps.sh
COPY ros_entrypoint.sh /ros_entrypoint.sh

COPY src/ ws/src/

# Install dependencies
RUN /install_deps.sh

RUN source /opt/ros/foxy/setup.bash && \
    cd ws && \
    rosdep install -i -r -y --from-paths . && \
    rosinstall deepracer_description; \
    colcon build --symlink-install

# Entrypoint e comando
ENTRYPOINT ["/ros_entrypoint.sh"]