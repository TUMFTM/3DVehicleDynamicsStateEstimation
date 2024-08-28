# ROS Distro
ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO

# Set working directory
RUN mkdir /ws
WORKDIR /ws

RUN sudo apt update && sudo apt install -y -q --no-install-recommends

# Copy repo into the container
COPY . /ws/src

# Source ROS and build package
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Set entrypoint by sourcing overlay workspace
RUN echo '#!/bin/bash\nset -e\n\n# setup ros environment\nsource "/opt/ros/$ROS_DISTRO/setup.bash"\n. \
    /ws/install/setup.bash\nexec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]