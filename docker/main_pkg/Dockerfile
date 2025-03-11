################################################################################
# There are places where your actions are needed in this Dockerfile.
# These are marked with the label `ACTIONS_NEEDED`.
# After you are done with those:
#
# Pull the robetarme noetic base image with
# `docker login registry.gitlab.com`
# `docker pull registry.gitlab.com/certh-iti-robotics-lab/robetarme/robetarme/ros-noetic:base`
#
# Then build this Dockerfile with
# `docker compose build`,
#
# and run it with
# `docker compose up -d`.
#
# Then gain access to the container with
# `docker attach CONTAINER_NAME`,
# and `docker exec -it --user robetarme_user CONTAINER_NAME bash`
# ** where CONTAINER_NAME is the one specified in the `docker-compose.yml` file under
# services/.../container_name.
#
# Then simply launch your ROS noetic packages.
################################################################################

FROM registry.gitlab.com/certh-iti-robotics-lab/robetarme/robetarme/ros-noetic:base
ARG USER=robetarme_user

#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
# Do not modify above this line
#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

# ACTIONS_NEEDED
# ------------------------------------------------------------------------------
# Install your dependencies here
# ------------------------------------------------------------------------------
# Add a few essential tools
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    python3-catkin-pkg \
    python3-catkin-tools \
    python3-osrf-pycommon \
    python3-rosdep \
    python3-wstool \
    ros-cmake-modules \
    ros-noetic-catkin \
    ros-noetic-realsense* \
    ros-noetic-pcl-* \
    ros-noetic-visualization-msgs
# ------------------------------------------------------------------------------

# ACTIONS_NEEDED
# ------------------------------------------------------------------------------
# Place your ROS packages into the host's catkin_ws/src directory
# i.e. $PWD/../../catkin_ws/src. This directory will be copied verbatim into
# the image.
#
# CAUTION! Anything else you `COPY` must be specified with
# `COPY --chown=${USER}:${USER} source destination`
# ------------------------------------------------------------------------------
COPY --chown=${USER}:${USER} --chmod=0755 t5.3-rebar-detection/sup/run_docker_rebars_segmentation.sh /home/${USER}/run_docker_rebars_segmentation.sh

#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
# Do not modify below this line
#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

# ------------------------------------------------------------------------------
# COPY the src/ directory of your host's ROS workspace into the container's
# ------------------------------------------------------------------------------
COPY --chown=${USER}:${USER} t5.3-rebar-detection/catkin_ws/src/ /home/${USER}/catkin_ws/src/
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
# catkin build ROS noetic packages
USER ${USER}
RUN cd /home/${USER}/catkin_ws                                              && \
    source /opt/ros/noetic/setup.bash                                       && \
    catkin build                                                            && \
    source /home/${USER}/catkin_ws/devel/setup.bash
USER root
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
# Bind mount/shared directory between container and host;
# robetarme_user is owner
# https://docs.docker.com/storage/bind-mounts/
USER ${USER}
RUN mkdir /home/${USER}/shared
USER root
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
# Entrypoint stuff
# ------------------------------------------------------------------------------
COPY --chown=${USER}:${USER} t5.3-rebar-detection/docker/main_pkg/entrypointd.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
# ------------------------------------------------------------------------------

WORKDIR /home/${USER}
CMD ["/bin/bash"]