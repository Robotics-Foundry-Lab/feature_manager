FROM althack/ros2:humble-dev 

# ** [Optional] Uncomment this section to install additional packages. **
#
# ENV DEBIAN_FRONTEND=noninteractive
# RUN apt-get update \
#    && apt-get -y install --no-install-recommends <your-package-list-here> \
#    #
#    # Clean up
#    && apt-get autoremove -y \
#    && apt-get clean -y \
#    && rm -rf /var/lib/apt/lists/*
# ENV DEBIAN_FRONTEND=dialog

# RUN sudo apt-get install ros-humble-joy
ARG DOCKER_GID=994
RUN groupadd -g ${DOCKER_GID} docker \
    && usermod -aG docker ros
    
# Docker CLI + compose plugin: the feature manager runs `docker compose` against the host daemon
RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates curl \
    && install -m 0755 -d /etc/apt/keyrings \
    && curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) stable" \
        > /etc/apt/sources.list.d/docker.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends docker-ce-cli docker-compose-plugin \
    && rm -rf /var/lib/apt/lists/*

# NavigateToPose action for the goals set in the 3D map of the web UI, and CycloneDDS because
# Navigation runs on it (see RMW_IMPLEMENTATION in docker-compose.yml)
RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-humble-nav2-msgs ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc