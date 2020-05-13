FROM ros:melodic-ros-core

RUN apt-get update && apt-get install -y \
	vim \
	default-jdk \
	wget \
	unzip \
	ros-melodic-mavros \
	&& rm -rf /var/lib/apt/lists/

# Set java home
ENV JAVA_HOME=/usr/lib/jvm/default-java

RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN chmod a+x install_geographiclib_datasets.sh
RUN ./install_geographiclib_datasets.sh

# Install gradle
RUN ["/bin/bash", "-c", "wget https://services.gradle.org/distributions/gradle-6.2.2-bin.zip -P /tmp && \
                            unzip -d /opt/gradle /tmp/gradle-*.zip"]

ENV GRADLE_HOME=/opt/gradle/gradle-6.2.2
ENV PATH=${GRADLE_HOME}/bin:${PATH}

RUN [ "/bin/bash","-c","source /opt/ros/melodic/setup.bash && \
        mkdir -p /jason_ros_ws/src && \
        cd /jason_ros_ws/src && catkin_init_workspace"]

WORKDIR /jason_ros_ws/src
RUN ["/bin/bash", "-c", "git clone https://github.com/jason-lang/jason_ros.git "]

RUN mkdir active-perception-experiments
# COPY * active-perception-experiments/*
COPY config/ active-perception-experiments/config/
COPY launch/ active-perception-experiments/launch/
COPY src/ active-perception-experiments/src/
COPY package.xml active-perception-experiments/
COPY CMakeLists.txt active-perception-experiments/
COPY entrypoint.sh active-perception-experiments/

WORKDIR /jason_ros_ws
RUN [ "/bin/bash","-c","source /opt/ros/melodic/setup.bash && \
        apt-get update && rosdep update && rosdep install --from-paths src --ignore-src -r -y \
        && catkin_make && rm -rf /var/lib/apt/lists/"]


# COPY actions_manifest /manifests/
# COPY perceptions_manifest /manifests/
#
# COPY uav_ap/ /uav_ap/

# RUN ["/bin/bash","-c","cp -r /jason-active-perception/src/java/active_perception/ /uav_ap/src/java/"]
# WORKDIR /uav_ap

# COPY entrypoint.sh /
ENTRYPOINT ["/jason_ros_ws/src/active-perception-experiments/entrypoint.sh"]
CMD ["bash"]
