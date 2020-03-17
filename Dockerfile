FROM ros:melodic-ros-core

RUN apt-get update && apt-get install -y \
	vim \
	default-jdk \
	wget \
	unzip \
	&& rm -rf /var/lib/apt/lists/

# Set java home
ENV JAVA_HOME=/usr/lib/jvm/default-java

# Install gradle
RUN ["/bin/bash", "-c", "wget https://services.gradle.org/distributions/gradle-6.2.2-bin.zip -P /tmp && \
                            unzip -d /opt/gradle /tmp/gradle-*.zip"]

ENV GRADLE_HOME=/opt/gradle/gradle-6.2.2
ENV PATH=${GRADLE_HOME}/bin:${PATH}

RUN [ "/bin/bash","-c","source /opt/ros/melodic/setup.bash && \
        mkdir -p /jason_ros_ws/src && \
        cd /jason_ros_ws/src && catkin_init_workspace"]

WORKDIR /jason_ros_ws/src
RUN ["/bin/bash", "-c", "git clone https://github.com/jason-lang/jason_ros.git"]

WORKDIR /jason_ros_ws
RUN [ "/bin/bash","-c","source /opt/ros/melodic/setup.bash && \
        rosdep update && rosdep install --from-paths src --ignore-src -r -y && catkin_make"]

WORKDIR /
RUN ["/bin/bash", "-c","git clone https://github.com/Rezenders/jason-active-perception.git"]

COPY uav_ap/ /uav_ap/

RUN ["/bin/bash","-c","cp -r /jason-active-perception/src/java/active_perception/ /uav_ap/src/java/"]
WORKDIR /uav_ap
RUN ["/bin/bash","-c","gradle build"]

COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
