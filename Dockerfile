# 1. Obraz bazowy: ROS 2 Humble Desktop
FROM osrf/ros:humble-desktop

# 2. Wyłączenie interaktywnych pytań instalatora
ENV DEBIAN_FRONTEND=noninteractive

# 3. Instalacja narzędzi systemowych, bibliotek Gazebo oraz sterowników ROS 2
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# 4. Utworzenie przestrzeni roboczej wewnątrz kontenera
WORKDIR /ros2_ws

# 5. Kopiowanie całego kodu źródłowego (wraz z rover_control i rover_description)
COPY src/ /ros2_ws/src/

# 6. Kompilacja przestrzeni roboczej Colcon
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install"

# 7. Automatyczne ładowanie środowiska ROS 2 oraz projektu przy starcie terminala
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc