cmake_minimum_required(VERSION 3.8)
project(rover_description)

# Szukanie głównego systemu budowania ROS 2
find_package(ament_cmake REQUIRED)


install(
  DIRECTORY urdf meshes launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# ament_package() MUSI być na samym końcu pliku!
ament_package()