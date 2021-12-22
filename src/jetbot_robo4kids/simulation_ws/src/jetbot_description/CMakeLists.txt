cmake_minimum_required(VERSION 2.8.3)

project(jetbot_description)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch meshes urdf rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
)
