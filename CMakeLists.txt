cmake_minimum_required(VERSION 2.8.3)
project(splashscreen)

find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(PROGRAMS
  scripts/splashscreen.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)