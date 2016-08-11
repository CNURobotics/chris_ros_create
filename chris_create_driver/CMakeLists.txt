cmake_minimum_required(VERSION 2.8.3)
project(chris_create_driver)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(PROGRAMS src/chris_create_driver/create_driver.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
