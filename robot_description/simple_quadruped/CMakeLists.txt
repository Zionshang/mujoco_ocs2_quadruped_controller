cmake_minimum_required(VERSION 3.8)
project(simple_quadruped_description)

find_package(ament_cmake REQUIRED)

install(
		DIRECTORY urdf launch config
		DESTINATION share/${PROJECT_NAME}/
)

ament_package()