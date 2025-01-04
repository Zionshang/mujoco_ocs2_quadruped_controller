cmake_minimum_required(VERSION 3.8)
project(bqr3_description)

find_package(ament_cmake REQUIRED)

install(
		DIRECTORY meshes urdf launch config xml
		DESTINATION share/${PROJECT_NAME}/
)

ament_package()