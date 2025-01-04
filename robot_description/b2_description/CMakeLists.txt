cmake_minimum_required(VERSION 3.8)
project(b2_description)

find_package(ament_cmake REQUIRED)

install(
		DIRECTORY urdf launch config assets xml xacro meshes
		DESTINATION share/${PROJECT_NAME}/
)

ament_package()