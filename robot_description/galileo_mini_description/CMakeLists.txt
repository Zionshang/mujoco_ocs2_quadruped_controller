cmake_minimum_required(VERSION 3.8)
project(galileo_mini_description)

find_package(ament_cmake REQUIRED)

install(
		DIRECTORY config launch meshes urdf xacro xml
		DESTINATION share/${PROJECT_NAME}/
)

ament_package()