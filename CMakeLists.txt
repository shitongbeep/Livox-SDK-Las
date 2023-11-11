cmake_minimum_required(VERSION 3.0)

project(las_file)

find_package(Boost COMPONENTS serialization REQUIRED)
find_package(libLAS REQUIRED)

set(livox_sdk_static /usr/local/lib/liblivox_sdk_static.a)

list(APPEND
	${PROJECT_NAME}_src
	las_file.cc
	las_file_handler.cc
	device_handler.cc)
add_library(${PROJECT_NAME}_lib ${${PROJECT_NAME}_src})

target_link_libraries(${PROJECT_NAME}_lib
	PRIVATE
	${livox_sdk_static}
	${libLAS_LIBRARIES}
	)

add_executable(${PROJECT_NAME} main.cc)
target_link_libraries(${PROJECT_NAME}
	${PROJECT_NAME}_lib
	${livox_sdk_static}
	${libLAS_LIBRARIES}
	)
