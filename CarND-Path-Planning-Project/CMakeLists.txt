project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/utils.cpp src/Road.cpp src/Road.h src/EgoVehicle.cpp src/EgoVehicle.h src/Option.cpp src/Option.h)

#SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/build)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${sources} src/main.cpp )

target_link_libraries(path_planning z ssl uv uWS)

