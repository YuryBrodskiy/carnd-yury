project(PARTICLE_FILTER)

#cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../CarND-Kidnapped-Vehicle-Project 
cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/build)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/particle_filter.cpp src/main.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(particle_filter ${sources})


target_link_libraries(particle_filter z ssl uv uWS)

