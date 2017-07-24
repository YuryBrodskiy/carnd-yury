project(MPC)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++11 -O3)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")



include_directories(/usr/local/include)
link_directories(/usr/local/lib)
include_directories(src/Eigen-3.3)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


set(sources src/MPC.cpp src/main.cpp src/ekf.cpp)
add_executable(mpc_native ${sources})
add_executable(mpc_ekf ${sources})
set_target_properties (mpc_ekf PROPERTIES COMPILE_DEFINITIONS "EKF_BASED")

target_link_libraries(mpc_native ipopt z ssl uv uWS)
target_link_libraries(mpc_ekf ipopt z ssl uv uWS)

