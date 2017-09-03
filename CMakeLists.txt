project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/constants.cpp src/cost_functions.cpp src/environment.cpp src/main.cpp src/jmt.cpp src/perspective.cpp src/planner.cpp src/polynomial.cpp src/ptg.cpp src/statistics.cpp src/telemetry.cpp src/trajectory.cpp src/util.cpp src/vehicle.cpp)
set(jmt_sources src/jmt.cpp src/jmt_main.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


add_executable(path_planning ${sources})
add_executable(jmt_solver ${jmt_sources})

target_link_libraries(path_planning z ssl uv uWS)
target_link_libraries(jmt_solver z)
