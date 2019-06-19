project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11 -g)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(planner_src src/main.cpp)
set(routelib_src src/route/route_frame.h src/route/route_index.h src/route/route_smoother.h src/route/route_types.h src/route/route_frame.cpp src/route/route_index.cpp src/route/route_smoother.cpp src/route/route_types.cpp src/utils.cpp)
set(trajlib_src src/trajectory/trajectory.cpp src/trajectory/utils.cpp)
set(decisionlib_src src/decision/decision.cpp)

include_directories(src/)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 
  include_directories(/usr/local/include)
  include_directories(/usr/local/opt/openssl/include)
  link_directories(/usr/local/lib)
  link_directories(/usr/local/opt/openssl/lib)
  link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_library(routelib ${routelib_src})
add_executable(routetest src/route/route_test.cpp)
target_link_libraries(routetest routelib)

add_library(trajlib ${trajlib_src})
add_executable(trajtest src/trajectory/test.cpp)
target_link_libraries(trajtest trajlib)

add_library(decisionlib ${decisionlib_src})
target_link_libraries(decisionlib trajlib)

add_executable(path_planning ${planner_src})
target_link_libraries(path_planning trajlib)
target_link_libraries(path_planning routelib)
target_link_libraries(path_planning decisionlib)

target_link_libraries(path_planning z ssl uv uWS)
#target_link_libraries(trajlib z ssl uv uWS)
