project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++14 -g)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

#set(sources src/main.cpp src/route_frame.cpp src/route_types.cpp src/route_index.cpp src/test_route.cpp)
set(routelib_src src/route/route_frame.h src/route/route_index.h src/route/route_smoother.h src/route/route_types.h src/route/route_frame.cpp src/route/route_index.cpp src/route/route_smoother.cpp src/route/route_types.cpp)
set(trajlib_src src/trajectory/trajectory.cpp src/trajectory/utils.cpp)

include_directories(src/)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 
  include_directories(/usr/local/include)
  include_directories(/usr/local/opt/openssl/include)
  link_directories(/usr/local/lib)
  link_directories(/usr/local/opt/openssl/lib)
  link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_library(routelib ${routelib_src})
add_library(trajlib ${trajlib_src})
add_executable(trajtest src/trajectory/test.cpp)
target_link_libraries(trajtest trajlib)
#add_executable(path_planning ${sources})

#target_link_libraries(path_planning z ssl uv uWS)
#target_link_libraries(trajlib z ssl uv uWS)
