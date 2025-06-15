cmake_minimum_required(VERSION 3.0)
project(vcsm)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")

add_library(vcsm SHARED user-vcsm.c)

target_link_libraries(vcsm vcos)
target_include_directories(vcsm PRIVATE . /usr/local/include /opt/vc/include)