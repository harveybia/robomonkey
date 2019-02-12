cmake_minimum_required(VERSION 3.5)
project(MKYCOM)

# mkycom CMakeList

add_subdirectory(rmlib)
include_directories(rmlib/include)

add_subdirectory(src)

