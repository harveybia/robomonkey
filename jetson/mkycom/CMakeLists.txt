cmake_minimum_required(VERSION 3.5)
project(MKYCOM)

# mkycom CMakeList
set(MKYCOM_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/include")

set(MKYCOM_SOURCE
  "${CMAKE_CURRENT_LIST_DIR}/src/mkycom.cpp"
)

set(MKYCOM_TEST_SOURCE
  "${CMAKE_CURRENT_LIST_DIR}/src/mkycom_test.cpp"
)

include_directories(
  "${MKYCOM_INCLUDE_DIRS}"
  "${RMLIB_INCLUDE_DIRS}"
)

# MKYCOM Communication Library
add_library(mkycom STATIC ${MKYCOM_SOURCE})
target_link_libraries(mkycom PUBLIC rmlib)

add_subdirectory(rmlib)
include_directories(rmlib/include)

# Install to system
set_target_properties(mkycom
	PROPERTIES PUBLIC_HEADER "${MKYCOM_INCLUDE_DIRS}/mkycom.h; ${RMLIB_INCLUDE_DIRS}/*.h"
)

install(TARGETS mkycom
	DESTINATION lib
	PUBLIC_HEADER DESTINATION include
)

