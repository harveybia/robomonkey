cmake_minimum_required(VERSION 3.5)
project(MKYCOM)

set(MKYCOM_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/include")

set(MKYCOM_SOURCE
  src/mkycom.cpp
)

set(MKYCOM_TEST_SOURCE
  src/mkycom_test.cpp
)

include_directories(
  "${MKYCOM_INCLUDE_DIRS}"
  "${RMLIB_INCLUDE_DIRS}"
)

# MKYCOM Communication Library
add_library(mkycom STATIC ${MKYCOM_SOURCE})
target_link_libraries(mkycom PUBLIC rmlib)

# MKYCOM Usage Example
add_executable(mkycom_test ${MKYCOM_TEST_SOURCE})
target_link_libraries(mkycom_test
  PUBLIC rmlib
  PUBLIC mkycom
  pthread
)

# Install to system
set_target_properties(mkycom PROPERTIES PUBLIC_HEADER "${MKYCOM_INCLUDE_DIRS}/mkycom.h")
install(TARGETS mkycom
	DESTINATION lib
	PUBLIC_HEADER DESTINATION include
)

# mkycom CMakeList
add_subdirectory(rmlib)
include_directories(rmlib/include)