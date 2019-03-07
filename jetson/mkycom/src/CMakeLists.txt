cmake_minimum_required(VERSION 3.5)

set(MKYCOM_SOURCE
  mkycom.cpp
)

set(MKYCOM_TEST_SOURCE
  mkycom_test.cpp
)

include_directories(
  "../include"
  "../../rmlib/include"
)

# MKYCOM Communication Library
add_library(mkycom ${MKYCOM_SOURCE})
target_link_libraries(mkycom PUBLIC rmlib)

# MKYCOM Usage Example
add_executable(mkycom_test ${MKYCOM_TEST_SOURCE})
target_link_libraries(mkycom_test
  PUBLIC rmlib
  PUBLIC mkycom
  pthread
)
