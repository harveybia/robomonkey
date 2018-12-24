cmake_minimum_required(VERSION 2.8)

include_directories(
  ${MKYCOM_INCLUDE_DIRS}
)

set(MKYCOM_TEST_SOURCE
  # Library
  mkycom.cpp

  # Test
  mkycom_test.cpp
)

add_executable(mkycom_test ${MKYCOM_TEST_SOURCE})
