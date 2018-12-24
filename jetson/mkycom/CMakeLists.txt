cmake_minimum_required(VERSION 2.8)
project(MKYCOM)

set(MKYCOM_INCLUDE_DIRS "${MKYCOM_SOURCE_DIR}/include")

include_directories(
  ${MKYCOM_INCLUDE_DIRS}
)

add_subdirectory(src)

