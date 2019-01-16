cmake_minimum_required (VERSION 3.5)
project (MkyJetson)

set (CMAKE_CXX_STANDARD 11)

set(MKYCOM_INCLUDE_DIRS "${MKYCOM_SOURCE_DIR}/include")
set(RMLIB_INCLUDE_DIRS "${MKYCOM_SOURCE_DIR}/rmlib/include")

include_directories(
  ${RMLIB_INCLUDE_DIRS}
)

add_subdirectory(mkycom)
