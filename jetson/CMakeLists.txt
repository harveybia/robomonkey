cmake_minimum_required (VERSION 3.5)
project (MkyJetson)

if (APPLE)
  MESSAGE("Building on Apple platform, using std=c++11")
  set(CMAKE_CXX_EXTENSIONS OFF) # Use -std=c++11 instead of -std=gnu++11
endif()
set(CMAKE_CXX_STANDARD 11)


set(MKYCOM_INCLUDE_DIRS "${MKYCOM_SOURCE_DIR}/include")
set(RMLIB_INCLUDE_DIRS "${MKYCOM_SOURCE_DIR}/rmlib/include")

include_directories(
  ${RMLIB_INCLUDE_DIRS}
)

add_subdirectory(mkycom)
