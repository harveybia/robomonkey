cmake_minimum_required (VERSION 3.5)
project (MkyJetson)

if (APPLE)
  MESSAGE("Building on Apple platform, using std=c++11")
  set(CMAKE_CXX_EXTENSIONS OFF) # Use -std=c++11 instead of -std=gnu++11
endif()
set(CMAKE_CXX_STANDARD 11)

set(RMLIB_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/mkycom/rmlib/include")

add_subdirectory(mkycom)
