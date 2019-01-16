cmake_minimum_required (VERSION 3.5)

include_directories(
  ${RMLIB_INCLUDE_DIRS}
)

# RMLIB library source files
set(RMLIB_SOURCE
    communicate.c
    data_fifo.c
    protocol.c
)

add_library(rmlib STATIC ${RMLIB_SOURCE})
