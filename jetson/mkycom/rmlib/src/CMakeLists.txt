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

# Install to system
set_target_properties(rmlib
	PROPERTIES PUBLIC_HEADER "${RMLIB_INCLUDE_DIRS}/infantry_info.h;${RMLIB_INCLUDE_DIRS}/stm32f4xx_hal.h"
)

install(TARGETS rmlib
	DESTINATION lib
	PUBLIC_HEADER DESTINATION include
)
