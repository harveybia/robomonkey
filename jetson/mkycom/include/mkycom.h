/**
 * @file mkycom.h
 * @author Haowen Shi
 * @date 22 Dec 2018
 * @brief A communication library for RoboMonkey microcontroller.
 * 
 * This header defines high level API for communicating with RoboMonkey
 * microcontroller, including sending commands and receiving feedback from it.
 */

#ifndef _MKYCOM_H_
#define _MKYCOM_H_

#include <sys/types.h>

#include "infantry_info.h"

/**
 * @brief Communication status.
 */
typedef enum {
  COM_OK = 0,   /**< Success */
  COM_FAIL,     /**< Failure */
  COM_TIMEOUT,  /**< Timeout */
} MKYCOM_Status;

int mkycom_init(void);

/*TODO: example functions, not official API. */
ssize_t mkycom_receive(char *buf, size_t len);
ssize_t mkycom_transmit(char *buf, size_t len);

void *read_and_unpack_thread(void *argu);

extern send_pc_t mky_chassis_stats;

#endif /* _MKYCOM_H_ */
