/**
 * @file mkycom.cpp
 * @author Haowen Shi
 * @date 22 Dec 2018
 * @brief A communication library for RoboMonkey microcontroller.
 */

#include "mkycom.h"

#include <assert.h>
#include <fcntl.h>
#include <unistd.h>

#ifndef MKYCOM_TTY
#define MKYCOM_TTY "/dev/ttyACM0" // default tty
#endif

int tty_fd = 0;


static ssize_t _com_receive(char *buf, size_t len);

static ssize_t _com_transmit(char *buf, size_t len);



ssize_t _com_receive(char *buf, size_t len) {
  if (tty_fd == 0) return COM_FAIL;
  ssize_t recv_len = read(tty_fd, buf, len);
  return recv_len;
}

ssize_t _com_transmit(char *buf, size_t len) {
  if (tty_fd == 0) return COM_FAIL;
  ssize_t tx_len = write(tty_fd, buf, len);
  return tx_len;
}

int mkycom_init(void) {
  if (tty_fd == 0) {
    tty_fd = open(MKYCOM_TTY, O_RDWR);
    if (tty_fd == 0) {
      // init fail, cannot open virtual com port
      return -1;
    }
  }
  assert(tty_fd != 0);
  return 0;
}

ssize_t mkycom_receive(char *buf, size_t len) {
  return _com_receive(buf, len);
}

ssize_t mkycom_transmit(char *buf, size_t len) {
  return _com_transmit(buf, len);
}
