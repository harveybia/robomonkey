/**
 * @file mkycom.cpp
 * @author Haowen Shi
 * @date 22 Dec 2018
 * @brief A communication library for RoboMonkey microcontroller.
 */

#include "mkycom.h"
#include "rmlib.hpp"

#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <termios.h>

#ifndef MKYCOM_TTY
#define MKYCOM_TTY "/dev/ttyACM0" // default tty
#endif

int tty_fd = 0;

/* ttyUSB serial device helpers */
static int set_interface_attribs (int fd, int speed, int parity);
static void set_blocking(int fd, int should_block);

static ssize_t _com_receive(char *buf, size_t len);
static ssize_t _com_transmit(char *buf, size_t len);

static void unix_error(const char *msg) {
  printf("%s: %s\n", strerror(errno), msg);
  exit(errno);
}

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
  const char *portname = "/dev/ttyUSB0";

  tty_fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (tty_fd < 0) {
    unix_error("error opening usbtty");
  }

  set_interface_attribs (tty_fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
  set_blocking (tty_fd, 0);                // set no blocking

  //write (fd, "hello!\n", 7);           // send 7 character greeting

  //usleep ((7 + 25) * 100);             // sleep enough to transmit the 7 plus
  //                                     // receive 25:  approx 100 uS per char transmit
  //char buf [100];
  //int n = read (fd, buf, sizeof buf);  // read up to 100 characters if ready to read
  return 0;
}

ssize_t mkycom_receive(char *buf, size_t len) {
  return _com_receive(buf, len);
}

ssize_t mkycom_transmit(char *buf, size_t len) {
  return _com_transmit(buf, len);
}

static int set_interface_attribs (int fd, int speed, int parity) {
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd, &tty) != 0) {
    unix_error("error from tcgetattr");
    return errno;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK;         // disable break processing
  tty.c_lflag = 0;                // no signaling chars, no echo,
                                  // no canonical processing
  tty.c_oflag = 0;                // no remapping, no delays
  tty.c_cc[VMIN]  = 0;            // read doesn't block
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    unix_error("error from tcsetattr");
    return -1;
  }
  return 0;
}

static void set_blocking(int fd, int should_block) {
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd, &tty) != 0) {
    unix_error("error from tggetattr");
    return;
  }

  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    unix_error("error setting term attributes");
  }
}


// ...
// char *portname = "/dev/ttyUSB1"
//  ...
// int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
// if (fd < 0)
// {
//   error_message ("error %d opening %s: %s", errno, portname, strerror (errno));
//   return;
// }

// set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
// set_blocking (fd, 0);                // set no blocking

// write (fd, "hello!\n", 7);           // send 7 character greeting

// usleep ((7 + 25) * 100);             // sleep enough to transmit the 7 plus
//                                      // receive 25:  approx 100 uS per char transmit
// char buf [100];
// int n = read (fd, buf, sizeof buf);  // read up to 100 characters if ready to read

#define UART_BUFF_SIZE (500)
char computer_rx_buf[UART_BUFF_SIZE];

void read_and_unpack_thread(void *argu) {
  uint8_t byte = 0;
  int32_t read_len;
  int32_t buff_read_index;

  uint16_t      data_len;
  unpack_step_e unpack_step;
  int32_t       index;
  uint8_t       protocol_packet[PROTOCAL_FRAME_MAX_SIZE];

  while (1)
  {
    read_len = read(tty_fd, computer_rx_buf, UART_BUFF_SIZE);
    buff_read_index = 0;

    while (read_len--)
    {
      byte = computer_rx_buf[buff_read_index++];

      switch(unpack_step)
      {
        case STEP_HEADER_SOF:
        {
          if(byte == UP_REG_ID)
          {
            unpack_step = STEP_LENGTH_LOW;
            protocol_packet[index++] = byte;
          }
          else
          {
            index = 0;
          }
        }break;

        case STEP_LENGTH_LOW:
        {
          data_len = byte;
          protocol_packet[index++] = byte;
          unpack_step = STEP_LENGTH_HIGH;
        }break;

        case STEP_LENGTH_HIGH:
        {
          data_len |= (byte << 8);
          protocol_packet[index++] = byte;

          if(data_len < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CRC_LEN))
          {
            unpack_step = STEP_FRAME_SEQ;
          }
          else
          {
            unpack_step = STEP_HEADER_SOF;
            index = 0;
          }
        }break;

        case STEP_FRAME_SEQ:
        {
          protocol_packet[index++] = byte;
          unpack_step = STEP_HEADER_CRC8;
        }break;

        case STEP_HEADER_CRC8:
        {
          protocol_packet[index++] = byte;

          if (index == HEADER_LEN)
          {
            if ( verify_crc8_check_sum(protocol_packet, uint16_t(HEADER_LEN)) )
            {
              unpack_step = STEP_DATA_CRC16;
            }
            else
            {
              unpack_step = STEP_HEADER_SOF;
              index = 0;
            }
          }
        }break;

        case STEP_DATA_CRC16:
        {
          if (index < (HEADER_LEN + CMD_LEN + data_len + CRC_LEN))
          {
             protocol_packet[index++] = byte;
          }
          if (index >= (HEADER_LEN + CMD_LEN + data_len + CRC_LEN))
          {
            unpack_step = STEP_HEADER_SOF;
            index = 0;

            if ( verify_crc16_check_sum(protocol_packet, uint32_t(HEADER_LEN + CMD_LEN + data_len + CRC_LEN)) )
            {
              // Upon successfully decoding a packet, handle it.
              //data_handle(protocol_packet);
            }
          }
        }break;

        default:
        {
          unpack_step = STEP_HEADER_SOF;
          index = 0;
        }break;
      }
    }
  }
}
