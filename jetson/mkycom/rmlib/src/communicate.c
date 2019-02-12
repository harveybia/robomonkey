/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *  Copyright (C) 2019 Haowen Shi
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/** @file communicate.h
 *  @version 1.0
 *  @date 16 Jan 2019
 *
 *  @brief the communication interface of main control with computer
 *
 *  Modified to work for PC host instead of RM board.
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *  @copyright 2019 Haowen Shi
 *
 */

#include "communicate.h"
#include "infantry_info.h"
#include "data_fifo.h"
#include "protocol.h"
#include "string.h"

uint8_t* protocol_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf)
{
  //memset(tx_buf, 0, 100);
  static uint8_t seq;

  uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;
  frame_header_t *p_header = (frame_header_t*)tx_buf;

  p_header->sof          = sof;
  p_header->data_length  = len;


  if (sof == UP_REG_ID)
  {
    if (seq++ >= 255)
      seq = 0;

    p_header->seq = seq;
  }
  else
  {
    p_header->seq = 0;
  }


  memcpy(&tx_buf[HEADER_LEN], (uint8_t*)&cmd_id, CMD_LEN);
  append_crc8_check_sum(tx_buf, HEADER_LEN);
  memcpy(&tx_buf[HEADER_LEN + CMD_LEN], p_data, len);
  append_crc16_check_sum(tx_buf, frame_length);

  return tx_buf;
}


uint32_t send_packed_fifo_data(fifo_s_t *pfifo, uint8_t sof)
{
#if (JUDGE_FIFO_BUFLEN > COMPUTER_FIFO_BUFLEN)
  uint8_t  tx_buf[JUDGE_FIFO_BUFLEN];
#else
  uint8_t  tx_buf[COMPUTER_FIFO_BUFLEN];
#endif

  uint32_t fifo_count = fifo_used_count(pfifo);

  if (fifo_count)
  {
    fifo_s_gets(pfifo, tx_buf, fifo_count);

    if (sof == UP_REG_ID) {
      return 0;
      // TODO: implement upload
      //write_uart_blocking(&COMPUTER_HUART, tx_buf, fifo_count);
    }
    else
      return 0;
  }

  return fifo_count;
}

