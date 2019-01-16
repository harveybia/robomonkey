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

#ifndef __COMMUNICATE_H__
#define __COMMUNICATE_H__

#include "data_fifo.h"
#include "protocol.h"

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef enum
{
  UART_IDLE_IT     = 0,
  UART_DMA_HALF_IT = 1,
  UART_DMA_FULL_IT = 2,
} uart_it_type_e;

typedef struct
{
  fifo_s_t       *data_fifo;
  frame_header_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[PROTOCAL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

uint8_t* protocol_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf);

void unpack_fifo_data(unpack_data_t *p_obj, uint8_t sof);

uint32_t send_packed_fifo_data(fifo_s_t *pfifo, uint8_t sof);

#endif
