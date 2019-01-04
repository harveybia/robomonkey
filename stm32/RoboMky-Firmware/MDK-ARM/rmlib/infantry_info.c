/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
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
/** @file infantry_info.c
 *  @version 1.0
 *  @date Oct 2017
 *
 *  @brief the information from computer
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#include "infantry_info.h"
#include "protocol.h"
#include "info_interactive.h"
#include "communicate.h"
#include "comm_task.h"
#include "string.h"
#include "sys_config.h"
#include "cmsis_os.h"
#include "chassis_task.h"
/* data send */
send_pc_t    pc_send_mesg;
/* data receive */
receive_pc_t pc_recv_mesg;

//for debug
int pc_seq            = 0;
int once_lost_num     = 0;
int lost_pack_percent = 0;

int pack_num_cnt   = 0;
int lost_num_sum_t = 0;
int pack_lost      = 0;


int gim_mode_js;
int pitch_angle_js;
int yaw_angle_js;
int dis_mm_js;

uint32_t pc_yaw_time;

int8_t recv_pc_glb  = 0;
int8_t glb_err_exit = 0;


int pc_state;

/**
  * @brief    get computer control message
  */
extern TaskHandle_t judge_unpack_task_t;
void pc_data_handler(uint8_t *p_frame)
{
  frame_header_t *p_header = (frame_header_t*)p_frame;
  memcpy(p_header, p_frame, HEADER_LEN);

  uint16_t data_length = p_header->data_length;
  uint16_t cmd_id      = *(uint16_t *)(p_frame + HEADER_LEN);
  uint8_t *data_addr   = p_frame + HEADER_LEN + CMD_LEN;

  //lost pack monitor
  pack_num_cnt++;
  
  if (pack_num_cnt <= 100)
  {
    once_lost_num = p_header->seq - pc_seq - 1;
    
    if (once_lost_num < 0)
    {
      once_lost_num += 256;
    }
    
    lost_num_sum_t += once_lost_num;
  }
  else
  {
    lost_pack_percent = lost_num_sum_t;
    lost_num_sum_t    = 0;
    pack_num_cnt      = 0;
  }
  
  
  if (once_lost_num != 0)
  {
    pack_lost = 1;
  }
  else
  {
    pack_lost = 0;
  }
  
  pc_seq = p_header->seq;
  //end lost pack monitor
  
  
  taskENTER_CRITICAL();
  
  switch (cmd_id)
  {
    case CHASSIS_CTRL_ID:
    {
      memcpy(&pc_recv_mesg.chassis_control_data, data_addr, data_length);
      chassis.ctrl_mode = (chassis_mode_e)pc_recv_mesg.chassis_control_data.ctrl_mode;
			//chassis.ctrl_mode = CHASSIS_STOP;
      
    }
    break;
    
    case ERROR_LEVEL_ID:
    {
      memcpy(&pc_recv_mesg.global_error_level, data_addr, data_length);
      
      pc_state = pc_recv_mesg.global_error_level.err_level;

    }
    break;
    
    case INFANTRY_STRUCT_ID:
      memcpy(&pc_recv_mesg.structure_data, data_addr, data_length);
    break;
		
  }
  
  taskEXIT_CRITICAL();
}







