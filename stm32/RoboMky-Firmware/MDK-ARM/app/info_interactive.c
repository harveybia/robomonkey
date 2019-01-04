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
/** @file info_interactive.c
 *  @version 1.0
 *  @date Oct 2017
 *
 *  @brief get hardware peripheral information
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#include "info_interactive.h"
#include "comm_task.h"
#include "info_get_task.h"
#include "detect_task.h"
#include "chassis_task.h"
#include "bsp_can.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "infantry_info.h"
#include "protocol.h"
#include "string.h"
#include "math.h"
#include "sys_config.h"
#include "cmsis_os.h"
#include "stdlib.h"

void get_chassis_info(void)
{
  /* get chassis wheel speed */
  for (uint8_t i = 0; i < 4; i++)
  {
    chassis.wheel_spd_fdb[i] = moto_chassis[i].speed_rpm;
  }

  /* get remote and keyboard chassis control information */
	// Nothing here: no remote controll supported
  
  /* get chassis structure configuration parameter */
  get_structure_param();
}

static void get_structure_param(void)
{
  if ((pc_recv_mesg.structure_data.chassis_config == CUSTOM_CONFIG)
   && (pc_recv_mesg.structure_data.wheel_perimeter != 0)
   && (pc_recv_mesg.structure_data.wheel_base      != 0)
   && (pc_recv_mesg.structure_data.wheel_track     != 0))
  {
    glb_struct.chassis_config  = CUSTOM_CONFIG;
    glb_struct.wheel_perimeter = pc_recv_mesg.structure_data.wheel_perimeter;
    glb_struct.wheel_base      = pc_recv_mesg.structure_data.wheel_base;
    glb_struct.wheel_track     = pc_recv_mesg.structure_data.wheel_track;
  }
  else
  {
    glb_struct.chassis_config  = DEFAULT_CONFIG;
    glb_struct.wheel_perimeter = PERIMETER;
    glb_struct.wheel_base      = WHEELBASE;
    glb_struct.wheel_track     = WHEELTRACK;
  }
}

extern int speed_debug;

int32_t position_x_mm, position_y_mm, angle_deg;
int16_t v_x_mm, v_y_mm, palstance_deg;
void chassis_position_measure(void)
{
  static float  rotate_ratio_fr;
  static float  rotate_ratio_fl;
  static float  rotate_ratio_bl;
  static float  rotate_ratio_br;
  static float  rpm_ratio;
  static float  ecd_ratio;
  static double chassis_angle;
  static double last_d_x,last_d_y,last_d_w,d_x,d_y,d_w,diff_d_x,diff_d_y,diff_d_w;
  static double position_x,position_y,angle_w;
  static double v_x,v_y,w_v;
  
  rotate_ratio_fr = ((glb_struct.wheel_base+glb_struct.wheel_track)/2.0f - \
                      chassis.rotate_x_offset + chassis.rotate_y_offset);
  rotate_ratio_fl = ((glb_struct.wheel_base+glb_struct.wheel_track)/2.0f - \
                      chassis.rotate_x_offset - chassis.rotate_y_offset);
  rotate_ratio_bl = ((glb_struct.wheel_base+glb_struct.wheel_track)/2.0f + \
                      chassis.rotate_x_offset - chassis.rotate_y_offset);
  rotate_ratio_br = ((glb_struct.wheel_base+glb_struct.wheel_track)/2.0f + \
                      chassis.rotate_x_offset + chassis.rotate_y_offset);
  rpm_ratio = glb_struct.wheel_perimeter*CHASSIS_DECELE_RATIO/(4*60.0f);
  ecd_ratio = glb_struct.wheel_perimeter*CHASSIS_DECELE_RATIO/(4*8192.0f);
  
  last_d_x = d_x;
  last_d_y = d_y;
  last_d_w = d_w;
  d_x = ecd_ratio * ( - moto_chassis[0].total_ecd + moto_chassis[1].total_ecd \
                      + moto_chassis[2].total_ecd - moto_chassis[3].total_ecd);
  d_y = ecd_ratio * ( - moto_chassis[0].total_ecd - moto_chassis[1].total_ecd \
                      + moto_chassis[2].total_ecd + moto_chassis[3].total_ecd);
  d_w = ecd_ratio * ( - moto_chassis[0].total_ecd/rotate_ratio_fr \
                      - moto_chassis[1].total_ecd/rotate_ratio_fl \
                      - moto_chassis[2].total_ecd/rotate_ratio_bl \
                      - moto_chassis[3].total_ecd/rotate_ratio_br);

  diff_d_x = d_x - last_d_x;
  diff_d_y = d_y - last_d_y;
  diff_d_w = d_w - last_d_w;
  
  /* use chassis gyro angle data */
  chassis_angle = chassis.gyro_angle/RADIAN_COEF;
 
  position_x += diff_d_x*cos(chassis_angle) - diff_d_y*sin(chassis_angle);
  position_y += diff_d_x*sin(chassis_angle) + diff_d_y*cos(chassis_angle);
  
  angle_w += diff_d_w;

  position_x_mm = (int32_t)(position_x); //mm
  position_y_mm = (int32_t)(position_y); //mm
  angle_deg     = (int32_t)(angle_w*RADIAN_COEF);//degree

  v_x = rpm_ratio * ( - moto_chassis[0].speed_rpm + moto_chassis[1].speed_rpm \
                      + moto_chassis[2].speed_rpm - moto_chassis[3].speed_rpm);
  v_y = rpm_ratio * ( - moto_chassis[0].speed_rpm - moto_chassis[1].speed_rpm \
                      + moto_chassis[2].speed_rpm + moto_chassis[3].speed_rpm);
  w_v = rpm_ratio * ( - moto_chassis[0].speed_rpm/rotate_ratio_fr \
                      - moto_chassis[1].speed_rpm/rotate_ratio_fl \
                      - moto_chassis[2].speed_rpm/rotate_ratio_bl \
                      - moto_chassis[3].speed_rpm/rotate_ratio_br);

  v_x_mm = (int16_t)(v_x);         //mm/s
  v_y_mm = (int16_t)(v_y);         //mm/s
  palstance_deg = (int16_t)(w_v*RADIAN_COEF);//degree/s
}

void get_infantry_info(void)
{
  /* chassis */
  pc_send_mesg.chassis_information.ctrl_mode      = chassis.ctrl_mode;
  pc_send_mesg.chassis_information.gyro_palstance = chassis.gyro_palstance;
  pc_send_mesg.chassis_information.gyro_angle     = chassis.gyro_angle;
  pc_send_mesg.chassis_information.ecd_palstance  = palstance_deg;
  pc_send_mesg.chassis_information.ecd_calc_angle = angle_deg;
  pc_send_mesg.chassis_information.x_spd          = v_x_mm;
  pc_send_mesg.chassis_information.y_spd          = v_y_mm;
  pc_send_mesg.chassis_information.x_position     = position_x_mm; //the absolute x axis position of chassis
  pc_send_mesg.chassis_information.y_position     = position_y_mm; //the absolute y axis position of chassis

  /* infantry error */
  pc_send_mesg.bottom_error_data.err_sta = DEVICE_NORMAL;
  for (uint8_t i = CHASSIS_GYRO_OFFLINE; i < ERROR_LIST_LENGTH; i++)
  {
    if (g_err.list[i].enable)
    {
      if (g_err.list[i].err_exist)
      {
        pc_send_mesg.bottom_error_data.err_sta = ERROR_EXIST;
        pc_send_mesg.bottom_error_data.err[i]  = ERROR_EXIST;
      }
      else
        pc_send_mesg.bottom_error_data.err[i]  = DEVICE_NORMAL;
    }
    else
      pc_send_mesg.bottom_error_data.err[i] = UNKNOWN_STATE;
  }
  
  /* structure config */
  pc_send_mesg.structure_config_data.chassis_config = glb_struct.chassis_config;
}

void get_custom_data_info(void)
{
  //memcpy(&pc_recv_mesg.pc_to_server_data, &student_to_judgesys_data, sizeof(user_to_server_t));
}


void send_chassis_motor_ctrl_message(int16_t chassis_cur[])
{
  send_chassis_cur(chassis_cur[0], chassis_cur[1], 
                   chassis_cur[2], chassis_cur[3]);
}

extern TaskHandle_t pc_unpack_task_t;

/**
  * @brief  Uart send data in non blocking mode
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  * @param  p_data: Pointer to data buffer
  * @param  size: Amount of data to be sent
  */
void write_uart_noblocking(UART_HandleTypeDef *huart, uint8_t *p_data, uint16_t size)
{
  HAL_UART_Transmit_DMA(huart, p_data, size);
}

void uart_write_completed_signal(UART_HandleTypeDef *huart)
{
  if (huart == &COMPUTER_HUART)
  {
    //osSignalSet(unpack_task_t, PC_UART_WRITE_SIGNAL);
  }
}

/**
  * @brief  Uart send data in blocking mode, 1000ms is the timeout value.
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  * @param  p_data: Pointer to data buffer
  * @param  size: Amount of data to be sent
  */
void write_uart_blocking(UART_HandleTypeDef *huart, uint8_t *p_data, uint16_t size)
{
  HAL_UART_Transmit(huart, p_data, size, 100);
}

/**
  * @brief  Receives an amount of data in non blocking mode. 
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @param  p_data: Pointer to data buffer
  * @param  size: Amount of data to be received
  */
void read_uart_noblocking(UART_HandleTypeDef *huart, uint8_t *p_data, uint16_t size)
{
  HAL_UART_Receive_DMA(huart, p_data, size);
}

void uart_read_completed_signal(UART_HandleTypeDef *huart)
{
  if (huart == &COMPUTER_HUART)
  {
    //osSignalSet(unpack_task_t, PC_UART_READ_SIGNAL);
  }
}

/**
  * @brief  Receives an amount of data in blocking mode. 
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @param  p_data: Pointer to data buffer
  * @param  size: Amount of data to be received
  */
extern TaskHandle_t judge_unpack_task_t;
extern TaskHandle_t pc_unpack_task_t;
void read_uart_blocking(UART_HandleTypeDef *huart, uint8_t *p_data, uint16_t size)
{
  HAL_UART_Receive(huart, p_data, size, 1000);
}

void uart_idle_interrupt_signal(UART_HandleTypeDef *huart)
{
  if (huart == &COMPUTER_HUART)
  {
    err_detector_hook(PC_SYS_OFFLINE);
    osSignalSet(pc_unpack_task_t, PC_UART_IDLE_SIGNAL);
  }
}

void uart_dma_full_signal(UART_HandleTypeDef *huart)
{
  if (huart == &COMPUTER_HUART)
  {
    /* remove DMA buffer full interrupt handler */
    //osSignalSet(pc_unpack_task_t, PC_DMA_FULL_SIGNAL);
  }
}

void get_dma_memory_msg(DMA_Stream_TypeDef *dma_stream, uint8_t *mem_id, uint16_t *remain_cnt)
{
  *mem_id     = dma_current_memory_target(dma_stream);
  *remain_cnt = dma_current_data_counter(dma_stream);
}
