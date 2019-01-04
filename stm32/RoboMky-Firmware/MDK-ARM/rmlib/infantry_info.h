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
/** @file infantry_info.h
 *  @version 1.0
 *  @date Oct 2017
 *
 *  @brief the information from computer
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#ifndef __INFANTRY_INFO_H__
#define __INFANTRY_INFO_H__

#include "stm32f4xx_hal.h"

#define COMPUTER_FIFO_BUFLEN 500


/** 
  * @brief  infantry robot data command id
  */
typedef enum
{
  CHASSIS_DATA_ID     = 0x0010,
  SHOOT_TASK_DATA_ID  = 0x0012,
  INFANTRY_ERR_ID     = 0x0013,
  CONFIG_RESPONSE_ID  = 0x0014,
  CALI_RESPONSE_ID    = 0x0015,
  REMOTE_CTRL_INFO_ID = 0x0016,
  BOTTOM_VERSION_ID   = 0x0017,
  
  CHASSIS_CTRL_ID     = 0x00A0,
  SHOOT_CTRL_ID       = 0x00A2,
  ERROR_LEVEL_ID      = 0x00A3,
  INFANTRY_STRUCT_ID  = 0x00A4,
} infantry_data_id_e;

typedef enum
{
  BOTTOM_DEVICE        = 0,
  CHASSIS_GYRO_OFFLINE = 2,
  CHASSIS_M1_OFFLINE   = 3,
  CHASSIS_M2_OFFLINE   = 4,
  CHASSIS_M3_OFFLINE   = 5,
  CHASSIS_M4_OFFLINE   = 6,
  REMOTE_CTRL_OFFLINE  = 7,
  JUDGE_SYS_OFFLINE    = 8,
  PC_SYS_OFFLINE       = 9,
  TRIGGER_MOTO_OFFLINE = 12,
  BULLET_JAM           = 13,
  CHASSIS_CONFIG_ERR   = 14,
  ERROR_LIST_LENGTH    = 16,
} err_id_e;

typedef enum
{
  DEVICE_NORMAL = 0,
  ERROR_EXIST   = 1,
  UNKNOWN_STATE = 2,
} bottom_err_e;

typedef enum
{
  GLOBAL_NORMAL        = 0,
  SOFTWARE_WARNING     = 1,
  SOFTWARE_ERROR       = 2,
  SOFTWARE_FATAL_ERROR = 3,
  CHASSIS_ERROR        = 5,
  HARAWARE_ERROR       = 6,
} err_level_e;

typedef enum
{
  NO_CONFIG      = 0,
  DEFAULT_CONFIG = 1,
  CUSTOM_CONFIG  = 3,
} struct_config_e;

/********** the information send to computer ***********/

/** 
  * @brief  chassis information
  */
typedef __packed struct
{
  uint8_t ctrl_mode;      /* chassis control mode */
  float   gyro_palstance; /* chassis palstance(degree/s) from gyroscope */
  float   gyro_angle;     /* chassis angle(degree) relative to ground from gyroscope */
  float   ecd_palstance;  /* chassis palstance(degree/s) from chassis motor encoder calculated */
  float   ecd_calc_angle; /* chassis angle(degree) relative to ground from chassis motor encoder calculated */
  int16_t x_spd;        /* chassis x-axis move speed(mm/s) from chassis motor encoder calculated */
  int16_t y_spd;        /* chassis y-axis move speed(mm/s) from chassis motor encoder calculated */
  int32_t x_position;     /* chassis x-axis position(mm) relative to the starting point */
  int32_t y_position;     /* chassis y-axis position(mm) relative to the starting point */
} chassis_info_t;

/** 
  * @brief  infantry error information
  */
typedef __packed struct
{
  bottom_err_e err_sta;                 /* bottom error state */
  bottom_err_e err[ERROR_LIST_LENGTH];  /* device error list */
} infantry_err_t;

/** 
  * @brief  infantry structure config response
  */
typedef __packed struct
{
  struct_config_e chassis_config;
} config_response_t;

/** 
  * @brief  bottom software version information
  */
typedef __packed struct
{
  uint8_t num[4];
} version_info_t;

/********** the information from computer **********/

typedef __packed struct
{
  int16_t x_offset;   /* offset(mm) relative to the x-axis of the chassis center */
  int16_t y_offset;   /* offset(mm) relative to the y-axis of the chassis center */
  float   w_spd;      /* rotation speed(degree/s) of chassis */
} chassis_rotate_t;

/** 
  * @brief  chassis control information
  */
typedef __packed struct
{
  uint8_t          ctrl_mode; /* chassis control mode */
  int16_t          x_spd;     /* x-axis move speed(mm/s) of chassis */
  int16_t          y_spd;     /* y-axis move speed(mm/s) of chassis */
  chassis_rotate_t w_info;    /* rotation control of chassis */
} chassis_ctrl_t;

/** 
  * @brief  robot system error level
  */
typedef __packed struct
{
  err_level_e err_level; /* the error level is included in err_level_e enumeration */
} global_err_level_t;

/** 
  * @brief  infantry structure configuration information
  */
typedef __packed struct
{
  struct_config_e  chassis_config;  /* chassis structure config state */
  uint16_t         wheel_perimeter; /* the perimeter(mm) of wheel */
  uint16_t         wheel_track;     /* wheel track distance(mm) */
  uint16_t         wheel_base;      /* wheelbase distance(mm) */
} infantry_structure_t;

/********* variables **********/
/** 
  * @brief  the data structure send to pc
  */
typedef struct
{
  /* data send */
  chassis_info_t    chassis_information;
  infantry_err_t    bottom_error_data;
  config_response_t structure_config_data;
  version_info_t    version_info_data;
} send_pc_t;
/** 
  * @brief  the data structure receive from pc
  */
typedef struct
{
  /* data receive */
  chassis_ctrl_t       chassis_control_data;
  global_err_level_t   global_error_level;
  infantry_structure_t structure_data;
} receive_pc_t;

/* data send */
extern send_pc_t    pc_send_mesg;
/* data receive */
extern receive_pc_t pc_recv_mesg;

void pc_data_handler(uint8_t *p_frame);

#endif
