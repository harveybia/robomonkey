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
/** @file detect_task.c
 *  @version 1.1
 *  @date June 2017
 *
 *  @brief detect module offline or online task
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#include "detect_task.h"
#include "cmsis_os.h"
#include "bsp_led.h"
#include "bsp_io.h"
#include "sys_config.h"

UBaseType_t detect_stack_surplus;

/* detect task global parameter */
global_err_t g_err;

/* detect task static parameter */
static offline_dev_t offline_dev[ERR_OFFLINE_LAST + 1];
/**
  * @brief     initialize detector error_list
  * @usage     used before detect loop in detect_task() function
  */
void global_err_detector_init(void)
{
  g_err.err_now = NULL;
  g_err.list[BOTTOM_DEVICE].dev    = NULL;
  g_err.list[BOTTOM_DEVICE].enable = 0;

  /* initialize device error type and offline timeout value */
  for (uint8_t i = CHASSIS_GYRO_OFFLINE; i < ERROR_LIST_LENGTH; i++)
  {
    if (i <= ERR_OFFLINE_LAST)
    {
      offline_dev[i].set_timeout = 500; //ms
      offline_dev[i].last_time   = 0;
      offline_dev[i].delta_time  = 0;
      
      g_err.list[i].dev  = &offline_dev[i];
      g_err.list[i].type = DEV_OFFLINE;
    }
    else if (i <= CHASSIS_CONFIG_ERR)
    {
      g_err.list[i].dev  = NULL;
      g_err.list[i].type = SYS_CONFIG_ERR;
    }
    
    offline_dev[PC_SYS_OFFLINE].set_timeout = 2000; //ms
  }
    
  /* initialize device error detect priority and enable byte */

  g_err.list[CHASSIS_GYRO_OFFLINE].err_exist = 0;
  g_err.list[CHASSIS_GYRO_OFFLINE].pri       = 7;
  g_err.list[CHASSIS_GYRO_OFFLINE].enable    = 1;

  for (int i = 0; i < 4; i++)
  {
    g_err.list[CHASSIS_M1_OFFLINE + i].err_exist = 0;
    g_err.list[CHASSIS_M1_OFFLINE + i].pri       = 2 + i; //2,3,4,5
    g_err.list[CHASSIS_M1_OFFLINE + i].enable    = 1;
  }
  
  g_err.list[PC_SYS_OFFLINE].err_exist    = 0;
  g_err.list[PC_SYS_OFFLINE].pri          = 1;
  g_err.list[PC_SYS_OFFLINE].enable       = 1;
}

/**
  * @brief     record the detected module return time to judge offline
  * @param     err_id: module id
  * @retval    None
  * @usage     used in CAN/usart.. rx interrupt callback
  */
void err_detector_hook(int err_id)
{
  if (g_err.list[err_id].enable)
      g_err.list[err_id].dev->last_time = HAL_GetTick();
}

void detector_param_init(void)
{
  global_err_detector_init();

  g_err.beep_tune = DEFAULT_TUNE;
  g_err.beep_ctrl = 0;
  
  LED_INIT;
}


extern int8_t recv_pc_glb;
extern int8_t glb_err_exit;
int pc_glb_cnt;

/**
  * @brief     according to the interval time
  * @param     err_id: module id
  * @retval    None
  * @usage     used in CAN/usart.. rx interrupt callback
  */
uint32_t detect_time_last;
int detect_time_ms;
void detect_task(void const *argu)
{
  uint32_t detect_wake_time = osKernelSysTick();
  while(1)
  {
    detect_time_ms = HAL_GetTick() - detect_time_last;
    detect_time_last = HAL_GetTick();
    
    /* module offline detect */
    module_offline_detect();

    
    if (glb_err_exit == 1)
    {
      if (pc_glb_cnt++ > 50)
        pc_glb_cnt = 0;
      
      if (pc_glb_cnt < 15)
        g_err.beep_ctrl = g_err.beep_tune/2;
      else
        g_err.beep_ctrl = 0;
    }
    else
    {
      if (g_err.err_now != NULL)
      {
        //LED_G_OFF;
        module_offline_callback();
      }
      else
      {
        g_err.beep_ctrl = 0;
        //LED_G_ON;
      }
    }
    
    beep_ctrl(g_err.beep_tune, g_err.beep_ctrl);
    
    detect_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
    
    osDelayUntil(&detect_wake_time, DETECT_TASK_PERIOD);
  }
}

static void module_offline_detect(void)
{
  int max_priority = 0;
  int err_cnt      = 0;
  for (uint8_t id = ERR_OFFLINE_FIRST; id <= ERR_OFFLINE_LAST; id++)
  {
    g_err.list[id].dev->delta_time = HAL_GetTick() - g_err.list[id].dev->last_time;
    
    if (g_err.list[id].enable 
        && (g_err.list[id].dev->delta_time > g_err.list[id].dev->set_timeout))
    {
      g_err.list[id].err_exist = 1; //this module is offline
      err_cnt++;
      if (g_err.list[id].pri > max_priority)
      {
        max_priority     = g_err.list[id].pri;
        g_err.err_now    = &(g_err.list[id]);
        g_err.err_now_id = (err_id_e)id;
      }
    }
    else
    {
      g_err.list[id].err_exist = 0;
    }
  }

  if (!err_cnt)
  {
    g_err.err_now    = NULL;
    g_err.err_now_id = BOTTOM_DEVICE;
  }
}

static void module_offline_callback(void)
{
  g_err.err_count++;
  if (g_err.err_count > 50)
    g_err.err_count = 0;

  switch (g_err.err_now_id)
  {
    case CHASSIS_GYRO_OFFLINE:
    {
      if (g_err.err_count == 1
          || g_err.err_count == 7
          || g_err.err_count == 13)
      {
        LED_R_ON;
        g_err.beep_ctrl = g_err.beep_tune/2;
      }
      else
      {
        LED_R_OFF;
        g_err.beep_ctrl = 0;
      }
    }break;

    case CHASSIS_M1_OFFLINE:
    case CHASSIS_M2_OFFLINE:
    case CHASSIS_M3_OFFLINE:
    case CHASSIS_M4_OFFLINE:
    {
      if (g_err.err_count == 1
          || g_err.err_count == 7
          || g_err.err_count == 13
          || g_err.err_count == 19
          || g_err.err_count == 25)
      {
        LED_R_ON;
        g_err.beep_ctrl = g_err.beep_tune/2;
      }
      else
      {
        LED_R_OFF;
        g_err.beep_ctrl = 0;
      }
    }break;
    
    default:
    {
      LED_R_ON;
      g_err.beep_ctrl = 0;
    }break;
  }
}
/*
static void module_offline_callback(void)
{
  g_err.err_count++;
  if (g_err.err_count > 50)
    g_err.err_count = 0;

  switch (g_err.err_now_id)
  {
    case REMOTE_CTRL_OFFLINE:
    {
      if (g_err.err_count == 1)
      {
        LED_R_ON;
        g_err.beep_ctrl = g_err.beep_tune/2;
      }
      else
      {
        LED_R_OFF;
        g_err.beep_ctrl = 0;
      }
    }break;
    
    case GIMBAL_PIT_OFFLINE:
    case GIMBAL_YAW_OFFLINE:
    {
      if (g_err.err_count == 1
          || g_err.err_count == 7)
      {
        LED_R_ON;
        g_err.beep_ctrl = g_err.beep_tune/2;
      }
      else
      {
        LED_R_OFF;
        g_err.beep_ctrl = 0;
      }
    }break;
    
    case TRIGGER_MOTO_OFFLINE:
    {
      if (g_err.err_count == 1
          || g_err.err_count == 7
          || g_err.err_count == 13)
      {
        LED_R_ON;
        g_err.beep_ctrl = g_err.beep_tune/2;
      }
      else
      {
        LED_R_OFF;
        g_err.beep_ctrl = 0;
      }
    }break;
    
    case CHASSIS_M1_OFFLINE:
    case CHASSIS_M2_OFFLINE:
    case CHASSIS_M3_OFFLINE:
    case CHASSIS_M4_OFFLINE:
    {
      if (g_err.err_count == 1
          || g_err.err_count == 7
          || g_err.err_count == 13
          || g_err.err_count == 19)
      {
        LED_R_ON;
        g_err.beep_ctrl = g_err.beep_tune/2;
      }
      else
      {
        LED_R_OFF;
        g_err.beep_ctrl = 0;
      }
    }break;
    
    case CHASSIS_GYRO_OFFLINE:
    {
      if (g_err.err_count == 1
          || g_err.err_count == 7
          || g_err.err_count == 13
          || g_err.err_count == 19
          || g_err.err_count == 25)
      {
        LED_R_ON;
        g_err.beep_ctrl = g_err.beep_tune/2;
      }
      else
      {
        LED_R_OFF;
        g_err.beep_ctrl = 0;
      }
    }break;
    
    default:
    {
      LED_R_ON;
      g_err.beep_ctrl = 0;
    }break;
  }
}
*/

