/*
 * File      : motor_control.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2012, RT-Thread Development Team
 *
 *  
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-12-25     lane      the first version
 * ...
 */

#include "motor_control.h"
#include "IO_config.h"

t_Motor_Control g_Motor_Control_DATA;

void Motor_Control_timer_cb(void)
{
    if(g_Motor_Control_DATA.read_sn_time_cnt)
    {
        g_Motor_Control_DATA.read_sn_time_cnt--;
		if(!g_Motor_Control_DATA.read_sn_time_cnt)
		{
			g_Motor_Control_DATA.read_sn_time_cnt=read_sn_time_Max;
//			set_master_Motor_Control_cmd_flag(MASTER_Motor_Controller_Info);
		}
    }

}

