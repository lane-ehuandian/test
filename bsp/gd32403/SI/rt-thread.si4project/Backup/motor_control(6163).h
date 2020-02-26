/*
 * File      : motor_control.h
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
#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#ifdef __cplusplus
extern "C"{
#endif

#include<stdint.h>
#include "gd32f403.h"
#include "master_rs485_list.h"
#define MOTOR_CONTROL_ENABLE ;

#define MOTOR_CONTROL_RS485_ADDR 0x83//控制器地址

#define read_sn_time_max 1000*60//*2//电机控制器定时器2分钟,间隔2分钟询问电机控制器设备信息
#define read_status_time_max 2000 //读取电机控制器状态，2秒

//enum{
////读电机控制器设备信息
//MOTOR_CONTROL_CMD_INFO = 0x10,
////读电机控制器实时状态
//MOTOR_CONTROL_CMD_STATUS = 0x11,
////设置电机控制器
//MOTOR_CONTROL_CMD_CONTROL1 = 0x12,
////转发仪表
//MOTOR_CONTROL_CMD_CONTROL2 = 0x13,
////设置电机控制器SN
//MOTOR_CONTROL_CMD_SETSN = 0x14,
//};

typedef struct
{
	uint16_t ctrl;//控制指令
	uint16_t speed_ratio;//限速比例
	uint16_t ODO;//累计里程
	uint16_t uv;//欠压值
	uint16_t ov;//过压值
	uint16_t oc;//过流值
	uint16_t speed_limt;//限速值
	uint16_t input_power;//限输入功率
	uint16_t recoil_v;//限反冲电压
	uint16_t recoil_c;//限反冲电流	
}t_Motor_Control_CFG_PARA;
typedef struct
{
	uint8_t month;//月
	uint8_t day;//日
	uint8_t hour;//时
	uint8_t minute;//分
	uint8_t second;//秒
	uint8_t Temp;//温度
	uint8_t RSSI;//信号强度
	uint8_t state;//状态信息
	uint8_t soc;//电量百分比
	uint8_t weather;//天气
}t_Motor_Control_Meter_PARA;
typedef struct
{
	uint8_t sn[14];
}t_Motor_Control_SN_PARA;
typedef struct
{
	uint16_t vendor_info;//厂商信息
	uint16_t ID;//设备序列号
	uint16_t PW;//协议版本
	uint16_t HW;//硬件版本号
	uint16_t baud;//波特率
	uint8_t  sn[14];	
}t_Motor_Control_INFO_PARA;//设备信息
typedef struct
{
	uint16_t state[4];//状态信息
	uint16_t voltage;//电压值
	uint16_t voltage_per;//电压百分比
	uint16_t current;//电流值
	uint16_t speed;//转速
	uint16_t ctrl;//控制指令
	uint16_t speed_Ratio;//限速比例
	uint16_t COD;//当前里程
	uint16_t speed_Max;//最高转速
	uint16_t uv;//欠压值
	uint16_t ov;//过压值
	uint16_t oc;//过流值
	uint16_t speed_Limt;//限速值
	uint16_t input_power;//限输入功率
	uint16_t recoil_Volt;//限反冲电压
	uint16_t recoil_Curr;//限反冲电流		
}t_Motor_Control_State_PARA;//只读数据
typedef struct
{
	uint32_t read_status_time_cnt;//电机控制器定时器,间隔5分钟询问状态设备信息
	uint32_t read_sn_time_cnt;//电机控制器定时器5分钟,间隔5分钟询问设备信息
	
	t_Motor_Control_INFO_PARA *inf;//设备信息	
	t_Motor_Control_State_PARA *sta;//状态
	t_Motor_Control_CFG_PARA *cfg;//控制指令
	t_Motor_Control_Meter_PARA *meter;//转发仪表
	t_Motor_Control_SN_PARA *sn;//设置SN

		
	
	
	
	
	
	uint8_t sending_cmd_flag;//是否允许发送
	uint8_t cmd_buff[5];//命令状态//读SN,读状态,设置指令,转发仪表,设置SN
	//命令发送的时间间隔计时
	unsigned int cmd_sending_interval_cnt;
	//接收应答命令超时计时
	unsigned int cmd_rcv_timerout_cnt;
	uint8_t exe_cmd_cnt;//发送命令计数
	uint8_t comm_error_cnt;//通讯故障计数
	unsigned int start_ACC_poweroff_time_cnt;//关ACC计时
	uint16_t Stop_counter_value;//电机锁计时，30s开锁
	

	uint8_t illegal_move_flag;//非法移动标志

}t_Motor_Control;//

extern void motor_control_timer_cb(void);
extern void motor_control_init(void);
extern void Motor_Controller_Info_req(void);//读取电机控制器设备信息
extern bool Motor_Controller_Info_cnf(st_hub_cmd *pl_hub_cmd);
extern void Motor_Controller_State_req(void);//读取电机控制器实时状态
extern bool Motor_Controller_State_cnf(st_hub_cmd *pl_hub_cmd);
extern void Motor_Controller_Control1_req(void);//设置电机控制器
extern bool Motor_Controller_Control1_cnf(st_hub_cmd *pl_hub_cmd);
extern void Motor_Controller_Control2_req(void);//转发仪表
extern bool Motor_Controller_Control2_cnf(st_hub_cmd *pl_hub_cmd);
extern void Motor_Controller_SetSN_req(void);//设置电机控制器SN
extern bool Motor_Controller_SetSN_cnf(st_hub_cmd *pl_hub_cmd);

#ifdef __cplusplus
}
#endif

#endif
