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

#define MOTOR_CONTROL_RS485_ADDR 0x83//��������ַ

#define read_sn_time_max 1000*60//*2//�����������ʱ��2����,���2����ѯ�ʵ���������豸��Ϣ
#define read_status_time_max 2000 //��ȡ���������״̬��2��

//enum{
////������������豸��Ϣ
//MOTOR_CONTROL_CMD_INFO = 0x10,
////�����������ʵʱ״̬
//MOTOR_CONTROL_CMD_STATUS = 0x11,
////���õ��������
//MOTOR_CONTROL_CMD_CONTROL1 = 0x12,
////ת���Ǳ�
//MOTOR_CONTROL_CMD_CONTROL2 = 0x13,
////���õ��������SN
//MOTOR_CONTROL_CMD_SETSN = 0x14,
//};

typedef struct
{
	uint16_t ctrl;//����ָ��
	uint16_t speed_ratio;//���ٱ���
	uint16_t ODO;//�ۼ����
	uint16_t uv;//Ƿѹֵ
	uint16_t ov;//��ѹֵ
	uint16_t oc;//����ֵ
	uint16_t speed_limt;//����ֵ
	uint16_t input_power;//�����빦��
	uint16_t recoil_v;//�޷����ѹ
	uint16_t recoil_c;//�޷������	
}t_Motor_Control_CFG_PARA;
typedef struct
{
	uint8_t month;//��
	uint8_t day;//��
	uint8_t hour;//ʱ
	uint8_t minute;//��
	uint8_t second;//��
	uint8_t Temp;//�¶�
	uint8_t RSSI;//�ź�ǿ��
	uint8_t state;//״̬��Ϣ
	uint8_t soc;//�����ٷֱ�
	uint8_t weather;//����
}t_Motor_Control_Meter_PARA;
typedef struct
{
	uint8_t sn[14];
}t_Motor_Control_SN_PARA;
typedef struct
{
	uint16_t vendor_info;//������Ϣ
	uint16_t ID;//�豸���к�
	uint16_t PW;//Э��汾
	uint16_t HW;//Ӳ���汾��
	uint16_t baud;//������
	uint8_t  sn[14];	
}t_Motor_Control_INFO_PARA;//�豸��Ϣ
typedef struct
{
	uint16_t state[4];//״̬��Ϣ
	uint16_t voltage;//��ѹֵ
	uint16_t voltage_per;//��ѹ�ٷֱ�
	uint16_t current;//����ֵ
	uint16_t speed;//ת��
	uint16_t ctrl;//����ָ��
	uint16_t speed_Ratio;//���ٱ���
	uint16_t COD;//��ǰ���
	uint16_t speed_Max;//���ת��
	uint16_t uv;//Ƿѹֵ
	uint16_t ov;//��ѹֵ
	uint16_t oc;//����ֵ
	uint16_t speed_Limt;//����ֵ
	uint16_t input_power;//�����빦��
	uint16_t recoil_Volt;//�޷����ѹ
	uint16_t recoil_Curr;//�޷������		
}t_Motor_Control_State_PARA;//ֻ������
typedef struct
{
	uint32_t read_status_time_cnt;//�����������ʱ��,���5����ѯ��״̬�豸��Ϣ
	uint32_t read_sn_time_cnt;//�����������ʱ��5����,���5����ѯ���豸��Ϣ
	
	t_Motor_Control_INFO_PARA *inf;//�豸��Ϣ	
	t_Motor_Control_State_PARA *sta;//״̬
	t_Motor_Control_CFG_PARA *cfg;//����ָ��
	t_Motor_Control_Meter_PARA *meter;//ת���Ǳ�
	t_Motor_Control_SN_PARA *sn;//����SN

		
	
	
	
	
	
	uint8_t sending_cmd_flag;//�Ƿ�������
	uint8_t cmd_buff[5];//����״̬//��SN,��״̬,����ָ��,ת���Ǳ�,����SN
	//����͵�ʱ������ʱ
	unsigned int cmd_sending_interval_cnt;
	//����Ӧ�����ʱ��ʱ
	unsigned int cmd_rcv_timerout_cnt;
	uint8_t exe_cmd_cnt;//�����������
	uint8_t comm_error_cnt;//ͨѶ���ϼ���
	unsigned int start_ACC_poweroff_time_cnt;//��ACC��ʱ
	uint16_t Stop_counter_value;//�������ʱ��30s����
	

	uint8_t illegal_move_flag;//�Ƿ��ƶ���־

}t_Motor_Control;//

extern void motor_control_timer_cb(void);
extern void motor_control_init(void);
extern void Motor_Controller_Info_req(void);//��ȡ����������豸��Ϣ
extern bool Motor_Controller_Info_cnf(st_hub_cmd *pl_hub_cmd);
extern void Motor_Controller_State_req(void);//��ȡ���������ʵʱ״̬
extern bool Motor_Controller_State_cnf(st_hub_cmd *pl_hub_cmd);
extern void Motor_Controller_Control1_req(void);//���õ��������
extern bool Motor_Controller_Control1_cnf(st_hub_cmd *pl_hub_cmd);
extern void Motor_Controller_Control2_req(void);//ת���Ǳ�
extern bool Motor_Controller_Control2_cnf(st_hub_cmd *pl_hub_cmd);
extern void Motor_Controller_SetSN_req(void);//���õ��������SN
extern bool Motor_Controller_SetSN_cnf(st_hub_cmd *pl_hub_cmd);

#ifdef __cplusplus
}
#endif

#endif
