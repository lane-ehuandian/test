/*
 * File      : master_rs485_list.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2012, RT-Thread Development Team
 *
 *  led run
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-12-16     lane      the first version
 * ...
 */
#ifndef __MASTER_RS485_LIST_H__
#define __MASTER_RS485_LIST_H__

#ifdef __cplusplus
extern "C"{
#endif

#include "gd32f403.h"
#include "list.h"
#include <stdio.h>
//���շ�������󣬷�����һ�������ʱ����
#define MASTER_NEXT_CMD_INTERVAL_CNT_MAX 20 // 20ms

typedef struct{
   unsigned char stl_cmd;//����
   unsigned char stl_dataL;//���ݳ���
   unsigned char *stl_data;//����ָ��
}st_hub_cmd;


typedef struct struct_mater_state{
//����͵ĳ�ʱ����
unsigned int cmd_timeout_cnt;	
//�Ƿ����ڴ�������������̣�ֱ��ɾ�������е����������
unsigned char cmd_processing_flag;
//����ͳ�ʱ��־
unsigned char cmd_snd_faild_flag;	
//���շ�������󣬷�����һ�������ʱ����
unsigned int next_cmd_interval_cnt;
}st_mater_state;
//���� ���������������
typedef enum{
    ENUM_M_CMD_NULL,
    ENUM_M_CMD_INFO = 0x10,
    ENUM_M_CMD_STATUS = 0x11,
    ENUM_M_CMD_CONTROL1 = 0x12,
    ENUM_M_CMD_CONTROL2 = 0x13,
    ENUM_M_CMD_SETSN = 0x14,
    ENUM_M_CMD_MAX
}ENUM_MASTER_CMD;
typedef enum {FALSE = 0, TRUE = !FALSE} bool;
//#pragma pack()

typedef struct struct_master_cmd{
    struct list_head list;
    //Ҫ���͸������������
    ENUM_MASTER_CMD cmd;
    //�����������ָ��
    unsigned char *data;
}st_master_cmd;

#define MASTER_CMD_NODE_MAX 3

//#pragma pack(1)

extern struct list_head master_rs485_head;
extern struct list_head *plist;
extern st_mater_state sl_master_state;

#define MASTER_DATA_BUF_MAX 16
extern st_master_cmd master_cmd_normal_node[MASTER_CMD_NODE_MAX];
extern unsigned char master_cmd_normal_node_data[MASTER_CMD_NODE_MAX][MASTER_DATA_BUF_MAX];


extern void cmd_list_init(void);
extern bool cmd_list_add(ENUM_MASTER_CMD master_cmd, unsigned char *data);
extern bool cmd_list_empty(void);
extern bool cmd_get_from_list(ENUM_MASTER_CMD *master_cmd, unsigned char **data);
extern bool cmd_delet_one_from_list(void);
extern void master_rs485_rcv_ack_process(void);

#ifdef __cplusplus
}
#endif

#endif
