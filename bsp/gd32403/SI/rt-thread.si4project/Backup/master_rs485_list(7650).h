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
//接收返回命令后，发送下一条命令的时间间隔
#define MASTER_NEXT_CMD_INTERVAL_CNT_MAX 20 // 20ms

typedef struct{
   unsigned char stl_cmd;//命令
   unsigned char stl_dataL;//数据长度
   unsigned char *stl_data;//数据指针
}st_hub_cmd;


typedef struct struct_mater_state{
//命令发送的超时计数
unsigned int cmd_timeout_cnt;	
//是否正在处理发送命令的流程，直到删除链表中的命令才清零
unsigned char cmd_processing_flag;
//命令发送超时标志
unsigned char cmd_snd_faild_flag;	
//接收返回命令后，发送下一条命令的时间间隔
unsigned int next_cmd_interval_cnt;
}st_mater_state;
//主机 电机控制器命令码
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
    //要发送给充电器的命令
    ENUM_MASTER_CMD cmd;
    //命令带的数据指针
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
