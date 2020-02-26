/*
 * File      : rs485_protocol.h
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
#ifndef __RS485_PROTOCOL_H__
#define __RS485_PROTOCOL_H__

#ifdef __cplusplus
extern "C"{
#endif

#include "gd32f403.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "app_fifo.h"
#include "master_rs485_list.h"
#include <stdio.h> 
#define BUFFERSIZE          256//发送接收数据长度

#define RS485_FRAME_HEAD 0X7E
#define RS485_FRAME_TAIL 0XFF
//通讯规约位置定义
#define HUM_FRAME_INDEX_ADD 0
#define HUM_FRAME_INDEX_CMD 1
#define HUM_FRAME_INDEX_DATAL 2
#define HUM_FRAME_INDEX_DATA_START 3

#define APP_ERROR_BASE_NUM      (0x0)       ///< Global error base

#define APP_SUCCESS                           (APP_ERROR_BASE_NUM + 0)  ///< Successful command
#define APP_ERROR_INTERNAL                    (APP_ERROR_BASE_NUM + 1)  ///< Internal Error
#define APP_ERROR_NO_MEM                      (APP_ERROR_BASE_NUM + 2)  ///< No Memory for operation
#define APP_ERROR_NOT_FOUND                   (APP_ERROR_BASE_NUM + 3)  ///< Not found
#define APP_ERROR_INVALID_PARAM               (APP_ERROR_BASE_NUM + 4)  ///< Invalid Parameter
#define APP_ERROR_INVALID_STATE               (APP_ERROR_BASE_NUM + 5)  ///< Invalid state, operation disallowed in this state
#define APP_ERROR_BUSY                        (APP_ERROR_BASE_NUM + 6) ///< Busy

//#define HUM_FRAME_INDEX_CTRL 0


#define SLAVE_RS485_COMM_TX_DELAY_CNT_MAX_SHORT 110 
#define SLAVE_RS485_COMM_TX_DELAY_CNT_MAX_LONG 5000 

 enum{
ENUM_UPDATE_TAR_SMART_BOARD = 0,
ENUM_UPDATE_TAR_PMS_BOARD = 1,
ENUM_UPDATE_TAR_BMS_BOARD = 2,
ENUM_UPDATE_TAR_CHARGER = 4,

};
//用来标志哪些数据已经从BMS中读取完成，可以供参考使用
#define BMS_DEV_INFO_FLAG_BIT (1<<0)
#define BMS_READONLY_DATA1_FLAG_BIT (1<<1)
#define BMS_READONLY_DATA2_FLAG_BIT (1<<2)
#define BMS_CTRL_FLAG_BIT (1<<3)
#define BMS_PARAM1_FLAG_BIT (1<<4)
#define BMS_PARAM2_FLAG_BIT (1<<5)
#define BMS_USER_FLAG_BIT (1<<6) 

//从机接收命令码
enum{
//SLAVE_RS485_CMD_UNLOCK = 0x31,//开锁命令
	
SLAVE_RS485_CMD_DEV_VERSION = 0x20,

SLAVE_RS485_CMD_UPDATE_START = 0x21,
SLAVE_RS485_CMD_UPDATE_EXCHANGE = 0x22,
SLAVE_RS485_CMD_UPDATE_DONE = 0x23,
SLAVE_RS485_CMD_CFG = 0x24,
SLAVE_RS485_CMD_FAN_TEMP_SETTING = 0x25,// 设置风扇的温度阈值
SLAVE_RS485_CMD_READ_BAT_RECORD = 0x26, // 读取电池历史数据
SLAVE_RS485_CMD_BYPASS = 0x27,
SLAVE_RS485_CMD_SN_SETTING = 0x28,


SLAVE_RS485_CMD_UNLOCK = 0x31,//开锁命令
SLAVE_RS485_CMD_CHARGE_EN = 0x32,//使能充电命令
SLAVE_RS485_CMD_CHECK_BAT = 0x33,//查询电池状态
SLAVE_RS485_CMD_CHECK_LOCK_PORT_CHG = 0x34,//查询锁、槽位和充电状态命令
SLAVE_RS485_CMD_DISCHARGE_EN = 0x37,//关闭充电命令

SLAVE_RS485_CMD_GET_BAT_RECORD_PARAM = 0x49,


//测试命令

SLAVE_RS485_CMD_FAN_EN = 0x70,//打开风扇
SLAVE_RS485_CMD_FAN_DISABLE = 0x71,//关闭风扇
SLAVE_RS485_CMD_R_RUNING_PARAM = 0x72, // 读取正在运行的参数
SLAVE_RS485_CMD_R_REAL_TIME_INFO = 0x73, // 读取正在运行的信息
SLAVE_RS485_CMD_R_BAT_INFO = 0x74, // 读取电池的重要信息
SLAVE_RS485_CMD_R_BAT_INFO_WHEN_CRITICAL_ERR = 0x75, // 读取发送错误的时候保存的寄存器信息
SLAVE_RS485_CMD_R_BAT_INFO_WHEN_EVENT_TO_STOP_CHG = 0x76, //读取检测到发生错误时候，现场保存
SLAVE_RS485_CMD_R_BAT_INFO_WHEN_DETECT_BAT_ERROR = 0x77, //读取检测到发生错误时候，现场保存
SLAVE_RS485_CMD_R_ADC_RAW_VALUE = 0x78, // 读取ADC原始数据
SLAVE_RS485_CMD_HEATER_ONOFF = 0x79, // 打开或者关闭加热模式，不用管是否允许加热。


//清除故障标志
SLAVE_RS485_CMD_CLEAR_PMS_ERROR_FLAG = 0x80, //清除PMS发现的故障标志
//重启PMS
SLAVE_RS485_CMD_PMS_RESET = 0x81, 
//禁止和允许主动测试电池是否有故障
SLAVE_RS485_CMD_DETECT_BAT_ERROR = 0x82,
//测试命令，用于调试一下不方便创造的条件
SLAVE_RS485_CMD_TEST_TEST = 0x83,

//设置停止检测故障的标志
SLAVE_RS485_CMD_EVENT_TO_STOP_CHARGING_FLAG_BE_DISABLE_FLAG = 0x84,
//清除停止检测故障的标志
SLAVE_RS485_CMD_EVENT_TO_STOP_CHARGING_FLAG_BE_DISABLE_FLAG_CLEAR = 0x85,
//设置/清除充电器给电池仓加热的使能
SLAVE_RS485_CMD_CHARGER_WARMER_ENABLE = 0x86,
//设置PMS认证电池的功能启用与否
SLAVE_RS485_CMD_PMS_BAT_AUTHORITY_ENABLE = 0x87,
//读取充电器的温度
SLAVE_RS485_CMD_READ_CHARGER_TEMP_VALUE = 0x88
};



typedef enum
{
  /** Disable both receiver and transmitter. */
  ENUM_USART_DISABLE  = 0x0,

  /** Enable receiver only, transmitter disabled. */
  ENUM_USART_RX_ENABLE,

  /** Enable transmitter only, receiver disabled. */
  ENUM_USART_TX_ENABLE,

  /** Enable both receiver and transmitter. */
  ENUM_USART_RX_TX_ENABLE
}ENUM_USART_EN_TYPE;




/*功率线模式切换；加热模式，充电器模式，放电模式*/
typedef enum{
    ENUM_POWER_MODE_IDLE,
    ENUM_POWER_MODE_CHARGE,
    ENUM_POWER_MODE_HEATER,
    ENUM_POWER_MODE_CHARGE_HEATER,
    ENUM_POWER_MODE_LOADER
}enum_power_mode;


typedef enum{
    ENUM_COMM_INDEX_COMM,
	ENUM_COMM_INDEX_MASTER,
    ENUM_COMM_INDEX_MAX
}enum_comm_index;
 typedef enum{
     MASTER_RS485_RUN_TX,
     MASTER_RS485_RUN_TX_WAIT_COMPLETE,
     MASTER_RS485_RUN_RX        
 }rs485_run_state_t;
 
//发送数据结构体
typedef struct{
    uint8_t rs485_tx_buf[BUFFERSIZE];
    int16_t rs485_tx_indx;
    bool rs485_tx_buf_busy;
}rs485_tx_t;
//接收数据结构体
typedef struct{
    uint8_t rs485_rx_buf[BUFFERSIZE];
    int16_t rs485_rx_indx;
    bool rs485_rx_sta_head;
    bool rs485_rx_sta_tail;
    bool rs485_rx_timer_to;
}rs485_rx_t;


#define CHARGER_USART_RX_BUF_MAX 256
#define CHARGER_USART_CONFIG_DEFAULT           \
{                                          \
    BAUD_RATE_9600, /* 9600 bits/s */  \
    DATA_BITS_8,      /* 8 databits */     \
    STOP_BITS_1,      /* 1 stopbit */      \
    PARITY_EVEN,      /* EVEN parity  */     \
    BIT_ORDER_LSB,    /* LSB first sent */ \
    NRZ_NORMAL,       /* Normal mode */    \
    RT_SERIAL_RB_BUFSZ, /* Buffer size */  \
    0                                      \
}
typedef struct{
    struct rt_semaphore usart_rx_sem;
    unsigned char usart_buf[CHARGER_USART_RX_BUF_MAX];
    rt_device_t usart_dev;

    app_fifo_t usart_rx_fifo;
    app_fifo_t usart_tx_fifo;

    unsigned char usart_tx_flag:1;/* 串口正在发送的标志 */
    unsigned char bits_reserved:7;
    /*串口配置参数*/
    struct serial_configure usart_config_param;
}st_charger_param;
 typedef enum{
     MASTER_RS485_RESET,
     MASTER_RS485_RUN,
     MASTER_RS485_SLEEP
 }rs485_state_t;

 
extern rs485_tx_t *pl_rs485_tx[ENUM_COMM_INDEX_MAX];//发送数据指针

 
extern unsigned char usart_rx_fifo_buf[CHARGER_USART_RX_BUF_MAX];
extern unsigned char usart_tx_fifo_buf[CHARGER_USART_RX_BUF_MAX];
//与上位机通讯时候，填写发送数据的地方设置个计数时间，超过这个时间则不再发送这些数据
//#ifdef COMM_BOARD_PROTOCOL_C
//GLOBAL_CMP unsigned int slave_rs485_comm_TX_delay_cnt_max = 0;
//#else
extern unsigned int slave_rs485_comm_TX_delay_cnt_max;
//#endif
extern st_charger_param gl_charger_paramdfg;
extern st_charger_param gl_rs485_param;


 
extern rs485_run_state_t slave_rs485_run_state[ENUM_COMM_INDEX_MAX];

typedef struct {
unsigned char cmd;
bool (*cmd_process_func)(st_hub_cmd *);
}st_slave_rs485_cmd; 
 
extern void slave_rs485_calc_TX_tail(void);
extern unsigned char slave_rs485_get_dev_addr(void); 
unsigned char slave_rs485_changed_bms_index_from_comm(unsigned char bms_index);
 
extern unsigned short slave_rs485_calc_CRC(unsigned char *pl_data, unsigned short len);
extern void slave_rs485_communication_init(enum_comm_index index);
extern void master_rs485_communication(rs485_state_t rs485_state); 
extern void master_rs485_process(void);
extern void master_rs485_timer_cb(void);
#ifdef __cplusplus
}
#endif

#endif
