/*
 * File      : comm_board_protocol.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2012, RT-Thread Development Team
 *
 *  rs485 thread
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-12-25     lane      the first version
 * ...
 */

#define COMM_BOARD_PROTOCOL_C
#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include "IO_config.h"
#include "comm_board_protocol.h"


#include "battery_process.h"
#include "xmodem.h"
#include "ntag_app.h"
#include "mis.h"

#include "motor_control.h"
#include "master_rs485_list.h"



st_charger_param gl_rs485_param;


/*
target: 
0: CTRL board(Port#无意义)
1: PMS(指定Port#端口号的PMS)
2: BAT(指定Port#端口号的电池)

*/
void slave_rs485_cmd_update_cnf(unsigned char target, unsigned char cmd, enum_comm_error_code error_code)
{
    //unsigned short vl_crc;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = cmd;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x03;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[3] = target;//target 
//    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[4] = slave_rs485_cmd_update_port;//PORT 
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = error_code;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 6;

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    
    //设置发送超时计数值，超时就不再发送数据
    slave_rs485_comm_TX_delay_cnt_max = SLAVE_RS485_COMM_TX_DELAY_CNT_MAX_LONG;
    slave_rs485_run_state[ENUM_COMM_INDEX_COMM]=MASTER_RS485_RUN_TX;
}




static rt_err_t master_rs485_charger_tx_done_cb(rt_device_t dev,void * buffer)
{
    gl_rs485_param.usart_tx_flag = 0;
    return RT_EOK;
}

static rt_err_t master_rs485_charger_input_cb(rt_device_t dev, rt_size_t size)
{

    rt_sem_release(&gl_rs485_param.usart_rx_sem);

    return RT_EOK;
}
void master_rs485_init(void)
{
    rt_pin_mode(RS485_DIR_PIN, PIN_MODE_OUTPUT);
	rt_pin_write(RS485_DIR_PIN, PIN_LOW);	    
    cmd_list_init();
    //init param
    gl_rs485_param.usart_tx_flag = 0;
    gl_rs485_param.usart_config_param.baud_rate = BAUD_RATE_9600; /* 9600 bits/s */
    gl_rs485_param.usart_config_param.data_bits = DATA_BITS_9;      /* 8 databits */
    gl_rs485_param.usart_config_param.stop_bits = STOP_BITS_1;      /* 1 stopbit */
    gl_rs485_param.usart_config_param.parity = PARITY_EVEN;      /* EVEN parity  */
    gl_rs485_param.usart_config_param.bit_order = BIT_ORDER_LSB;    /* LSB first sent */
    gl_rs485_param.usart_config_param.invert = NRZ_NORMAL;       /* Normal mode */
    gl_rs485_param.usart_config_param.bufsz = RT_SERIAL_RB_BUFSZ; /* Buffer size */
    gl_rs485_param.usart_config_param.reserved = 0;		
}
void thread_rs485_entry(void* parameter)
{
    rt_tick_t vl_delay_tick_cnt;
    master_rs485_init();

    app_fifo_init(&gl_rs485_param.usart_rx_fifo, usart_rx_fifo_buf, CHARGER_USART_RX_BUF_MAX);
    app_fifo_init(&gl_rs485_param.usart_tx_fifo, usart_tx_fifo_buf, CHARGER_USART_RX_BUF_MAX);
    
    rt_sem_init(&gl_rs485_param.usart_rx_sem, "RS485_SEM", 0, RT_IPC_FLAG_FIFO);
      
    gl_rs485_param.usart_dev = rt_device_find(RS485_USART_DEV_NAME);
    rt_device_open(gl_rs485_param.usart_dev, RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_TX);
    rt_device_control(gl_rs485_param.usart_dev, RT_DEVICE_CTRL_CONFIG, &gl_rs485_param.usart_config_param);
    rt_device_set_tx_complete(gl_rs485_param.usart_dev, master_rs485_charger_tx_done_cb);
    rt_device_set_rx_indicate(gl_rs485_param.usart_dev, master_rs485_charger_input_cb);   

    master_rs485_communication(MASTER_RS485_RESET);
	
    motor_control_init();

    rt_thread_mdelay(10);

    vl_delay_tick_cnt = rt_tick_from_millisecond(1);
    while(1)
    {
        master_rs485_process();
        //rt_thread_mdelay(1);
        if(RT_EOK == rt_sem_take(&gl_rs485_param.usart_rx_sem, vl_delay_tick_cnt))
        {
            unsigned char ch;
            if(rt_device_read(gl_rs485_param.usart_dev, -1, &ch, 1) == 1)
            {
                if(app_fifo_length(&gl_rs485_param.usart_rx_fifo)<= gl_rs485_param.usart_rx_fifo.buf_size_mask)
                {
                    app_fifo_put(&gl_rs485_param.usart_rx_fifo, ch);
                }
            }
        }
        rt_thread_mdelay(10);
    }
}

