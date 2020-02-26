/*
 * File      : rs485_protocol.c
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

#include "rs485_protocol.h"
#include "IO_config.h"

#include <string.h>
#include "fm175xx.h"
#include "battery_process.h"
#include "motor_control.h"
//发送变量
rs485_tx_t master_rs485_tx={
    .rs485_tx_indx=0,
    .rs485_tx_buf_busy=FALSE        
};
rs485_tx_t *pl_rs485_tx[ENUM_COMM_INDEX_MAX];//发送数据指针
static uint8_t *pl_DMA_buf[ENUM_COMM_INDEX_MAX];//发送数据获取内存指针
//接收变量
rs485_rx_t master_rs485_rx={
    .rs485_rx_indx=0,
    .rs485_rx_sta_head=FALSE,
    .rs485_rx_sta_tail=FALSE,
    .rs485_rx_timer_to=FALSE
};
rs485_rx_t *pl_rs485_rx[ENUM_COMM_INDEX_MAX];//接收数据指针
st_charger_param gl_rs485_param;

st_mater_state sl_master_state;
uint8_t usart0_rx_buf[BUFFERSIZE];

static uint8_t *pl_rx_buf[ENUM_COMM_INDEX_MAX];


static uint32_t sl_sizeof_tx_buf[ENUM_COMM_INDEX_MAX];
static uint32_t sl_sizeof_rx_buf[ENUM_COMM_INDEX_MAX];
unsigned char usart_rx_fifo_buf[CHARGER_USART_RX_BUF_MAX];
unsigned char usart_tx_fifo_buf[CHARGER_USART_RX_BUF_MAX];
rs485_tx_t slave_rs485_tx={
    .rs485_tx_indx=0,
    .rs485_tx_buf_busy=FALSE        
};

rs485_rx_t slave_rs485_rx={
    .rs485_rx_indx=0,
    .rs485_rx_sta_head=FALSE,
    .rs485_rx_sta_tail=FALSE,
    .rs485_rx_timer_to=FALSE
};







static void (*slave_rs485_usart_enable[ENUM_COMM_INDEX_MAX])(ENUM_USART_EN_TYPE);

static uint32_t (*slave_rs485_put_byte[ENUM_COMM_INDEX_MAX])(uint8_t);
static uint32_t (*slave_rs485_put_bytes_by_dma[ENUM_COMM_INDEX_MAX])(uint8_t*, uint8_t);
static void (*slave_rs485_flush_tx_fifo[ENUM_COMM_INDEX_MAX])(void);
static bool (*slave_rs485_tx_busy[ENUM_COMM_INDEX_MAX])(void);
static uint32_t (*slave_rs485_get_byte[ENUM_COMM_INDEX_MAX])(uint8_t*);

bool slave_rs485_cmd_unlock(st_hub_cmd *pl_hub_cmd);

st_slave_rs485_cmd gl_slave_rs485_cmd[] = {
{SLAVE_RS485_CMD_UNLOCK, slave_rs485_cmd_unlock},
};
const st_slave_rs485_cmd gl_master_rs485_cmd[] = {
{ENUM_M_CMD_INFO, Motor_Controller_Info_cnf},
{ENUM_M_CMD_STATUS, Motor_Controller_State_cnf},
{ENUM_M_CMD_CONTROL1, Motor_Controller_Control1_cnf},
{ENUM_M_CMD_CONTROL2, Motor_Controller_Control2_cnf},
{ENUM_M_CMD_SETSN, Motor_Controller_SetSN_cnf},
};

struct list_head master_rs485_head;
st_master_cmd master_cmd_normal_node[MASTER_CMD_NODE_MAX];
unsigned char master_cmd_normal_node_data[MASTER_CMD_NODE_MAX][MASTER_DATA_BUF_MAX];

bool cmd_list_add(ENUM_MASTER_CMD master_cmd, unsigned char *data)
{
    unsigned char i;
    for(i = 0; i < MASTER_CMD_NODE_MAX; i++)
    {
        if(list_empty(&master_cmd_normal_node[i].list))
        {//这里死机
            master_cmd_normal_node[i].cmd = master_cmd;
            if(NULL != data)
            {
                memcpy(&master_cmd_normal_node_data[i][0], data, MASTER_DATA_BUF_MAX);
            }            
            master_cmd_normal_node[i].data = &master_cmd_normal_node_data[i][0];
            list_add_tail(&master_cmd_normal_node[i].list, &master_rs485_head);
            return TRUE;
        }
    }
    return FALSE;
}
bool cmd_list_empty(void)
{
    if(list_empty(&master_rs485_head))
    {
        return TRUE;
    }
    return FALSE;
}

bool cmd_get_from_list(ENUM_MASTER_CMD *master_cmd, unsigned char **data)
{
    st_master_cmd *pl_cmd;
    if(!list_empty(&master_rs485_head))
    {
        pl_cmd = list_entry(master_rs485_head.next, st_master_cmd, list);

        *master_cmd = pl_cmd->cmd;
        *data = pl_cmd->data;
        return TRUE;
    }
    return FALSE;
}

bool cmd_delet_one_from_list(void)
{
    if(!list_empty(&master_rs485_head))
    {
        list_del_init(master_rs485_head.next);
        return TRUE;
    }
    return FALSE;
}
void cmd_list_init(void)
{    
    unsigned char i;    
    //初始化链表
    INIT_LIST_HEAD(&master_rs485_head);
    //初始化节点数据
    for(i = 0; i < MASTER_CMD_NODE_MAX; i++)
    {
        INIT_LIST_HEAD(&master_cmd_normal_node[i].list);
    }
}
//接收到正确的应答包，删除命令和重置变量
void master_rs485_rcv_ack_process(void)
{
    //删除链表中的命令
    cmd_delet_one_from_list();
    sl_master_state.cmd_processing_flag = 0;
    sl_master_state.next_cmd_interval_cnt = MASTER_NEXT_CMD_INTERVAL_CNT_MAX;
    sl_master_state.cmd_timeout_cnt = 0;
}

bool slave_rs485_cmd_unlock(st_hub_cmd *pl_hub_cmd)
{


}


unsigned short slave_rs485_calc_CRC(unsigned char *pl_data, unsigned short len)
{
    unsigned short vl_crc;
    unsigned short i;
    unsigned char *vl_p;
    vl_p = pl_data;
    for(i = 0, vl_crc = 0; i < len; i++)
    {
        vl_crc += vl_p[i];
    }
    return vl_crc;
}
void slave_rs485_calc_TX_tail(void)
{
    unsigned short vl_crc;
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
}
//根据485地址转换bms_index为0~(NFC_READER_COUNT_MAX-1)范围
unsigned char slave_rs485_changed_bms_index_from_comm(unsigned char bms_index)
{
    unsigned char vl_base_addr;
    vl_base_addr = ((slave_rs485_get_dev_addr() - 1) * NFC_READER_COUNT_MAX);
    if(vl_base_addr <= bms_index)
    {
        bms_index -= vl_base_addr;
    }
    else
    {
        bms_index = 0xFF;
    }
    return bms_index;
}
rs485_run_state_t slave_rs485_run_state[ENUM_COMM_INDEX_MAX]={MASTER_RS485_RUN_TX, MASTER_RS485_RUN_TX};
//从串口接收缓冲中提取接收到的数据
void slave_rs485_get_rx_data_from_buf(enum_comm_index index, unsigned char** data_out, unsigned int *data_len)
{
    *data_out = &pl_rs485_rx[index]->rs485_rx_buf[0];
    *data_len = pl_rs485_rx[index]->rs485_rx_indx;
    
}
void drv_usart0_enable(ENUM_USART_EN_TYPE enable_type)
{
    unsigned char vl_ctrl_cmd;
    switch(enable_type)
    {
        /** Disable both receiver and transmitter. */
        case ENUM_USART_DISABLE:
            vl_ctrl_cmd = RT_DEVICE_CTRL_USART_DISABLE;
            break;
        /** Enable receiver only, transmitter disabled. */
        case ENUM_USART_RX_ENABLE:
            vl_ctrl_cmd = RT_DEVICE_CTRL_USART_RX_ENABLE;
            break;
        /** Enable transmitter only, receiver disabled. */
        case ENUM_USART_TX_ENABLE:
            vl_ctrl_cmd = RT_DEVICE_CTRL_USART_TX_ENABLE;
            break;
        /** Enable both receiver and transmitter. */
        case ENUM_USART_RX_TX_ENABLE:
            vl_ctrl_cmd = RT_DEVICE_CTRL_USART_RX_TX_ENABLE;
            break;
    }
    if(RT_NULL != gl_rs485_param.usart_dev)
    {
        rt_device_control(gl_rs485_param.usart_dev, vl_ctrl_cmd, RT_NULL);
    }
}
uint32_t drv_usart0_put_byte(uint8_t data)
{    
    uint32_t sta;
    sta=app_fifo_put(&gl_rs485_param.usart_tx_fifo, data);
    /* Enable interrupt on USART TX Buffer*/
    //USART_IntEnable(USART0, USART_IEN_TXBL); 
    return sta;
    
}

uint32_t drv_usart0_put_bytes_by_dma(uint8_t *data, unsigned char data_len)
{    
    if(RT_NULL != gl_rs485_param.usart_dev)
    {
        gl_rs485_param.usart_tx_flag = 1;
        rt_device_write(gl_rs485_param.usart_dev, 0, data, data_len);
    }
}

void drv_usart0_flush_tx_fifo(void)
{
    app_fifo_flush(&gl_rs485_param.usart_tx_fifo);
}

bool drv_usart0_tx_busy(void)
{    
    if((0 == app_fifo_length(&gl_rs485_param.usart_tx_fifo)) 
        && (0 == gl_rs485_param.usart_tx_flag)
        &&(RT_NULL != gl_rs485_param.usart_dev)
        &&(RT_EBUSY != rt_device_control(gl_rs485_param.usart_dev, RT_DEVICE_CTRL_USART_GET_TC, RT_NULL)))
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}


uint32_t drv_usart0_get_byte(uint8_t *data)
{    
    return app_fifo_get(&gl_rs485_param.usart_rx_fifo,data);
}



static void slave_rs485_TX_enable(enum_comm_index index, bool enable)
{
	if(index == ENUM_COMM_INDEX_COMM)
    {
    }
    else if(index == ENUM_COMM_INDEX_MASTER)
    {
        if(enable)
        {
            rt_pin_write(RS485_DIR_PIN, PIN_HIGH);
        }
        else
        {
            rt_pin_write(RS485_DIR_PIN, PIN_LOW);
        }
    }
}




void master_rs485_timer_cb(void)
{
    //发送命令超时计数
    if(sl_master_state.cmd_timeout_cnt)
    {
        sl_master_state.cmd_timeout_cnt--;
        if(!sl_master_state.cmd_timeout_cnt)
        {
            sl_master_state.cmd_snd_faild_flag = 1;
        }
    }
    if(sl_master_state.next_cmd_interval_cnt)
        sl_master_state.next_cmd_interval_cnt--;
    motor_control_timer_cb();    
}
//存储PMS被设置的485地址
static unsigned char gl_RS485_dev_addr;
unsigned char slave_rs485_get_dev_addr(void)
{
    return gl_RS485_dev_addr;
}
bool slave_rs485_cmd_process(st_hub_cmd *pl_hub_cmd)
{
    unsigned int i, vl_cmd_cnt;
    vl_cmd_cnt = sizeof(gl_slave_rs485_cmd)/sizeof(st_slave_rs485_cmd);
    for(i = 0; i < vl_cmd_cnt; i++)
    {
        if(pl_hub_cmd->stl_cmd == gl_slave_rs485_cmd[i].cmd)
        {
            return gl_slave_rs485_cmd[i].cmd_process_func(pl_hub_cmd);
        }
    }
    return FALSE;
}
bool master_rs485_rcv_cmd_process(st_hub_cmd *pl_hub_cmd)
{
    unsigned int i, vl_cmd_cnt;
    vl_cmd_cnt = sizeof(gl_master_rs485_cmd)/sizeof(st_slave_rs485_cmd);
    for(i = 0; i < vl_cmd_cnt; i++)
    {
        if(pl_hub_cmd->stl_cmd == gl_master_rs485_cmd[i].cmd)
        {
            return gl_master_rs485_cmd[i].cmd_process_func(pl_hub_cmd);
        }
    }
    return FALSE;
}
bool slave_rs485_data_handler_new_protocol(void)
{
    uint16_t crc=0;
    uint8_t i;
    st_hub_cmd vl_hub_cmd;
    for(i=0;i<(slave_rs485_rx.rs485_rx_indx-2);i++)
    {
        crc+=slave_rs485_rx.rs485_rx_buf[i];
    }
    if(slave_rs485_rx.rs485_rx_buf[(slave_rs485_rx.rs485_rx_indx-2)]==((uint8_t)(0xFF&(crc>>0))) &&\
        slave_rs485_rx.rs485_rx_buf[(slave_rs485_rx.rs485_rx_indx-1)]==((uint8_t)(0xFF&(crc>>8))))
    {
        //vl_hub_cmd.stl_ctrl = slave_rs485_rx.rs485_rx_buf[HUM_FRAME_INDEX_CTRL];
        if(slave_rs485_rx.rs485_rx_buf[0] != slave_rs485_get_dev_addr())
        {
            //判断地址是否是给自己的
            return FALSE;
        }
        vl_hub_cmd.stl_cmd = slave_rs485_rx.rs485_rx_buf[HUM_FRAME_INDEX_CMD];
        vl_hub_cmd.stl_dataL = slave_rs485_rx.rs485_rx_buf[HUM_FRAME_INDEX_DATAL];
        vl_hub_cmd.stl_data = &slave_rs485_rx.rs485_rx_buf[HUM_FRAME_INDEX_DATA_START];
        
        return slave_rs485_cmd_process(&vl_hub_cmd);
    }
    return FALSE;
}

bool master_rs485_data_handler_new_protocol(void)
{
    uint16_t crc=0;
    uint8_t i;
    st_hub_cmd vl_hub_cmd;

    //是否在命令处理有效期接收到数据
    if(!sl_master_state.cmd_processing_flag)
        return FALSE;

    for(i=0;i<(master_rs485_rx.rs485_rx_indx-2);i++)
    {
        crc+=master_rs485_rx.rs485_rx_buf[i];
    }
    if(master_rs485_rx.rs485_rx_buf[(master_rs485_rx.rs485_rx_indx-2)]==((uint8_t)(0xFF&(crc>>0))) &&\
        master_rs485_rx.rs485_rx_buf[(master_rs485_rx.rs485_rx_indx-1)]==((uint8_t)(0xFF&(crc>>8))))
    {
        if(master_rs485_rx.rs485_rx_buf[HUM_FRAME_INDEX_ADD] != MOTOR_CONTROL_RS485_ADDR)
        {
            //判断地址是否是给自己的
            return FALSE;
        }
        vl_hub_cmd.stl_cmd = master_rs485_rx.rs485_rx_buf[HUM_FRAME_INDEX_CMD];
        vl_hub_cmd.stl_dataL = master_rs485_rx.rs485_rx_buf[HUM_FRAME_INDEX_DATAL];
        vl_hub_cmd.stl_data = &master_rs485_rx.rs485_rx_buf[HUM_FRAME_INDEX_DATA_START];
        
        return master_rs485_rcv_cmd_process(&vl_hub_cmd);
    }
    return FALSE;
}


void slave_rs485_communication_init(enum_comm_index index)
{
    if(ENUM_COMM_INDEX_COMM == index)
    {        
    }
    else if(ENUM_COMM_INDEX_MASTER == index)
    {
        slave_rs485_usart_enable[index] = drv_usart0_enable;
        
        pl_rs485_tx[index] = &master_rs485_tx;
        pl_rs485_rx[index] = &master_rs485_rx;

        /* 这段代码可能没有 用*/
        pl_rx_buf[index] = (uint8_t *)usart0_rx_buf;
        sl_sizeof_rx_buf[index] = sizeof(usart0_rx_buf);
		
        pl_DMA_buf[index] = RT_NULL;
        
        slave_rs485_put_byte[index] = drv_usart0_put_byte;
        slave_rs485_put_bytes_by_dma[index] = drv_usart0_put_bytes_by_dma;
        slave_rs485_flush_tx_fifo[index] = drv_usart0_flush_tx_fifo;
        slave_rs485_tx_busy[index] = drv_usart0_tx_busy;
        slave_rs485_get_byte[index] = drv_usart0_get_byte;

        cmd_list_init();
		app_fifo_init(&gl_rs485_param.usart_rx_fifo, usart_rx_fifo_buf, CHARGER_USART_RX_BUF_MAX);
		app_fifo_init(&gl_rs485_param.usart_tx_fifo, usart_tx_fifo_buf, CHARGER_USART_RX_BUF_MAX);

		motor_control_init();

		memset(&sl_master_state,0,sizeof(st_mater_state));
		
    }
}
//index: rs485通道号
void slave_rs485_communication(enum_comm_index index, rs485_state_t rs485_state)
{
    uint8_t time_of=0;    
    switch(rs485_state)
    {
        case MASTER_RS485_RESET:
            slave_rs485_communication_init(index);
            slave_rs485_run_state[index]=MASTER_RS485_RUN_RX;
            slave_rs485_TX_enable(index, FALSE);
        break;
        case MASTER_RS485_RUN://发送数据给中控，从中控接收数据并处理            
            switch(slave_rs485_run_state[index])
            {
                uint8_t data;
                case MASTER_RS485_RUN_TX:
                    slave_rs485_flush_tx_fifo[index]();                    
                    slave_rs485_TX_enable(index, TRUE);
                    {
                        unsigned char k;
                        if(RT_NULL == pl_DMA_buf[index])
                        {
                            pl_DMA_buf[index] = (unsigned char *)rt_malloc(256);
                        }
                        if(RT_NULL == pl_DMA_buf[index])
                        {                            
                        }
                        else
                        {
                            k = 0;
                            pl_DMA_buf[index][k++] = RS485_FRAME_HEAD;
                            for (uint32_t i = 0; i < (pl_rs485_tx[index]->rs485_tx_indx); i++)
                            {
                                if(pl_rs485_tx[index]->rs485_tx_buf[i]==0x7e)
                                {
                                    pl_DMA_buf[index][k++] = 0x8c;
                                    pl_DMA_buf[index][k++] = 0x81;
                                }
                                else if(pl_rs485_tx[index]->rs485_tx_buf[i]==0xff)
                                {
                                    pl_DMA_buf[index][k++] = 0x8c;
                                    pl_DMA_buf[index][k++] = 0x00;
                                }
                                else if(pl_rs485_tx[index]->rs485_tx_buf[i]==0x8c)
                                {
                                    pl_DMA_buf[index][k++] = 0x8c;
                                    pl_DMA_buf[index][k++] = 0x73;
                                }
                                else
                                {
                                    pl_DMA_buf[index][k++] = (pl_rs485_tx[index]->rs485_tx_buf[i]);
                                }
                            }
                            pl_DMA_buf[index][k++] = RS485_FRAME_TAIL;                            
                            slave_rs485_put_bytes_by_dma[index](pl_DMA_buf[index], k);
                        }
                    }
                    slave_rs485_run_state[index]=MASTER_RS485_RUN_TX_WAIT_COMPLETE;
                    pl_rs485_tx[index]->rs485_tx_indx = 0;
                break;
                case MASTER_RS485_RUN_TX_WAIT_COMPLETE:                    
                    if(!slave_rs485_tx_busy[index]())
                    {
                        rt_free(pl_DMA_buf[index]);
                        pl_DMA_buf[index] = RT_NULL;
                        //disable rs485 tx
                        slave_rs485_TX_enable(index, FALSE);
                        pl_rs485_rx[index]->rs485_rx_timer_to=FALSE;
                        pl_rs485_rx[index]->rs485_rx_sta_head=FALSE;
                        pl_rs485_rx[index]->rs485_rx_sta_tail=FALSE;
                        pl_rs485_rx[index]->rs485_rx_indx=0;  
                        //uart1_rxBuf=(uart_circularBuf_T){ {0}, 0, 0, 0, FALSE }; 
                        memset(pl_rx_buf[index], 0, sl_sizeof_rx_buf[index]);
                        slave_rs485_run_state[index]=MASTER_RS485_RUN_RX;
                    }
                break;
                case MASTER_RS485_RUN_RX:                                        
                    slave_rs485_TX_enable(index, FALSE);
                    while(slave_rs485_get_byte[index](&data)==APP_SUCCESS)        
                    {
                        static uint8_t pre_data=0x00;
                        if(data==0x7e)                            
                        {
                            pl_rs485_rx[index]->rs485_rx_sta_head=TRUE;
                            pl_rs485_rx[index]->rs485_rx_sta_tail=FALSE;
                            pl_rs485_rx[index]->rs485_rx_indx=0;
                            pre_data=0x00;
                        }
                        else if((pl_rs485_rx[index]->rs485_rx_sta_head==TRUE) && (pl_rs485_rx[index]->rs485_rx_sta_tail==FALSE))
                        {
                            if(pl_rs485_rx[index]->rs485_rx_indx<BUFFERSIZE)
                            {                                
                                if(pre_data == 0x8c && data==0x81)
                                {
                                    pl_rs485_rx[index]->rs485_rx_buf[pl_rs485_rx[index]->rs485_rx_indx]=0x7e;  
                                    pl_rs485_rx[index]->rs485_rx_indx++; 
                                }
                                else if(pre_data == 0x8c && data==0x00)
                                {
                                    pl_rs485_rx[index]->rs485_rx_buf[pl_rs485_rx[index]->rs485_rx_indx]=0xff;  
                                    pl_rs485_rx[index]->rs485_rx_indx++; 
                                }
                                else if(pre_data == 0x8c && data==0x73)
                                {
                                    pl_rs485_rx[index]->rs485_rx_buf[pl_rs485_rx[index]->rs485_rx_indx]=0x8c;  
                                    pl_rs485_rx[index]->rs485_rx_indx++; 
                                }
                                else if(data!=0x8c&&data!=0xff)
                                {
                                    pl_rs485_rx[index]->rs485_rx_buf[pl_rs485_rx[index]->rs485_rx_indx]=data;  
                                    pl_rs485_rx[index]->rs485_rx_indx++; 
                                }                                
                                else if(data==0xff)
                                {
                                    bool vl_result = FALSE;
                                    if(index == ENUM_COMM_INDEX_COMM)//从机
                                    {
                                    }
                                    else if(index == ENUM_COMM_INDEX_MASTER)
                                    {
                                        vl_result = master_rs485_data_handler_new_protocol();
                                        vl_result = FALSE; // 主机不用回应
                                    }
                                    if(vl_result)
                                    {
                                        slave_rs485_run_state[index]=MASTER_RS485_RUN_TX;
                                    }
                                }
                                pre_data=data;
                            }
                            else
                            {
                                pl_rs485_rx[index]->rs485_rx_sta_head=FALSE;
                                pl_rs485_rx[index]->rs485_rx_sta_tail=FALSE;
                                pl_rs485_rx[index]->rs485_rx_indx=0;   
                                //slave_rs485_run_state=MASTER_RS485_RUN_TX;
                                //uart1_rxBuf=(uart_circularBuf_T){ {0}, 0, 0, 0, FALSE };  
                                memset(pl_rx_buf[index], 0, sl_sizeof_rx_buf[index]);
                            }
                        }
                        
                        if(time_of<15)
                        {
                            time_of++;
                        }
                        else
                        {
                            time_of=0;
                            break;
                        }
                    }    
                    
                break;                
                default:
                    slave_rs485_run_state[index]=MASTER_RS485_RUN_TX;
                    //设置发送超时计数值，超时就不再发送数据
                    pl_rs485_tx[index]->rs485_tx_buf_busy=FALSE;
                break;
            }
        break;
        case MASTER_RS485_SLEEP:
            slave_rs485_TX_enable(index, FALSE);
            ////slave_rs485_power_en(FALSE); 
            ////GPIO_PinModeSet(gpioPortC, 0, gpioModeWiredAnd, 1);
        break;
        default:

        break;
    }
}
void master_rs485_communication(rs485_state_t rs485_state)
{
    slave_rs485_communication(ENUM_COMM_INDEX_MASTER, rs485_state);
}
//轮询发送命令
void master_rs485_cmd_poll_process(void)
{
    ENUM_MASTER_CMD master_cmd;
    unsigned char *data;
        
    if(sl_master_state.cmd_snd_faild_flag)
    {        
        //超时处理
        //获取当前处理的命令，并进入对应的处理回调
        if(cmd_get_from_list(&master_cmd, &data))
        {
            switch(master_cmd)
            {
                case ENUM_M_CMD_INFO:
//                    sl_master_state.chg_485_cmd_poll_faild_cnt++; // = 1;
                    break;
                default:
                    break;
            }
        }
        //删除链表中的命令
        cmd_delet_one_from_list();
        sl_master_state.cmd_processing_flag = 0;
        sl_master_state.next_cmd_interval_cnt = MASTER_NEXT_CMD_INTERVAL_CNT_MAX;
        sl_master_state.cmd_snd_faild_flag = 0;
    }
    //发送链表中的命令
    if((sl_master_state.cmd_processing_flag == 0)&&
        (cmd_get_from_list(&master_cmd, &data))&&
        (0 == sl_master_state.next_cmd_interval_cnt))
    {
        sl_master_state.cmd_processing_flag = 1;
        sl_master_state.cmd_timeout_cnt = 1000;
        sl_master_state.cmd_snd_faild_flag = 0;

        switch(master_cmd)
        {
            case ENUM_M_CMD_INFO:
				Motor_Controller_Info_req();
                break;
            case ENUM_M_CMD_STATUS:
				Motor_Controller_State_req();
                break;			
            default:
                break;
        }
        
    }
    
}
void master_rs485_process(void)
{
    master_rs485_communication(MASTER_RS485_RUN);
    master_rs485_cmd_poll_process();
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
	
    gl_rs485_param.usart_dev = rt_device_find(RS485_USART_DEV_NAME);
    rt_device_open(gl_rs485_param.usart_dev, RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_TX);
    rt_device_control(gl_rs485_param.usart_dev, RT_DEVICE_CTRL_CONFIG, &gl_rs485_param.usart_config_param);
    rt_device_set_tx_complete(gl_rs485_param.usart_dev, master_rs485_charger_tx_done_cb);
    rt_device_set_rx_indicate(gl_rs485_param.usart_dev, master_rs485_charger_input_cb);

    rt_sem_init(&gl_rs485_param.usart_rx_sem, "RS485_SEM", 0, RT_IPC_FLAG_FIFO);	
}

void thread_rs485_entry(void* parameter)
{
    rt_tick_t vl_delay_tick_cnt;
    master_rs485_init();
    master_rs485_communication(MASTER_RS485_RESET);

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
//    slave_rs485_comm_TX_delay_cnt_max = SLAVE_RS485_COMM_TX_DELAY_CNT_MAX_LONG;
    slave_rs485_run_state[ENUM_COMM_INDEX_COMM]=MASTER_RS485_RUN_TX;
}
