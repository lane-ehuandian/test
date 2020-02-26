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
#include "fm175xx.h"
#include "nfc_cmd_process.h"
#include "battery_process.h"
#include "xmodem.h"
#include "ntag_app.h"
#include "mis.h"

#include "motor_control.h"
#include "master_rs485_list.h"

//#define UNMASK_CODE

extern unsigned int gl_delay_cnt_test;

#define BAT1_READY_MASK 0X01
#define BAT2_READY_MASK 0X02
#define BAT3_READY_MASK 0X04
#define BAT4_READY_MASK 0X08
#define BAT_CHG_SWITCH_MASK 0X01
#define BAT_DISCHG_SWITCH_MASK 0X02
#define BAT_PRECHG_SWITCH_MASK 0X04
#define BAT_PREDISCHG_SWITCH_MASK 0X08
    

#define RS485_FRAME_INDEX_TX_ADDR 0U
#define RS485_FRAME_INDEX_RX_ADDR 1U
#define RS485_FRAME_INDEX_CMD 2U
#define RS485_FRAME_INDEX_DATA_LEN 3U
#define RS485_FRAME_INDEX_DATA 4U

#define RS485_FRAME_BAT1_ADDR 0x03
#define RS485_FRAME_BAT2_ADDR 0x04
#define RS485_FRAME_PMS_ADDR 0x00

#define RS485_FRAME_MASTER_ADDR 0xC0
#define RS485_FRAME_SLAVE_ADDR 0x40

//master RS485 general command
#define MASTER_GENERAL_CMD 0x01
#define MASTER_GENERAL_CMD_INDEX_DATA1 0U
#define MASTER_GENERAL_CMD_LEN 2U

//master RS485 general command
#define MASTER_GENERAL_CMD_RSP 0x01
#define MASTER_GENERAL_CMD_RSP_INDEX_ID 0U
#define MASTER_GENERAL_CMD_RSP_INDEX_SOC 4U
#define MASTER_GENERAL_CMD_RSP_INDEX_VOLTAGE 5U
#define MASTER_GENERAL_CMD_RSP_INDEX_TEMPERATURE 7U
#define MASTER_GENERAL_CMD_RSP_INDEX_CURRENT 8U
#define MASTER_GENERAL_CMD_RSP_INDEX_CYCLE 10U
#define MASTER_GENERAL_CMD_RSP_INDEX_MOS_STA 21U
#define MASTER_GENERAL_CMD_RSP_INDEX_FAULT 22U
#define MASTER_GENERAL_CMD_RSP_INDEX_WARNING 25U

#define BAT_FAULT_CHG_OVP (1U<0)
#define BAT_FAULT_CHG_OFF (1U<1)
#define BAT_FAULT_CHG_UTP (1U<2)
#define BAT_FAULT_CHG_OTP (1U<3)
#define BAT_FAULT_CHG_OCP (1U<4)
#define BAT_FAULT_DISCHG_UVP (1U<8)
#define BAT_FAULT_DISCHG_UTP (1U<9)
#define BAT_FAULT_DISCHG_OTP (1U<10)
#define BAT_FAULT_DISCHG_OCP1 (1U<11)
#define BAT_FAULT_DISCHG_OCP2 (1U<12)
#define BAT_FAULT_INTERNAL_ERR (1U<16)
#define BAT_FAULT_CHG_MOS (1U<17)
#define BAT_FAULT_DISCHG_MOS (1U<18)
#define BAT_FAULT_CELL_DEAD (1U<19)
#define BAT_FAULT_MOS_OTP (1U<20)
#define BAT_FAULT_SCP (1U<21)

#define PMS_FAULT_OVP (1U<0)
#define PMS_FAULT_UVP (1U<1)
#define PMS_FAULT_OCP (1U<2)
#define PMS_FAULT_OTP (1U<3)
#define PMS_FAULT_UTP (1U<4)
#define PMS_FAULT_OTHERS (1U<5)

#define RS485_FRAM_CMD 0x01
#define RS485_FRAM_CRC_L 0x00
#define RS485_FRAM_CRC_H 0x00
#define RS485_FRAM_LENGTH  0X02
#define RS485_FRAM_TOTAL_LENGTH (RS485_FRAM_LENGTH+8) 
#define RS485_FRAM_RX_ADDR_INDEX 0X02 
#define RS485_FRAM_CRC_INDEX (RS485_FRAM_TOTAL_LENGTH-3) 
#define RS485_BAT_FRAM_RSP_ALERT_INDEX 27 
#define RS485_BAT_SWITCH_STA_INDEX 26 

#define SLAVE_PACK_RS485_ADDR 0x40
#define SLAVE_SYSTEM_RS485_ADDR 0xC0
#define SLAVE_RS485_RSP_DATA_LEN 0X2D

#define SP_BAT_CMD_DISCHG_MOS (1U<<0)
#define SP_BAT_CMD_CHG_MOS (1U<<1)
#define SP_BAT_CMD_PREDISCHG_MOS (1U<<2)
#define SP_BAT_CMD_PRECHG_MOS (1U<<3)
#define SP_BAT_CMD_BALANCE (1U<<4)
#define SP_BAT_CMD_SLEEP (1U<<5)
#define SP_BAT_CMD_SHUT_DOWN (1U<<6)
#define SP_BAT_CMD_CHARGING (1U<<7)

#define SP_BAT_STA_CHG_MOS (1U<<0)
#define SP_BAT_STA_DISCHG_MOS (1U<<1)
#define SP_BAT_STA_PRECHG_MOS (1U<<2)
#define SP_BAT_STA_PREDISCHG_MOS (1U<<3)
#define SP_BAT_STA_BALANCE (1U<<4)
#define SP_BAT_STA_SLEEP (1U<<5)
#define SP_BAT_STA_SHUT_DOWN (1U<<6)
#define SP_BAT_STA_CHARGING (1U<<7)
#define SP_BAT_STA_ON_SITE_CNT_MAX 3U





















//extern uint8_t usart0_rx_buf[BUFFERSIZE], usart0_tx_buf[BUFFERSIZE];
uint8_t usart1_rx_buf[BUFFERSIZE_OF_COMM_UART], usart1_tx_buf[BUFFERSIZE];




static unsigned char slave_rs485_changed_bms_index_to_comm(unsigned char bms_index);

st_charger_param gl_rs485_param;
////////////////////////////////////////////////////////////////////////////////////////////////////
//��ֲ����ϵͳ�������Խ�


void drv_usart1_enable(ENUM_USART_EN_TYPE enable_type)
{
    
}

void drv_usart1_init(uint8_t *p_tx_buf, uint16_t tx_buf_size,uint8_t *p_rx_buf, uint16_t rx_buf_size)
{
    
}

uint32_t drv_usart1_put_byte(uint8_t data)
{
    return 1;
}

void drv_usart1_flush_tx_fifo(void)
{
    
}

bool drv_usart1_tx_busy(void)
{
    return FALSE;
}

uint32_t drv_usart1_get_byte(uint8_t *data)
{
    return 0;
}



void drv_usart0_init(uint8_t *p_tx_buf, uint16_t tx_buf_size,uint8_t *p_rx_buf, uint16_t rx_buf_size)
{
    
}





////////////////////////////////////////////////////////////////////////////////////////////////////




/*****************************************************************************
 * @brief
 *   handle slave rs485 communication
 *
 * @details
 *   
 *
 * @param[in] none
 *   
 *
 * @param[out] none
 *
 *
 * @return
 *   none
 ******************************************************************************/


bool slave_rs485_cmd_get_dev_version(st_hub_cmd *pl_hub_cmd);
void slave_canopen_cmd_unlock(void);

void slave_canopen_cmd_charge_en(unsigned char charge_flag);
bool slave_rs485_cmd_charge_en(st_hub_cmd *pl_hub_cmd);

bool slave_rs485_cmd_check_lock_port_chg(st_hub_cmd *pl_hub_cmd);
bool slave_rs485_cmd_get_bat_record_param(st_hub_cmd *pl_hub_cmd);
bool slave_rs485_cmd_check_bat(st_hub_cmd *pl_hub_cmd);
bool slave_rs485_cmd_discharge_en(st_hub_cmd *pl_hub_cmd);
bool slave_rs485_cmd_fan_enable(st_hub_cmd *pl_hub_cmd);
bool slave_rs485_cmd_fan_disable(st_hub_cmd *pl_hub_cmd);
bool slave_rs485_cmd_update_start(st_hub_cmd *pl_hub_cmd);
bool slave_rs485_cmd_update_exchange(st_hub_cmd *pl_hub_cmd);
bool slave_rs485_cmd_update_done(st_hub_cmd *pl_hub_cmd);
bool slave_rs485_cmd_cfg(st_hub_cmd *pl_hub_cmd);
bool slave_rs485_cmd_read_bat_record(st_hub_cmd *pl_hub_cmd);
bool slave_rs485_cmd_bypass(st_hub_cmd *pl_hub_cmd);
bool slave_rs485_cmd_set_SN(st_hub_cmd *pl_hub_cmd);
bool slave_rs485_cmd_fan_temp_setting(st_hub_cmd *pl_hub_cmd);
bool slave_rs485_cmd_read_runing_param(st_hub_cmd *pl_hub_cmd);
bool slave_rs485_cmd_read_real_time_info(st_hub_cmd *pl_hub_cmd);
bool slave_rs485_cmd_read_BAT_info(st_hub_cmd *pl_hub_cmd);
bool slave_rs485_cmd_read_BAT_info_when_critical_err(st_hub_cmd *pl_hub_cmd);
bool slave_rs485_cmd_read_BAT_info_when_even_to_stop(st_hub_cmd *pl_hub_cmd);
bool slave_rs485_cmd_read_BAT_info_when_detect_bat_error(st_hub_cmd *pl_hub_cmd);
bool slave_rs485_cmd_read_ADC_raw_value(st_hub_cmd *pl_hub_cmd);
bool slave_rs485_cmd_clear_pms_error_flag(st_hub_cmd *pl_hub_cmd);
bool slave_rs485_cmd_pms_reset(st_hub_cmd *pl_hub_cmd);
#ifdef TEST_DETECT_BAT_ERROR_EN
bool slave_rs485_cmd_detect_bat_error(st_hub_cmd *pl_hub_cmd);
#endif
#ifdef TEST_TEST_CONDITION_CREATE_ENABLE
bool slave_rs485_cmd_test_test(st_hub_cmd *pl_hub_cmd);
#endif

#ifdef BAT_PMS_AUTHORITY_FUNCTION_EN
bool slave_rs485_cmd_pms_bat_authority(st_hub_cmd *pl_hub_cmd);
#endif

bool slave_rs485_cmd_read_charger_temp_value(st_hub_cmd *pl_hub_cmd);

#ifdef EVENT_TO_STOP_CHARGING_ENABLE
bool slave_rs485_cmd_event_to_stop_charging_flag_be_disable_flag(st_hub_cmd *pl_hub_cmd);
bool slave_rs485_cmd_event_to_stop_charging_flag_be_disable_flag_clear(st_hub_cmd *pl_hub_cmd);
#endif
#ifdef CABINET_WARMOR_SUPPLY_BY_CHARGER
bool slave_rs485_cmd_charger_warmer_enable(st_hub_cmd *pl_hub_cmd);
bool slave_rs485_cmd_charger_warmer_test_enable(st_hub_cmd *pl_hub_cmd);
#endif










/*****************************************************************************
 * @brief
 *   handle slave rs485 communication
 *
 * @details
 *   
 *
 * @param[in] none
 *   
 *
 * @param[out] none
 *
 *
 * @return
 *   none
 ******************************************************************************/
/*
�ȵ�������ɺ��Ƿ���Ҫ������
��bit0��ʼ������enum_comm_index�е�˳��ÿ��λ����һ��ͨ��

*/
static unsigned char slave_rs485_reboot_after_tx; 




void slave_rs485_cmd_timer_cb(void)
{
//    unsigned char i;
//    for(i = 0; i < ENUM_COMM_INDEX_MAX; i++)
//    if(sl_slave_usart_reenable_count[i])
//    {
//        sl_slave_usart_reenable_count[i]--;
//        if(!sl_slave_usart_reenable_count[i])
//            slave_rs485_usart_enable[i](ENUM_USART_RX_TX_ENABLE);
//    }

//    if(slave_rs485_comm_TX_delay_cnt_max)
//        slave_rs485_comm_TX_delay_cnt_max--;
}


void slave_rs485_dev_addr_init(void)
{
    unsigned char vl_result;

    #ifdef UNMASK_CODE
    //RS485��ַѡ�������
    GPIO_PinModeSet(RS485_DEV_ADDR_BIT3_PORT, RS485_DEV_ADDR_BIT3_PIN, gpioModeInput, 1);
    GPIO_PinModeSet(RS485_DEV_ADDR_BIT2_PORT, RS485_DEV_ADDR_BIT2_PIN, gpioModeInput, 1);
    GPIO_PinModeSet(RS485_DEV_ADDR_BIT1_PORT, RS485_DEV_ADDR_BIT1_PIN, gpioModeInput, 1);
    GPIO_PinModeSet(RS485_DEV_ADDR_BIT0_PORT, RS485_DEV_ADDR_BIT0_PIN, gpioModeInput, 1);

    vl_result = 0;
    vl_result |= GPIO_PinInGet(RS485_DEV_ADDR_BIT3_PORT, RS485_DEV_ADDR_BIT3_PIN);
    vl_result = (vl_result<<1);
    vl_result |= GPIO_PinInGet(RS485_DEV_ADDR_BIT2_PORT, RS485_DEV_ADDR_BIT2_PIN);
    vl_result = (vl_result<<1);
    vl_result |= GPIO_PinInGet(RS485_DEV_ADDR_BIT1_PORT, RS485_DEV_ADDR_BIT1_PIN);
    vl_result = (vl_result<<1);
    vl_result |= GPIO_PinInGet(RS485_DEV_ADDR_BIT0_PORT, RS485_DEV_ADDR_BIT0_PIN);
    #endif
    //û�е�ַ0�������1��ʼ
    if(vl_result == 0)
        vl_result = 0xFF;

//    gl_RS485_dev_addr = vl_result;
}










/*
RSP	1
Data L	1
Pack available output current	2
Pack Voltage	2
Pack Current	2
Pack state	1


Battery1 SOC	1
Battery1 voltage	2
Battery1 current	2
Battery1 temperature	1
Battery1 fault	1
Battery1 cycle	2
Battery1 ID	4
Battery1 nominal voltage	2
Battery1 nominal current	2
Battery1 design capacity	2
*/





/*
Pack available output current	2
Pack Voltage	2
Pack Current	2
Pack state	1
Bat Data number	1
Bat Data size	1




Port0	1
Bat 0 SOC	1
Bat 0 voltage	2
Bat 0 current	2
Bat 0 temperature	1
Bat 0 damage	1
Bat 0 fault	1
Bat 0 cycle	2
Bat 0 ID	4
Bat 0 nominal voltage	2
Bat 0 nominal current	2
Bat 0 design capacity	2
*/
#pragma pack(1)
    
    typedef struct{
    unsigned char stl_ID[6];
    unsigned char stl_port;
    unsigned char stl_SOC; // ��λ�ٷֱ�%
    unsigned char stl_V[2]; //10mV
    unsigned char stl_A[2]; //10mA
    unsigned char stl_temperature;
    unsigned char stl_fault;
    unsigned char stl_damage;
    unsigned char stl_cycle[2];
    
    unsigned char stl_nominal_V[2]; //10mV
    unsigned char stl_nominal_A[2]; //10mA //�пض���Ϊ���ŵ����
    unsigned char stl_design_cap[2]; // 100mAh // �пض���Ϊ�������
    //2019-3-6
    unsigned char stl_bat_cell_voltage_max[2];//mV//����о��ѹ
    unsigned char stl_bat_cell_voltage_min[2];//mV//��С��о��ѹ
    unsigned char stl_bat_cell_max_voltage_index;//����ѹ��о���
    unsigned char stl_bat_cell_min_voltage_index;//��С��ѹ��о���
    unsigned char stl_bat_pcb_temp; //���BMS PCB���¶�
    unsigned char stl_bat_connector_temp; //����������¶�
    unsigned char stl_bat_MOS_state; // ���ӵ�״̬
    }st_bat_info_data;
    
    typedef struct{
    unsigned char stl_RSP_code;
    unsigned char stl_dataL;
    unsigned char stl_available_output_A[2]; //10mA
    unsigned char stl_V[2]; //10mV
    unsigned char stl_A[2]; //10mA
    unsigned char stl_state;
    unsigned char stl_bat_number;
    unsigned char stl_info_data_size;
    //st_bat_info_data *stl_bat_info_data;
    }st_check_bat_info_rsp_head;


    typedef struct {
    unsigned char stl_pms_tmep[2];
    unsigned char stl_charger_state1;
    unsigned char stl_charger_state2;
    unsigned char stl_active_detected_bat_err;
    unsigned char stl_detcted_bat_err[4];
    unsigned char stl_pms_bat_other_err;
    unsigned char stl_charger_temp;
    }st_hb_pms_data; // heart beat on pms data

    typedef struct{
    unsigned char stl_port;
    unsigned char stl_SOC; // ��λ�ٷֱ�%
    unsigned char stl_V[2]; //10mV
    unsigned char stl_A[2]; //10mA
    unsigned char stl_temperature;
    unsigned char stl_fault;
    unsigned char stl_damage;
    unsigned char stl_cycle[2];
    unsigned char stl_ID[6];
    unsigned char stl_nominal_V[2]; //10mV
    unsigned char stl_nominal_A[2]; //10mA
    unsigned char stl_design_cap[2]; // 100mAh
    unsigned char stl_Fuel_Gauge_Temp;
    unsigned char stl_bat_Temp_1;
    unsigned char stl_chg_mos_Temp;
    unsigned char stl_connector_Temp;
    unsigned char stl_Dev_Fault1[2];
    unsigned char stl_Dev_Fault2[2];
    unsigned char stl_Operation_Fault1[2];
    unsigned char stl_Operation_Fault2[2];
    unsigned char stl_bms_state[2];
    unsigned char stl_bat0_voltage[2];
    unsigned char stl_bat1_voltage[2];
    unsigned char stl_bat2_voltage[2];
    unsigned char stl_bat3_voltage[2];
    unsigned char stl_bat4_voltage[2];
    unsigned char stl_bat5_voltage[2];
    unsigned char stl_bat6_voltage[2];
    unsigned char stl_bat7_voltage[2];
    unsigned char stl_bat8_voltage[2];
    unsigned char stl_bat9_voltage[2];
    unsigned char stl_bat10_voltage[2];
    unsigned char stl_bat11_voltage[2];
    unsigned char stl_bat12_voltage[2];
    unsigned char stl_bat13_voltage[2];
    unsigned char stl_bat14_voltage[2];
    unsigned char stl_bat15_voltage[2];
    unsigned char stl_TVS_Temp;
    unsigned char stl_max_voltage_rate[2];
    unsigned char stl_max_voltage_rate_index;
    unsigned char stl_max_temp_rate[2];
    unsigned char stl_max_temp_rate_index;
    }st_hb_bat_data; // heart beat on battery data
    
#pragma pack()


    
static void slave_rs485_get_total_valid_output_A(unsigned char* output_A)
{
    st_modbus_reg_unit vl_tmp;
    unsigned short vl_tmp_value;
    
    vl_tmp.bits16_H = 0;
    vl_tmp.bits16_L = 0;
    if(gl_bms_info_p[0]->reg_value_ready)
    {
        vl_tmp = gl_bms_info_p[0]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_BCAP)];
    }
    if((0xFF == vl_tmp.bits16_L)&&(0xFF == vl_tmp.bits16_H))
    {
        vl_tmp_value = 0;
    }
    else
    {
        vl_tmp_value = ((vl_tmp.bits16_H<<8)+vl_tmp.bits16_L);
    }
    
    vl_tmp.bits16_H = 0;
    vl_tmp.bits16_L = 0;
    if(gl_bms_info_p[1]->reg_value_ready)
    {
        vl_tmp = gl_bms_info_p[1]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_BCAP)];
    }

    if((0xFF == vl_tmp.bits16_L)&&(0xFF == vl_tmp.bits16_H))
    {
        vl_tmp_value += 0;
    }
    else
    {
        vl_tmp_value += ((vl_tmp.bits16_H<<8)+vl_tmp.bits16_L);
    }
    vl_tmp_value /=10;
    vl_tmp_value *=3;

    output_A[0] = vl_tmp_value&0xFF;
    output_A[1] = (vl_tmp_value>>8)&0xFF;

}

static void slave_rs485_get_total_current_output_V(unsigned char* output_V)
{
    st_modbus_reg_unit vl_tmp;
    unsigned short vl_tmp_value;
    
    vl_tmp.bits16_H = 0;
    vl_tmp.bits16_L = 0;
    if(gl_bms_info_p[0]->reg_value_ready)
    {
        vl_tmp = gl_bms_info_p[0]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_TVOLT)];
        
    }
    else if(gl_bms_info_p[1]->reg_value_ready)
    {
        vl_tmp = gl_bms_info_p[1]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_TVOLT)];
    }

    if((0xFF == vl_tmp.bits16_L)&&(0xFF == vl_tmp.bits16_H))
    {
        vl_tmp_value = 0;
    }
    else
    {
        vl_tmp_value = ((vl_tmp.bits16_H<<8)+vl_tmp.bits16_L);
    }
    output_V[0] = vl_tmp_value&0xFF;
    output_V[1] = (vl_tmp_value>>8)&0xFF;

    return;


}
#if 0
static void slave_rs485_get_total_current_output_A(unsigned char* output_A)
{
    st_modbus_reg_unit vl_tmp;
    unsigned short vl_tmp_value;
    
    vl_tmp.bits16_H = 0;
    vl_tmp.bits16_L = 0;
    if(gl_bms_info_p[0]->reg_value_ready)
    {
        vl_tmp = gl_bms_info_p[0]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_TCURR)];
        
    }
    vl_tmp_value = ((vl_tmp.bits16_H<<8)+vl_tmp.bits16_L);

    vl_tmp.bits16_H = 0;
    vl_tmp.bits16_L = 0;
    if(gl_bms_info_p[1]->reg_value_ready)
    {
        vl_tmp = gl_bms_info_p[1]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_TCURR)];
    }

    vl_tmp_value += ((vl_tmp.bits16_H<<8)+vl_tmp.bits16_L);
        
    output_A[0] = vl_tmp_value&0xFF;
    output_A[1] = (vl_tmp_value>>8)&0xFF;

    return;


}
#endif
static unsigned char slave_rs485_get_valid_bat_number(void)
{
    unsigned char i;
    i = 0;
    if(gl_bms_info_p[0]->reg_value_ready)
    {
        i++;
    }
    if(gl_bms_info_p[1]->reg_value_ready)
    {
        i++;
    }
    return i;


}

static void slave_rs485_get_v_reg_value(unsigned char bms_index, unsigned char buf[2], unsigned short pl_reg_addr)
{
    st_modbus_reg_unit vl_tmp;
    vl_tmp = gl_bms_info_p[bms_index]->reg_unit[MM_get_reg_addr_position_in_table(pl_reg_addr)];
    buf[0] = vl_tmp.bits16_L;
    buf[1] = vl_tmp.bits16_H;
}

unsigned int gl_test_cnt_test = 0;

//return: 0, ��ȡʧ�ܣ�1,��ȡ�ɹ�
static unsigned char slave_rs485_get_bat_info_data(unsigned char bms_index, st_bat_info_data *pl_bat_info_data)
{
    st_modbus_reg_unit vl_tmp;
    unsigned short vl_tmp_value;
    unsigned short vl_pos;
    
    //if(gl_bms_info_p[bms_index]->reg_value_ready)
    if(
        
        #ifdef PMS_CASE_CHECK_BAT_NEW
        (gl_check_bat.is_bat_in[bms_index]&&(0 == gl_check_bat.is_hall_in_bat[bms_index]))
        ||
        #endif
        
        #ifdef PMS_CASE_WITH_HALL_SWITCH_EN            
        ((sl_bms_had_enable_flag&(1<<bms_index))&&(0 == bms_index)&&(0 == (gl_hall_gpio_state&0x01)))//������Ч���������ؼ�⵽���
        ||((sl_bms_had_enable_flag&(1<<bms_index))&&(1 == bms_index)&&(0 == (gl_hall_gpio_state&0x02)))
        ||
        #endif
        (
        #ifdef PMS_CASE_WITH_HALL_SWITCH_EN            
        (((0 == bms_index)&&(0 == (gl_hall_gpio_state&0x01)))//������Ч���������ؼ�⵽���
        ||((1 == bms_index)&&(0 == (gl_hall_gpio_state&0x02))))
        &&
        //||((sl_bms_had_enable_flag&(1<i))&&(0 == i)&&(0 == GPIO_PinInGet(HALL_SWITCH_BMS0_PORT, HALL_SWITCH_BMS0_PIN)))//������Ч���������ؼ�⵽���
        //||((sl_bms_had_enable_flag&(1<i))&&(1 == i)&&(0 == GPIO_PinInGet(HALL_SWITCH_BMS1_PORT, HALL_SWITCH_BMS1_PIN)))
        #endif
    ((BMS_READONLY_DATA1_FLAG_BIT|BMS_READONLY_DATA2_FLAG_BIT) & Battery_get_reg_valid_flag(bms_index))))
    {
        pl_bat_info_data->stl_port = bms_index;
                 
        vl_tmp = gl_bms_info_p[bms_index]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_SOC)]; // ��λ�ٷֱ�%
        vl_tmp_value = ((vl_tmp.bits16_H<<8)+vl_tmp.bits16_L);
        if(vl_tmp_value == 0xFFFF)
        {
            pl_bat_info_data->stl_SOC = 0xFF;
        }
        else
        {
            pl_bat_info_data->stl_SOC = vl_tmp_value/10;
        }
        
        vl_tmp = gl_bms_info_p[bms_index]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_TVOLT)]; // 10mv
        pl_bat_info_data->stl_V[0] = vl_tmp.bits16_L;
        pl_bat_info_data->stl_V[1] = vl_tmp.bits16_H;

        vl_tmp = gl_bms_info_p[bms_index]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_TCURR)]; // 10mv
        pl_bat_info_data->stl_A[0] = vl_tmp.bits16_L;
        pl_bat_info_data->stl_A[1] = vl_tmp.bits16_H;

        vl_tmp = gl_bms_info_p[bms_index]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_HTEMP)]; // ��λ�ٷֱ�%
        vl_tmp_value = ((vl_tmp.bits16_H<<8)+vl_tmp.bits16_L);
        if(0xFFFF == vl_tmp_value)
        {
            pl_bat_info_data->stl_temperature = 0xFF;
        }
        else
        {
            pl_bat_info_data->stl_temperature = vl_tmp_value/10;
        }
        
        /*
                
                Bit      Bit7~ Bit6     Bit5             Bit4    Bit3    Bit2    Bit1    Bit0
                Fault   Reserved    Other Fault    UTP    OTP    OCP    UVP    OVP
                */
         //�������Ҫ�ж�Ŀǰ�Ǵ��ڳ��״̬���Ƿŵ�״̬�����������Ȳ��ж�
        vl_pos = MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_OPFT1);
        vl_tmp = gl_bms_info_p[bms_index]->reg_unit[vl_pos]; // ���й�����1
        vl_tmp_value = ((vl_tmp.bits16_H<<8)+vl_tmp.bits16_L);
        pl_bat_info_data->stl_fault = 0;
        if(vl_tmp_value&(1<<0))
        {
            //��ѹ
            pl_bat_info_data->stl_fault |= (1<<0);
        }

        if(vl_tmp_value&((1<<1)|(1<<2)))
        {
            //Ƿѹ
            pl_bat_info_data->stl_fault |= (1<<1);
        }

        if(vl_tmp_value&((1<<3)|((1<<4)|(1<<5))))
        {
            //����
            pl_bat_info_data->stl_fault |= (1<<2);
        }

        if(vl_tmp_value &((1<<6)|(1<<7)|(1<<10)|(1<<11)|(1<<12)))
        {
            //����
            pl_bat_info_data->stl_fault |= (1<<3);
        }

        if(vl_tmp_value &((1<<8)|(1<<9)))
        {
            //����
            pl_bat_info_data->stl_fault |= (1<<4);
        }
        vl_pos = MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_DEVFT1);
        if((vl_tmp_value &((1<<14)|(1<<15)))||
            (gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_L != 0)||
            (gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_H != 0)||
            (gl_bms_info_p[bms_index]->reg_unit[vl_pos+1].bits16_L != 0)||
            (gl_bms_info_p[bms_index]->reg_unit[vl_pos+1].bits16_H != 0)||
            (gl_bms_info_p[bms_index]->reg_unit[vl_pos+3].bits16_L != 0)||
            (gl_bms_info_p[bms_index]->reg_unit[vl_pos+3].bits16_H != 0))
        {
            //����
            pl_bat_info_data->stl_fault |= (1<<5);
        }

        vl_tmp = gl_bms_info_p[bms_index]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_LTSTA)];
        pl_bat_info_data->stl_damage = ((vl_tmp.bits16_L<<1)&0x02)|((vl_tmp.bits16_L>>1)&0x01);

        vl_tmp = gl_bms_info_p[bms_index]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_CYCLE)];
        pl_bat_info_data->stl_cycle[0] = vl_tmp.bits16_L;
        pl_bat_info_data->stl_cycle[1] = vl_tmp.bits16_H;

        vl_tmp = gl_bms_info_p[bms_index]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_SN34)];
        pl_bat_info_data->stl_ID[0] = vl_tmp.bits16_H;
        pl_bat_info_data->stl_ID[1] = vl_tmp.bits16_L;
                
        vl_tmp = gl_bms_info_p[bms_index]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_SN56)];
        pl_bat_info_data->stl_ID[2] = vl_tmp.bits16_H;
        pl_bat_info_data->stl_ID[3] = vl_tmp.bits16_L;

        vl_tmp = gl_bms_info_p[bms_index]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_SN78)];
        pl_bat_info_data->stl_ID[4] = vl_tmp.bits16_H;
        pl_bat_info_data->stl_ID[5] = vl_tmp.bits16_L;

        vl_tmp = gl_bms_info_p[bms_index]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_BVOLT)];
        
        pl_bat_info_data->stl_nominal_V[0] = vl_tmp.bits16_L; //10mV
        pl_bat_info_data->stl_nominal_V[1] = vl_tmp.bits16_H; //10mV

        vl_tmp = gl_bms_info_p[bms_index]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_DSOP)]; 
        pl_bat_info_data->stl_nominal_A[0] = vl_tmp.bits16_L; //10mA
        pl_bat_info_data->stl_nominal_A[1] = vl_tmp.bits16_H; //10mA

        vl_tmp = gl_bms_info_p[bms_index]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_BCAP)]; 
        vl_tmp_value = (vl_tmp.bits16_L + (vl_tmp.bits16_H << 8));
        vl_tmp_value /= 100;
        pl_bat_info_data->stl_design_cap[0] = vl_tmp_value&0xff; //10mA
        pl_bat_info_data->stl_design_cap[1] = ((vl_tmp_value>>8)&0xff); //10mA

        vl_tmp = gl_bms_info_p[bms_index]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_HVOLT)]; 
        pl_bat_info_data->stl_bat_cell_voltage_max[0] = vl_tmp.bits16_L; //mV
        pl_bat_info_data->stl_bat_cell_voltage_max[1] = vl_tmp.bits16_H; //mV

        vl_tmp = gl_bms_info_p[bms_index]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_LVOLT)]; 
        pl_bat_info_data->stl_bat_cell_voltage_min[0] = vl_tmp.bits16_L; //mV
        pl_bat_info_data->stl_bat_cell_voltage_min[1] = vl_tmp.bits16_H; //mV

        vl_tmp = gl_bms_info_p[bms_index]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_HVNUM)]; 
        pl_bat_info_data->stl_bat_cell_max_voltage_index = vl_tmp.bits16_L;
        
        vl_tmp = gl_bms_info_p[bms_index]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_LVNUM)]; 
        pl_bat_info_data->stl_bat_cell_min_voltage_index = vl_tmp.bits16_L;

        vl_tmp = gl_bms_info_p[bms_index]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_DMOST)]; 
        vl_tmp_value = (vl_tmp.bits16_L + (vl_tmp.bits16_H << 8));
        if(0xFFFF == vl_tmp_value)
        {
            pl_bat_info_data->stl_bat_pcb_temp = 0xFF;
        }
        else
        {
            pl_bat_info_data->stl_bat_pcb_temp = vl_tmp_value/10;
        }

        vl_tmp = gl_bms_info_p[bms_index]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_CONT)]; 
        vl_tmp_value = (vl_tmp.bits16_L + (vl_tmp.bits16_H << 8));
        if(0xFFFF == vl_tmp_value)
        {
            pl_bat_info_data->stl_bat_connector_temp = 0xFF;
        }
        else
        {
            pl_bat_info_data->stl_bat_connector_temp = vl_tmp_value/10;
        }

        vl_tmp_value = Battery_get_switch_state(bms_index);
        pl_bat_info_data->stl_bat_MOS_state = (vl_tmp_value & 0x0003);
        
    }
    else
    {
  
        #ifdef PMS_CASE_WITH_HALL_SWITCH_EN
        unsigned char vl_str_buf[32];
        unsigned char vl_tmp_value;

        strcpy((char*)vl_str_buf, "Bat info status:");
        vl_tmp_value = Battery_get_reg_valid_flag(bms_index);
        str_i2Asiic((const uint8 *) &gl_check_bat.is_bat_in[bms_index],1,(char *)&vl_str_buf[16],16);
        str_i2Asiic((const uint8 *) &gl_check_bat.is_hall_in_bat[bms_index],1,(char *)&vl_str_buf[18],14);
        str_i2Asiic((const uint8 *) &sl_bms_had_enable_flag,1,(char *)&vl_str_buf[20],12);
        str_i2Asiic((const uint8 *) &gl_hall_gpio_state,1,(char *)&vl_str_buf[22],10);
        str_i2Asiic((const uint8 *) &vl_tmp_value,1,(char *)&vl_str_buf[24],8);
        vl_str_buf[26] = '\r';
        vl_str_buf[27] = '\n';
        vl_str_buf[28] = 0;
        DEBUG_PRINTF(vl_str_buf);
        #endif
        return 0;
    }
    return 1;
}

// heart beat on pms data
static unsigned char slave_rs485_hb_get_pms_data(unsigned char bms_index, st_hb_pms_data *pl_hb_pms_data)
{
    //pms temp, 2 bytes
    {
        unsigned int vl_pms_temp;
//        vl_pms_temp = ADC_get_adc_raw_value();
        if(vl_pms_temp > 0xFFFF)
        {
            vl_pms_temp = 0xFFFF;
        }
        pl_hb_pms_data->stl_pms_tmep[0] = (0xFF & vl_pms_temp);
        pl_hb_pms_data->stl_pms_tmep[1] = (0xFF & (vl_pms_temp>>8));
    }
    //�����״̬1
    //�����״̬2
    if(sl_master_state.chg_485_cmd_poll_faild_cnt >= MASTER_485_CMD_POLL_ERROR_CNT_MAX)
    {
        //�����485ͨѶʧ�ܣ�Ĭ���ϴ�����ֵ
        //�����״̬1
        pl_hb_pms_data->stl_charger_state1 = 0xC0;
        //�����״̬2
        pl_hb_pms_data->stl_charger_state2 = 0x00;
    }
    else
    {
        if(gl_charger_err_status.is_need_update)
        {
            gl_charger_err_status.is_need_update = 0;
            //�����״̬1
            pl_hb_pms_data->stl_charger_state1 = gl_charger_err_status.charger_err[0];
            //�����״̬2
            pl_hb_pms_data->stl_charger_state2 = gl_charger_err_status.charger_err[1];
        }
        else
        {
            //�����״̬1
            pl_hb_pms_data->stl_charger_state1 = ((unsigned char*)&gl_poll_cmd_cnf_param)[6];
            //�����״̬2
            pl_hb_pms_data->stl_charger_state2 = ((unsigned char*)&gl_poll_cmd_cnf_param)[7];
        }
    }
    #ifdef TEST_BAT_V_VIA_CHARGER_EN
    //��������ع���
    pl_hb_pms_data->stl_active_detected_bat_err = Battery_check_bat_get_bat_error();
    #endif

    #ifdef EVENT_TO_STOP_CHARGING_ENABLE
    //��������ع���
    {
        unsigned int vl_tmp_int;
        vl_tmp_int = Battery_get_src_of_stop_flag();
        pl_hb_pms_data->stl_detcted_bat_err[0] = ((vl_tmp_int)&0xFF);
        pl_hb_pms_data->stl_detcted_bat_err[1] = ((vl_tmp_int>>8)&0xFF);
        pl_hb_pms_data->stl_detcted_bat_err[2] = ((vl_tmp_int>>16)&0xFF);
        pl_hb_pms_data->stl_detcted_bat_err[3] = ((vl_tmp_int>>24)&0xFF);
    }
    #else
    pl_hb_pms_data->stl_detcted_bat_err[0] = 0;
    pl_hb_pms_data->stl_detcted_bat_err[1] = 0;
    pl_hb_pms_data->stl_detcted_bat_err[2] = 0;
    pl_hb_pms_data->stl_detcted_bat_err[3] = 0;
    #endif
        
    //����������
    {
        unsigned char vl_other_err_tmp;
        vl_other_err_tmp = 0;
        
        #ifdef UNMASK_CODE
        //������ָ��򲻿�����
        if((ELOCK_LOCK == ELock_get_status((ENUM_ELOCK_INDEX)(ELock_bms_index_to_elock_index(bms_index))))&&
            (ENUM_ELOCK_CMD_STATUS_OPEN == ELock_get_cmd_status(ELock_bms_index_to_elock_index(bms_index)))&&
            (FALSE == main_led_is_opening_mode()))
        {
            vl_other_err_tmp |= (1<<0); //bit6������������
        }
        #endif
        
        ////�Ƿ�ʱ����û�е���
//        if(Battery_is_chg_current_false())
//        {
//           vl_other_err_tmp |= (1<<1);
//        }

        //����ģʽ�£���⵽���ȹ���(����������������)
        #ifdef CABINET_WARMOR_SUPPLY_BY_CHARGER
        // ����ģʽ�£���⵽����������������������Ϸ���
        if(master_rs485_is_charger_warmer_err())
        {
            vl_other_err_tmp |= (1<<2);
        }
        #endif

        pl_hb_pms_data->stl_pms_bat_other_err = vl_other_err_tmp;
    }

    //������¶�
    if(sl_master_state.chg_485_cmd_poll_faild_cnt >= MASTER_485_CMD_POLL_ERROR_CNT_MAX)
    {
        //�����485ͨѶʧ�ܣ��ϴ���Чֵ
        pl_hb_pms_data->stl_charger_temp = 0xFF;
    }
    else
    {
        if(gl_poll_cmd_cnf_param.charger_temp > 160)
        {
            pl_hb_pms_data->stl_charger_temp = 160;
        }
        else
        {
            pl_hb_pms_data->stl_charger_temp = gl_poll_cmd_cnf_param.charger_temp;
        }
    }

    return 1;
}


////////////////////////////////////////////////////////////////////////////////////////////

void slave_canopen_cmd_unlock(void)
{
    //ֹͣ��������
    sl_master_state.chger_enable_on_unlock_cmd_flag = 0;
    
//    if(!ELock_get_open_delay_cnt(ENUM_ELOCK_NUM_0))
//    {
//        //masked for delay���ӳٿ���
//        //ELock_open(ELock_bms_index_to_elock_index(vl_bms_index));
////        ELock_open_with_delay(ENUM_ELOCK_NUM_0);
//    }
}



/*
*charge_flag: ������0��ʾ������
*/
void slave_canopen_cmd_charge_en(unsigned char charge_flag)
{
    if(charge_flag)
    {
        //���������������
        sl_master_state.chger_enable_on_unlock_cmd_flag = 1;
//        Battery_chg_set_charge_enable(0);
    }
    else
    {
//        Battery_chg_set_charge_disable(0);
    }
}


bool slave_rs485_cmd_charge_en(st_hub_cmd *pl_hub_cmd)
{
    unsigned char vl_bms_index;
    //unsigned short vl_crc;

    if(pl_hub_cmd->stl_dataL < 1)
    {
        return 0;
    }
    vl_bms_index = pl_hub_cmd->stl_data[0];

    //ת���ַΪʵ�ʰ��ϵĵ�ַ
    vl_bms_index = slave_rs485_changed_bms_index_from_comm(vl_bms_index);
    
    if(vl_bms_index >= NFC_READER_COUNT_MAX)
    {
        return 0;
    }
    //���������������
    sl_master_state.chger_enable_on_unlock_cmd_flag = 1;
    
//    Battery_chg_set_charge_enable(vl_bms_index);

    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_CHARGE_EN;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x00;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 3;

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    return 1;
}
bool slave_rs485_cmd_discharge_en(st_hub_cmd *pl_hub_cmd)
{
    unsigned char vl_bms_index;
    //unsigned short vl_crc;

    if(pl_hub_cmd->stl_dataL < 1)
    {
        return 0;
    }
    vl_bms_index = pl_hub_cmd->stl_data[0];
    //ת���ַΪʵ�ʰ��ϵĵ�ַ
    vl_bms_index = slave_rs485_changed_bms_index_from_comm(vl_bms_index);
    
    if(vl_bms_index >= NFC_READER_COUNT_MAX)
    {
        return 0;
    }
    
//    Battery_chg_set_charge_disable(vl_bms_index);

    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_DISCHARGE_EN;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x00;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 3;

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    return 1;
}

bool slave_rs485_cmd_get_bat_record_param(st_hub_cmd *pl_hub_cmd)
{
    unsigned int vl_int_tmp;
    //unsigned short vl_crc;

    if(pl_hub_cmd->stl_dataL < 4)
    {
        return 0;
    }

    //Target
    if(1 != pl_hub_cmd->stl_data[0])
    {
        return 0;
    }
    
    //Port
    //pl_hub_cmd->stl_data[1];

    //Even type
    if(3 != pl_hub_cmd->stl_data[2])
    {
        return 0;
    }

    //Data len
    //pl_hub_cmd->stl_data[3];

    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_GET_BAT_RECORD_PARAM;

    #ifdef UNMASK_CODE
    if(((BMS_READONLY_DATA1_FLAG_BIT|BMS_READONLY_DATA2_FLAG_BIT)&
            Battery_get_reg_valid_flag(ELock_bms_index_to_elock_index(0)))||
            (gl_elock_param[0].elock_bms_flag&(1<<ELock_bms_index_to_elock_index(0))))
    {
        //�е�أ�����APP״̬
        pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x12;
    }
    else
    {
        pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x04;
    }
    #endif
    
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 3;

    //Target
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = 1;
    
    //Port
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = pl_hub_cmd->stl_data[1];

    //Even type
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = 3;

    slave_rs485_calc_TX_tail();
    return 1;
}



bool slave_rs485_cmd_check_bat(st_hub_cmd *pl_hub_cmd)
{

    unsigned char i, k;
    unsigned char vl_result;
    unsigned char vl_bat_num;
    //unsigned short vl_crc;
    unsigned short vl_bms_reg_value;
    unsigned char vl_bms_reg_valid_flag;


    st_hb_pms_data* pl_hb_pms_data;
    st_hb_bat_data* pl_hb_bat_data;
    
    //if(pl_hub_cmd->stl_dataL < 0)
    //{
    //    return 0;
    //}

    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_CHECK_BAT;
    //pl_rs485_tx->rs485_tx_buf[2] = 
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 3;

    for(i  = 0, vl_result = 0; i < NFC_READER_COUNT_MAX; i++)
    {
        vl_result = (vl_result<<1);
        #ifdef UNMASK_CODE
        vl_result |= ELock_get_status((ENUM_ELOCK_INDEX)(ELock_bms_index_to_elock_index(NFC_READER_COUNT_MAX-1-i)));
        #endif
    }
    // Data	Lock status
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx] = vl_result;
    #ifdef UNMASK_CODE
    //Data Lock status, Bit7���±�����Ϊ������־
    if(main_get_pms_restart_flag())
    {
        pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx] |= (1<<7);
    }
    #endif
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++;
    
    for(i  = 0, vl_result = 0; i < NFC_READER_COUNT_MAX; i++)
    {
        vl_result = (vl_result<<1);
        #ifdef UNMASK_CODE
        if((
            #ifdef CHECKING_BAT_IN_IO_FLAG_ENABLE
            //(
            //(FALSE == Battery_is_checking_mode_on_bat_in_by_HW())|| // ��ʹ��Ӳ�����ؼ��Ļ����������������
            //(TRUE == Battery_is_bat_in_via_IO_switch()))&& // ��ҪӲ����⿪�ؼ�⵽��أ����Ǳ�Ҫ����
            #endif
            #ifdef TEST_BAT_V_VIA_CHARGER_EN
            //(
            //(TRUE == Battery_check_bat_v_is_end()) &&
            //(TRUE == Battery_check_bat_v_is_valid()))&& // �ز��ż�����Ƿ����Ӻ�������
            #endif
            ((BMS_READONLY_DATA1_FLAG_BIT|BMS_READONLY_DATA2_FLAG_BIT)&
            Battery_get_reg_valid_flag(ELock_bms_index_to_elock_index(NFC_READER_COUNT_MAX-1-i))))||
            (gl_elock_param[0].elock_bms_flag&(1<<ELock_bms_index_to_elock_index(NFC_READER_COUNT_MAX-1-i)))||
            (gl_bms_app_bl_status[ELock_bms_index_to_elock_index(NFC_READER_COUNT_MAX-1-i)].is_bl_mode))//����bootloaderģʽ
        {
            vl_result |= 1;
        }
        #endif
    }
////////////////////////////////////////////////////////////////////////////////////////////////////
    //Data	Port status

    #ifdef UNMASK_CODE
    //�����ȡ�����ʷ��¼���ݵ�״̬λ��bit1~bit2
    if((BMS_READONLY_DATA1_FLAG_BIT|BMS_READONLY_DATA2_FLAG_BIT)&
            Battery_get_reg_valid_flag(ELock_bms_index_to_elock_index(0)))
    {
        if(gl_bat_history_record_param.is_earliest_param_valid
            &&gl_bat_history_record_param.is_lastest_param_valid)
        {
            vl_result |= ((Battery_history_record_state_of_process()) << 1);
        }
    }
    else
    {
        //û�е��ʱ�����³�ʼ����ȡ��ʷ��¼�Ĳ���
        Battery_history_record_param_reset();
    }
    #endif
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_result;
    
////////////////////////////////////////////////////////////////////////////////////////////////////

    for(i  = 0, vl_result = 0; i < NFC_READER_COUNT_MAX; i++)
    {
        vl_result = (vl_result<<1);
        vl_bms_reg_value = Battery_get_switch_state(NFC_READER_COUNT_MAX-1-i);
        vl_bms_reg_valid_flag = Battery_get_reg_valid_flag(NFC_READER_COUNT_MAX-1-i);

//        if((ADC_is_CHG_enable())&& // ADC�������¶��Ƿ�������
//            (master_rs485_is_charger_OK())&& // �����û�й���
//            (ELOCK_LOCK == ELock_get_status((ENUM_ELOCK_INDEX)(NFC_READER_COUNT_MAX-1-i)))&& //���ǹرյ�
//            (TRUE == Battery_chg_is_charge_enable(NFC_READER_COUNT_MAX-1-i))&&
        //���ܴ򿪲��������ڳ��
//        ((vl_bms_reg_valid_flag & (BMS_READONLY_DATA1_FLAG_BIT|BMS_READONLY_DATA2_FLAG_BIT))&&
//            (vl_bms_reg_value & BMS_CTRL_BIT_CHG_SWITCH)))
//            (gl_bms_app_bl_status[ELock_bms_index_to_elock_index(NFC_READER_COUNT_MAX-1-i)].is_bl_mode == 0))// ������bootloaderģʽ
        {
            vl_result |= 1;
        }
    }
    //Data	Charger status
    //�����жϼ̵���(PMS�¶ȳ���60���϶���λ)// 2019-4-19
//    if(ADC_get_adc_OT_flag())
//    {
//        vl_result |= 0x80;
//    }

    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_result;

    #if 0
    //Bat Data number
    for(i  = 0, vl_bat_num = 0; i < NFC_READER_COUNT_MAX; i++)
    {
        if((
            #ifdef CHECKING_BAT_IN_IO_FLAG_ENABLE
            //(
            //(FALSE == Battery_is_checking_mode_on_bat_in_by_HW())|| // ��ʹ��Ӳ�����ؼ��Ļ����������������
            //(TRUE == Battery_is_bat_in_via_IO_switch()))&& // ��ҪӲ����⿪�ؼ�⵽��أ����Ǳ�Ҫ����
            #endif
            #ifdef TEST_BAT_V_VIA_CHARGER_EN
            //(
            //(TRUE == Battery_check_bat_v_is_end()) &&
            //(TRUE == Battery_check_bat_v_is_valid()))&& // �ز��ż�����Ƿ����Ӻ�������
            #endif
            ((BMS_READONLY_DATA1_FLAG_BIT|BMS_READONLY_DATA2_FLAG_BIT)&
            Battery_get_reg_valid_flag(ELock_bms_index_to_elock_index(NFC_READER_COUNT_MAX-1-i))))||
            (gl_elock_param[0].elock_bms_flag&(1<<ELock_bms_index_to_elock_index(NFC_READER_COUNT_MAX-1-i)))||
            (gl_bms_app_bl_status[ELock_bms_index_to_elock_index(NFC_READER_COUNT_MAX-1-i)].is_bl_mode))// ����bootloaderģʽ
        {
            vl_bat_num++;
        }
    }
    
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_bat_num;
    //Bat Data size
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = sizeof(st_bat_info_data);

    vl_bat_info_data = (st_bat_info_data *)&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx];
    for(i  = 0, k = 0; i < NFC_READER_COUNT_MAX; i++)
    {
        //if(Battery_get_reg_valid_flag(ELock_bms_index_to_elock_index(i)))
        if(gl_bms_app_bl_status[i].is_bl_mode == 0)
        {
            if(slave_rs485_get_bat_info_data(i,&vl_bat_info_data[k]))
                k++;
        }
        else
        {
            memset(&vl_bat_info_data[k], 0, sizeof(st_bat_info_data));
            //PMS Temp
            vl_bat_info_data[k].stl_pms_tmep[0] = (0xFF & ADC_get_adc_raw_value());
            vl_bat_info_data[k].stl_pms_tmep[1] = (0xFF & ((ADC_get_adc_raw_value())>>8));
            
            vl_bat_info_data[k].stl_port = slave_rs485_changed_bms_index_to_comm(i);
            k++;
        }
    }
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx += sizeof(st_bat_info_data)*vl_bat_num;
    #else
    //���ư�V3.20�Ժ��Э��

////////////////////////////////////////////////////////////////////////////////////////////////////
    //PMS DATA 
    
    //data type
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = 0; // 0, PMS
    //data size
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = 0x0B;
    pl_hb_pms_data = (st_hb_pms_data *)&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx];
    slave_rs485_hb_get_pms_data(0, pl_hb_pms_data);
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx += sizeof(st_hb_pms_data);
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
    #ifdef UNMASK_CODE
    //BAT DATA
    if(((BMS_READONLY_DATA1_FLAG_BIT|BMS_READONLY_DATA2_FLAG_BIT)&
            Battery_get_reg_valid_flag(0))||
            (gl_elock_param[0].elock_bms_flag&(1<<ELock_bms_index_to_elock_index(0)))||
            (gl_bms_app_bl_status[0].is_bl_mode))
    {
        // �е��

        //data type
        pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = 1; // 0, BAT
        //data size
        pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = 0x4C;

        pl_hb_bat_data = (st_hb_bat_data *)&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx];
        if(gl_bms_app_bl_status[0].is_bl_mode == 0)
        {
            slave_rs485_hb_get_bat_data(0,pl_hb_bat_data);
        }
        else
        {
            //��ش���bootloaderģʽ
            memset((char*)pl_hb_bat_data, 0, sizeof(st_hb_bat_data));
            pl_hb_bat_data->stl_port = slave_rs485_changed_bms_index_to_comm(0);
        }

        pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx += sizeof(st_hb_bat_data);
    }
    else
    {
        //�޵��
    }
    #endif
////////////////////////////////////////////////////////////////////////////////////////////////////


    #endif
    
    //�����ݳ���
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx-3;

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif

    return TRUE;
}

bool slave_rs485_cmd_fan_enable(st_hub_cmd *pl_hub_cmd)
{
    //unsigned short vl_crc;
    //if(pl_hub_cmd->stl_dataL < 0)
    //{
    //    return 0;
    //}

    #ifdef UNMASK_CODE
    Fan_open_force();
    #endif
    
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_FAN_EN;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x00;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 3;

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    return 1;
}    

bool slave_rs485_cmd_fan_disable(st_hub_cmd *pl_hub_cmd)
{
    //unsigned short vl_crc;
    //if(pl_hub_cmd->stl_dataL < 0)
    //{
    //    return 0;
    //}

    #ifdef UNMASK_CODE
    Fan_close();
    #endif
    
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_FAN_DISABLE;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x00;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 3;

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    return 1;
}    

bool slave_rs485_cmd_fan_temp_setting(st_hub_cmd *pl_hub_cmd)
{
    //unsigned short vl_crc;
    if((pl_hub_cmd->stl_dataL < 4)||
        (pl_hub_cmd->stl_data[0]!=1))//ֻ����target=PMS
    {
        return 0;
    }

    #ifdef UNMASK_CODE
    
    //����Ҫ�����޸�
    if(pl_hub_cmd->stl_data[2] > pl_hub_cmd->stl_data[3])
        Fan_temp_setting(pl_hub_cmd->stl_data[2], pl_hub_cmd->stl_data[3]);
    
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_FAN_TEMP_SETTING;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x04;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[3] = pl_hub_cmd->stl_data[0];
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[4] = pl_hub_cmd->stl_data[1];

    Fan_temp_get_value(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5], &pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[6]);
    #endif
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 7;

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    return 1;
}

/*
���ص���������ʽ:
1. ��λ���Ƿ�������
2. ����Ƿ�����
3. ����Ƿ��в�������Ĵ���
4. ��طŵ���Ƿ��
5. ��س����Ƿ��
6. ����Ƿ���bootloader״̬
7. �����ID���Ƿ��(ͨ����IO���ж�)
8. �����485��ͨ���Ƿ�����
9. �������ǰ״̬
10. Ӳ��������Ŀ����Ƿ��Ѿ���
11. ADC�¶��Ƿ�������
12. ����״̬
13. �������Ƿ����ڴ�
14. ������Ƿ��������У����䷽��(1 ��ʾ���ڴ���)
15. LED״̬
16. ���ڷ����Ƿ��

*/
bool slave_rs485_cmd_read_runing_param(st_hub_cmd *pl_hub_cmd)
{
    //unsigned short vl_crc;
    unsigned char *vl_p;
    unsigned short vl_tmp_data;
    unsigned short vl_data_index;
    //if(pl_hub_cmd->stl_dataL != 0)
    //{
    //    return 0;
    //}


    
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_R_RUNING_PARAM;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x00;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 3;

    vl_p = &pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx];

    // 1. ��λ���Ƿ�������
//    if(Battery_chg_is_charge_enable(0))
//        vl_p[0] = 1;
//    else
//        vl_p[0] = 0;
    if(sl_master_state.chger_enable_on_unlock_cmd_flag)
    {
        vl_p[0] |= (1<<1);
    }

    //    2. ����Ƿ�����
//    if(is_battery_voltage_OK(0))
//    {
//        unsigned short vl_switch_state;
//        vl_p[1] = 1;

//        vl_switch_state = Battery_get_switch_state(0);
//        //��ػ����Ƿ��Ӧ������
//        if((1<<3) & vl_switch_state) // Bit3    1--G2��Ч��0--G2��Ч
//        {
//            vl_p[1] |= 2;
//        }
//        // V30�汾�Ժ󡣵�س�����־����λ
//        if((1<<14) & vl_switch_state) // Bit14   1--�������־(���״̬��Ч)
//        
//        {
//            vl_p[1] |= 4;
//        }
//    }
//    else
//        vl_p[1] = 0;

    // 3. ����Ƿ��в�������Ĵ���
    if(is_battery_error(0))
        vl_p[2] = 1;
    else
        vl_p[2] = 0;

    // 4. ��طŵ���Ƿ��
    vl_tmp_data = Battery_get_switch_state(0);
    if(BMS_CTRL_BIT_SUPPLY_SWITCH&vl_tmp_data)
        vl_p[3] = 1;
    else
        vl_p[3] = 0;

    
    // 5. ��س����Ƿ��
    if(BMS_CTRL_BIT_CHG_SWITCH&vl_tmp_data)
        vl_p[4] = 1;
    else
        vl_p[4] = 0;

    // 6. ����Ƿ���bootloader״̬
    if(gl_bms_app_bl_status[0].is_bl_mode)
        vl_p[5] = 1;
    else
        vl_p[5] = 0;

    // 7. �����ID���Ƿ��(ͨ����IO���ж�)bit0��ʾ; ID���Ƿ����bit1;485�����Ƿ�������bit2
//    if(gpio_output_bit_get(BATTERY_CHARGER_ENABLE_PORT, BATTERY_CHARGER_ENABLE_PIN))
//        vl_p[6] = 1;
//    else
//        vl_p[6] = 0;
//    if(0 == gpio_input_bit_get(BATTERY_CHARGER_TEST_PORT, BATTERY_CHARGER_TEST_PIN))
//    {
//        vl_p[6] |= 0x02;
//    }

    if(master_rs485_is_charge_en())
    {
        vl_p[6] |= 0x04;
    }    

    // 8. �����485��ͨ���Ƿ�����
    if(sl_master_state.chg_485_cmd_poll_faild_cnt >= MASTER_485_CMD_POLL_ERROR_CNT_MAX) // ����������ͨѶʧ�ܣ��򱨴���
        vl_p[7] = 1;
    else
        vl_p[7] = 0;

    // 9. �������ǰ״̬
    vl_p[8] = ((unsigned char*)&gl_poll_cmd_cnf_param)[6];

    #ifdef BATTERY_CHG_ENABLE_PORT
    // 10. Ӳ��������Ŀ����Ƿ��Ѿ���
    if(GPIO_PinOutGet(BATTERY_CHG_ENABLE_PORT, BATTERY_CHG_ENABLE_PIN))
        vl_p[9] = 1;
    else
        vl_p[9] = 0;   
    #else
    vl_p[9] = 0;   
    #endif

    #ifdef UNMASK_CODE
    // 11. ADC�¶��Ƿ�������
    if(ADC_is_CHG_enable())
        vl_p[10] = 1;
    else
        vl_p[10] = 0;
    
    // 12. ����״̬, 1, �ر�; 0, ��
    vl_p[11] = ELock_get_status((ENUM_ELOCK_INDEX)0);

    // 13. �������Ƿ����ڴ�
    if(GPIO_PinOutGet(ELOCK_CTRL_OUTSIDE_PORT, ELOCK_CTRL_OUTSIDE_PIN))
    {
        vl_p[12] = 1;
    }
    else
    {
        vl_p[12] = 0;
    }

    // 14. ������Ƿ��������У����䷽��(1 ��ʾ���ڴ���)
    if((gl_stepper_motor_param.timer_cnt == 0)&&
    (gl_stepper_motor_param.step_valid_flag == 0)&&
    (gl_stepper_motor_param.sm_dir_step_counter == 0))
    {
        vl_p[13] = 0;
    }
    else
    {
        vl_p[13] = 1;
    }
    
    if(gl_stepper_motor_param.dir == ENUM_SM_OPEN_DIR)
    {
        vl_p[13] |= 0x80;
    }
        
    // 15. LED״̬
    vl_p[14] = main_led_get_mode();

    // 16. ���ڷ����Ƿ��
    if(GPIO_PinOutGet(FAN_CTRL_PORT, FAN_CTRL_PIN))
        vl_p[15] = 1;
    else
        vl_p[15] = 0;

    #endif

    // 17 NFC��������ѯ״̬�� bit7��1��ʾһֱ����ѯ��bit0~bit6��ʾ��ѯʱ����0��ʾ1���ӣ�1��ʾ10���ӣ�2��ʾ30����
    if(sl_bat_poll_param.is_poll_allways_en_flag)
    {
        vl_p[16] = 0x80;
    }
    else
    {
        vl_p[16] = 0x00;
//        if(sl_bat_poll_param.poll_interval_MAX == BAT_POLL_BIG_INTERVAL_MAX)
//        {
//            vl_p[16] |= 0x01;
//        }
//        if(sl_bat_poll_param.poll_interval_MAX == BAT_POLL_BIG_BIG_INTERVAL_MAX)
//        {
//            vl_p[16] |= 0x02;
//        }
    }

    // 18 ��ѯ���ʱ��ļ���������5000Ϊ��λ
    vl_p[17] = sl_bat_poll_param.poll_inerval_timer_cnt/5000;

    // 19 ����bmsʱ���ӳټ����ѯ�ļ�������1��ʾ�����ӳ��У�0��ʾ�ӳٽ���
    if(sl_bat_poll_param.poll_interval_for_bms_update_cnt)
    {
        vl_p[18] = 1;
    }
    else
    {
        vl_p[18] = 0;
    }

    // 20 ~21���ó�����ĵ�ѹ�����ֽ���ǰ��0.1Ϊ��λ
    vl_p[19] = gl_poll_cmd_param.expect_voltage[0];
    vl_p[20] = gl_poll_cmd_param.expect_voltage[1];
    // 22~23���ó�����ĵ��������ֽ���ǰ��0.1Ϊ��λ
    vl_p[21] = gl_poll_cmd_param.expect_current[0];
    vl_p[22] = gl_poll_cmd_param.expect_current[1];

    vl_data_index = 23;
    
    #ifdef EVENT_TO_STOP_CHARGING_ENABLE
    //V28�Ժ�
    //�����¼�����־��0��ʾ�����磬1��ʾֹͣ���
    vl_p[vl_data_index++] = Battery_get_event_to_stop_chg_flag();
    {
        unsigned int vl_tmp_int_value;
        vl_tmp_int_value = Battery_get_src_of_stop_flag();
        vl_p[vl_data_index++] = vl_tmp_int_value&0xff;
        vl_p[vl_data_index++] = (vl_tmp_int_value>>8)&0xff;
        vl_p[vl_data_index++] = (vl_tmp_int_value>>16)&0xff;
        vl_p[vl_data_index++] = (vl_tmp_int_value>>24)&0xff;
    }
    #endif

    #ifdef TEST_DETECT_BAT_ERROR_EN
    //V30�Ժ�
    if(Battery_check_bat_is_on())
    {
        vl_p[vl_data_index] = 1;
    }
    else
    {
        vl_p[vl_data_index] = 0;
    }
    if(slave_rs485_is_detect_bat_error_enable())
    {
        vl_p[vl_data_index] |= (1<<1);
    }
    
    vl_data_index++;
    vl_p[vl_data_index++] = Battery_check_bat_get_bat_error();
    #endif

    //V30�Ժ�
    //���ӳ�������ص���Ϣ
    memcpy(&vl_p[vl_data_index], ((unsigned char*)&gl_poll_cmd_cnf_param), 6);
    vl_data_index += 6;

////////////////////////////////////////////////////////////////////////////////////////////////////
    vl_p[vl_data_index] = 0;
    //��ʱ��û�г�����(V32�汾�Ժ���Ч)
//    if(Battery_is_chg_current_false())
//    {
//        vl_p[vl_data_index] |= (1<<0);
//    }


    #ifdef CHECKING_BAT_IN_IO_FLAG_ENABLE
    /*
        ����΢�����ص��źŷ��ڵ����Ϣ�����е�damage�ֽ�bit4��
        û�м�⵽ʱ������Ϊ1����⵽����Ϊ0��
        */
    if((gl_bat_theft_prevention_param.switch_default_state)|| //ǿ���ϴ�δ��λ
        (
    (TRUE == Battery_is_checking_mode_on_bat_in_by_HW())&&
    (FALSE == Battery_is_bat_in_via_IO_switch())&&
    (FALSE == SM_is_opening())&&
    (ENUM_ELOCK_CMD_STATUS_CLOSE == ELock_get_cmd_status(ELock_bms_index_to_elock_index(0)))))
    {
        vl_p[vl_data_index] |= (1<<1);
    }
    #endif   
    
    #ifdef TEST_BAT_V_VIA_CHARGER_EN
    if((0 == gl_bat_theft_prevention_param.connector_default_state)&&
        (
        #ifdef CHECKING_BAT_IN_IO_FLAG_ENABLE
        //(FALSE== Battery_is_checking_mode_on_bat_in_by_HW()) ||
        #endif
        ((TRUE == Battery_check_bat_v_is_end()) &&
    (TRUE == Battery_check_bat_v_is_valid()))))// �ز��ż�����Ƿ����Ӻ�������
    {
        //pl_bat_info_data->stl_damage &= (~(1<<5));
    }
    else
    {
        if((FALSE == Battery_check_bat_v_is_off()) &&
            (ENUM_ELOCK_CMD_STATUS_CLOSE == ELock_get_cmd_status(ELock_bms_index_to_elock_index(0))))
        {
            vl_p[vl_data_index] |= (1<<2);
        }
    }
    #endif

    #ifdef UNMASK_CODE
    //������ָ��ܴ򿪲���
    if((ELOCK_LOCK == ELock_get_status((ENUM_ELOCK_INDEX)(ELock_bms_index_to_elock_index(0))))&&
            (ENUM_ELOCK_CMD_STATUS_OPEN == ELock_get_cmd_status(ELock_bms_index_to_elock_index(0)))&&
            (FALSE == main_led_is_opening_mode()))
    {
        vl_p[vl_data_index] |= (1<<3);
    }

    //���ű��쳣��
    if(((ELOCK_OPEN == ELock_get_status((ENUM_ELOCK_INDEX)(ELock_bms_index_to_elock_index(0)))))&&
            (ENUM_ELOCK_CMD_STATUS_CLOSE == ELock_get_cmd_status(ELock_bms_index_to_elock_index(0))))
    {
        vl_p[vl_data_index] |= (1<<4);
    }

    // ���ù���"���ݵ���¶Ⱦ����Ƿ���ͳ�����"  2019-6-20
    if(TRUE == main_get_temp_charge_enable_flag())
    {
        //����
        vl_p[vl_data_index] |= (1<<5);
    }
    #endif
    vl_data_index++;

    //V40�汾��֧�ֲ鿴��׿���·�������������ѹ
    vl_p[vl_data_index++] = (0xff & master_rs485_get_chg_current_max_value());
    vl_p[vl_data_index++] = (0xff & ((master_rs485_get_chg_current_max_value()) >> 8));
    vl_p[vl_data_index++] = (0xff & master_rs485_get_chg_voltage_max_value());
    vl_p[vl_data_index++] = (0xff & ((master_rs485_get_chg_voltage_max_value()) >> 8));
    
    #ifdef BAT_TEMP_CHARGER_ENABLE
    memcpy(&vl_p[vl_data_index], (unsigned char*)&gl_bat_low_temp_param, 20);
    vl_data_index += 20;
    #endif

    //V41�汾�Ժ�֧�֣���������صĸ��������Ϣ
    vl_p[vl_data_index++] = ((unsigned char*)&gl_poll_cmd_cnf_param)[7];
    vl_p[vl_data_index++] = ((unsigned char*)&gl_poll_cmd_cnf_param)[8];
    vl_p[vl_data_index++] = ((unsigned char*)&gl_poll_cmd_cnf_param)[9];

    #ifdef PMS_AUTO_INCREASE_SETTING_VOLTAGE_ENABLE
    memcpy(&vl_p[vl_data_index], (unsigned char*)&gl_charger_setting_V_offset_param, 12);
    vl_data_index += 12;
    #else
    memset(&vl_p[vl_data_index], 0, 12);
    vl_data_index += 12;
    #endif

    #ifdef CABINET_WARMOR_SUPPLY_BY_CHARGER
    //��λ���Ƿ��������
    if(master_rs485_get_charger_warmer_flag())
        vl_p[vl_data_index] = 1;
    else
        vl_p[vl_data_index] = 0;
    // PMS�ж��Ƿ���ϼ�������
    if(master_rs485_get_charger_warmer_action_enable_flag())
        vl_p[vl_data_index] |= (1<<1);

    //PMS�жϵ�����ģʽ�·�������--- ��ؼ�⵽�е�������������
    if(master_rs485_is_charger_warmer_err())
        vl_p[vl_data_index] |= (1<<2);

    //�������Ȳ���ģʽ
    if(master_rs485_is_charger_warmer_test_enable())
        vl_p[vl_data_index] |= (1<<3);

    vl_data_index++;
    #else
    vl_p[vl_data_index++] = 0;
    #endif

    
    #ifdef BAT_PMS_AUTHORITY_FUNCTION_EN
    //�Ƿ�����PMS��֤BAT����
    if(Authority_get_onoff_about_pms_bat())
    {
        vl_p[vl_data_index] = 1;
    }
    else
    {
        vl_p[vl_data_index] = 0;
    }
    
    //pms��֤����Ƿ�ɹ�
    if(Is_Authority_done_about_pms_bat())
    {
        vl_p[vl_data_index] |= (1<<1);
    }

    //�����֤PMS�Ƿ�ɹ�
    if(Is_Authority_done_about_bat_pms())
    {
        vl_p[vl_data_index] |= (1<<2);
    }
    vl_data_index++;
    #else
    vl_p[vl_data_index++] = 0;
    #endif    


    //���ò���ʱ����Ҫ��vl_p[vl_data_index] һ����ʼֵ����Ȼ���ܳ�����ʵ�����������
    
    
////////////////////////////////////////////////////////////////////////////////////////////////////

    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = vl_data_index;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx += vl_data_index;

    //#ifdef EVENT_TO_STOP_CHARGING_ENABLE
    //pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] += 5;
    //pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx += 5;
    //#endif

    //#ifdef TEST_DETECT_BAT_ERROR_EN
    //pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] += 1;
    //pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx += 1;
    //#endif

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    return 1;
}

#ifdef UNMASK_CODE

extern st_elock_param gl_elock_param[ENUM_ELOCK_NUM_MAX];
extern st_led_param sl_led_param;
extern unsigned char sl_fan_enable_flag;
extern unsigned char sl_temp_up_value;
extern unsigned char sl_temp_down_value;
extern unsigned char sl_force_to_open_fan_flag;
#endif
bool slave_rs485_cmd_read_real_time_info(st_hub_cmd *pl_hub_cmd)
{
    //unsigned short vl_crc;
    unsigned char *vl_p;
    unsigned int vl_index;
    
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_R_REAL_TIME_INFO;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x00;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 3;

    vl_p = &pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx];

    #ifdef UNMASK_CODE
    //1. ���뿪��IO   
    vl_p[0] = 0;
    vl_p[0] |= GPIO_PinInGet(RS485_DEV_ADDR_BIT3_PORT, RS485_DEV_ADDR_BIT3_PIN);
    vl_p[0] = (vl_p[0]<<1);
    vl_p[0] |= GPIO_PinInGet(RS485_DEV_ADDR_BIT2_PORT, RS485_DEV_ADDR_BIT2_PIN);
    vl_p[0] = (vl_p[0]<<1);
    vl_p[0] |= GPIO_PinInGet(RS485_DEV_ADDR_BIT1_PORT, RS485_DEV_ADDR_BIT1_PIN);
    vl_p[0] = (vl_p[0]<<1);
    vl_p[0] |= GPIO_PinInGet(RS485_DEV_ADDR_BIT0_PORT, RS485_DEV_ADDR_BIT0_PIN);

    //2. LEDʹ��
    vl_p[1] = GPIO_PinInGet(LED_GREEN_ENABLE_PORT, LED_GREEN_ENABLE_PIN);
    vl_p[1] = (vl_p[1]<<1);
    vl_p[1] |= GPIO_PinInGet(LED_RED_ENABLE_PORT, LED_RED_ENABLE_PIN);

    //3. 
    vl_p[2] = gpio_output_bit_get(BATTERY_CHARGER_485_TX_ENABLE_PORT, BATTERY_CHARGER_485_TX_ENABLE_PIN);;
    vl_p[2] = (vl_p[2]<<1);
    vl_p[2] |= GPIO_PinInGet(FAN_CTRL_PORT, FAN_CTRL_PIN);
    vl_p[2] = (vl_p[2]<<1);
    vl_p[2] |= gpio_output_bit_get(BATTERY_CHARGER_ENABLE_PORT, BATTERY_CHARGER_ENABLE_PIN);
    vl_p[2] = (vl_p[2]<<1);
    vl_p[2] |= gpio_input_bit_get(BATTERY_CHARGER_TEST_PORT, BATTERY_CHARGER_TEST_PIN);
    vl_p[2] = (vl_p[2]<<1);
    #ifdef BATTERY_CHG_ENABLE_PORT   
    vl_p[2] |= GPIO_PinInGet(BATTERY_CHG_ENABLE_PORT, BATTERY_CHG_ENABLE_PIN);
    #endif
    vl_p[2] = (vl_p[2]<<1);
    vl_p[2] |= GPIO_PinInGet(FM17522_NPD_PORT, FM17522_NPD_PIN);
    vl_p[2] = (vl_p[2]<<1);
    vl_p[2] |= GPIO_PinInGet(MICRO_SWITCH_OPEN_TEST_PORT, MICRO_SWITCH_OPEN_TEST_PIN);
    vl_p[2] = (vl_p[2]<<1);
    vl_p[2] |= GPIO_PinInGet(MICRO_SWITCH_CLOSE_TEST_PORT, MICRO_SWITCH_CLOSE_TEST_PIN);

    //4. 
    vl_p[3] = GPIO_PinInGet(ELOCK_TEST_OUTSIDE_PORT, ELOCK_TEST_OUTSIDE_PIN);
    vl_p[3] = (vl_p[3]<<1);
    vl_p[3] |= GPIO_PinInGet(ELOCK_CTRL_OUTSIDE_PORT, ELOCK_CTRL_OUTSIDE_PIN);
    vl_index = 3;
/*
    // gl_stepper_motor_param
    // gl_elock_param
    // sl_led_param
    static unsigned char sl_fan_enable_flag;
    static unsigned char sl_temp_up_value;
    static unsigned char sl_temp_down_value;
    static unsigned char sl_force_to_open_fan_flag;
    sl_bat_poll_param
    gl_bms_info_p
*/
    vl_index++;
    memcpy(&vl_p[vl_index], &gl_stepper_motor_param, sizeof(st_stepper_motor_param));
    vl_index+=sizeof(st_stepper_motor_param);
    memcpy(&vl_p[vl_index], &gl_elock_param, sizeof(st_elock_param));
    vl_index+=sizeof(st_elock_param);

    memcpy(&vl_p[vl_index], &sl_led_param, sizeof(st_led_param));
    vl_index+=sizeof(st_led_param);

    vl_p[vl_index++] = sl_fan_enable_flag;
    vl_p[vl_index++] = sl_temp_up_value;
    vl_p[vl_index++] = sl_temp_down_value;
    vl_p[vl_index++] = sl_force_to_open_fan_flag;

    memcpy(&vl_p[vl_index], &sl_bat_poll_param, sizeof(st_bat_poll_param));
    vl_index+=sizeof(st_bat_poll_param);

    //memcpy(&vl_p[vl_index], gl_bms_info_p[0], 4);
    //vl_index+=4;

    
    #endif
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = vl_index;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx += vl_index;

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    
    return 1;
}

//С��ģʽ����Ĵ���ֵ
static void slave_rs485_get_get_reg_value_by_little_end(unsigned char bms_index, 
                                                                unsigned short reg_index, 
                                                                unsigned char *pl_data_out)
{
    unsigned short vl_tmp_data;
    vl_tmp_data = Battery_get_reg_value(bms_index, reg_index);
    pl_data_out[0] = vl_tmp_data & 0xFF;
    pl_data_out[1] = (vl_tmp_data>>8) & 0xFF;
}

bool slave_rs485_cmd_read_BAT_info(st_hub_cmd *pl_hub_cmd)
{
    //unsigned short vl_crc;
    unsigned char *vl_p;
    unsigned int vl_index;
    unsigned char i;
    
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_R_BAT_INFO;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x00;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 3;

    vl_p = &pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx];

    //    ����Ƿ�����
//    if(is_battery_voltage_OK(0))
//    {
//        vl_index = 0;
//        vl_p[vl_index++] = 1;
//        vl_p++;

//        //512 CTRL    BMS������
//        slave_rs485_get_get_reg_value_by_little_end(0, ENUM_REG_ADDR_CTRL, vl_p);
//        vl_p += 2;
//        for(i = 0; i < 5; i++)
//        {
//            slave_rs485_get_get_reg_value_by_little_end(0, ENUM_REG_ADDR_STATE + i, vl_p);
//            vl_p += 2;
//        }
//        #if 0
//        // 260 STATE   BMS״̬
//        slave_rs485_get_get_reg_value_by_little_end(0, ENUM_REG_ADDR_STATE, vl_p);
//        vl_p += 2;
//        //261 SOC �����ɵ�״̬
//        slave_rs485_get_get_reg_value_by_little_end(0, ENUM_REG_ADDR_SOC, vl_p);
//        vl_p += 2;
//        //262 TVOLT   �ܵ�ѹ
//        slave_rs485_get_get_reg_value_by_little_end(0, ENUM_REG_ADDR_TVOLT, vl_p);
//        vl_p += 2;
//        //263 TCURR   �ܵ���
//        slave_rs485_get_get_reg_value_by_little_end(0, ENUM_REG_ADDR_TCURR, vl_p);
//        vl_p += 2;
//        //264 HTEMP   ��ߵ���¶�
//        slave_rs485_get_get_reg_value_by_little_end(0, ENUM_REG_ADDR_HTEMP, vl_p);
//        vl_p += 2;
//        #endif
//        //266 LTEMP   ��͵���¶�
//        slave_rs485_get_get_reg_value_by_little_end(0, ENUM_REG_ADDR_LTEMP, vl_p);
//        vl_p += 2;
//        //268 HVOLT   ��ߵ����ѹ
//        slave_rs485_get_get_reg_value_by_little_end(0, ENUM_REG_ADDR_HVOLT, vl_p);
//        vl_p += 2;
//        //270 LVOLT   ��͵����ѹ
//        slave_rs485_get_get_reg_value_by_little_end(0, ENUM_REG_ADDR_LVOLT, vl_p);
//        vl_p += 2;
//        //273 CSOP    10s������������
//        slave_rs485_get_get_reg_value_by_little_end(0, ENUM_REG_ADDR_CSOP, vl_p);
//        vl_p += 2;
//        //274 SOH ����״̬
//        slave_rs485_get_get_reg_value_by_little_end(0, ENUM_REG_ADDR_SOH, vl_p);
//        vl_p += 2;
//        //275 CYCLE   ѭ������
//        slave_rs485_get_get_reg_value_by_little_end(0, ENUM_REG_ADDR_CYCLE, vl_p);
//        vl_p += 2;

//        for(i = 0; i < 28; i++)
//        {
//            slave_rs485_get_get_reg_value_by_little_end(0, ENUM_REG_ADDR_DEVFT1 + i, vl_p);
//            vl_p += 2;
//        }
//        //315     ������ѹ
//        slave_rs485_get_get_reg_value_by_little_end(0, ENUM_REG_ADDR_MAXCHGV, vl_p);
//        vl_p += 2;

//        vl_index += (41*2);
//        pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = vl_index;
//        pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx += vl_index;
//    }
//    else
//    {
//        vl_p[0] = 0;
//        pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 1;
//        pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx += 1;
//    }

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    
    return 1;
}

#ifndef CABINET_WARMOR_SUPPLY_BY_CHARGER 
bool slave_rs485_cmd_read_BAT_info_when_critical_err(st_hub_cmd *pl_hub_cmd)
{
    //unsigned short vl_crc;
    unsigned char *vl_p;
    unsigned int vl_index;
    
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_R_BAT_INFO_WHEN_CRITICAL_ERR;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x00;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 3;

    vl_p = &pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx];

//    vl_index = Battery_get_bat_reg_state_when_critical_error(vl_p);
    
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = vl_index;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx += vl_index;

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    
    return 1;
}

bool slave_rs485_cmd_read_BAT_info_when_even_to_stop(st_hub_cmd *pl_hub_cmd)
{
    //unsigned short vl_crc;
    unsigned char *vl_p;
    unsigned int vl_index;
    
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_R_BAT_INFO_WHEN_EVENT_TO_STOP_CHG;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x00;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 3;

    vl_p = &pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx];
    #ifdef EVENT_TO_STOP_CHARGING_ENABLE
    vl_index = Battery_get_bat_reg_state_when_event_to_stop_chg(vl_p);
    #else
    vl_index = 0;
    #endif
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = vl_index;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx += vl_index;

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    
    return 1;
}
#endif

bool slave_rs485_cmd_read_BAT_info_when_detect_bat_error(st_hub_cmd *pl_hub_cmd)
{
    //unsigned short vl_crc;
    unsigned char *vl_p;
    unsigned int vl_index;
    
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_R_BAT_INFO_WHEN_DETECT_BAT_ERROR;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x00;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 3;

    vl_p = &pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx];
    #ifdef TEST_DETECT_BAT_ERROR_EN
    vl_index = Battery_get_bat_reg_state_when_detect_bat_error(vl_p);
    #else
    vl_index = 0;
    #endif
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = vl_index;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx += vl_index;

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    
    return 1;
}

bool slave_rs485_cmd_read_ADC_raw_value(st_hub_cmd *pl_hub_cmd)
{
    //unsigned short vl_crc;
    unsigned char *vl_p;
    unsigned int vl_index;
    unsigned int vl_adc_value;
    
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_R_ADC_RAW_VALUE;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x00;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 3;

    vl_p = &pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx];
    vl_index = 0;

//    vl_adc_value = ADC_get_adc_raw_value();
    memcpy((char*)&vl_p[vl_index], (char*)&vl_adc_value, sizeof(unsigned int));
    vl_index += sizeof(unsigned int);
    vl_adc_value = (3300*vl_adc_value); // ADC�Ĳο���ѹ��3.3V, ������0~0xFFF
    vl_adc_value /= 0xFFF;
    memcpy((char*)&vl_p[vl_index], (char*)&vl_adc_value, sizeof(unsigned int));
    vl_index += sizeof(unsigned int);
    
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = vl_index;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx += vl_index;

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    
    return 1;
}


bool slave_rs485_cmd_clear_pms_error_flag(st_hub_cmd *pl_hub_cmd)
{
    //unsigned short vl_crc;
    if(pl_hub_cmd->stl_dataL < 1)
    {
        return 0;
    }

    if(pl_hub_cmd->stl_data[0] & (1<<0))
    {
        #ifdef EVENT_TO_STOP_CHARGING_ENABLE
        //������¼���ֹͣ���ı�־��
        Battery_clear_event_to_stop_chg_flag();
        #endif
    }

    if(pl_hub_cmd->stl_data[0] & (1<<1))
    {
        #ifdef TEST_DETECT_BAT_ERROR_EN
        Battery_check_bat_clear_on_and_error();
        #endif
    }

    if(pl_hub_cmd->stl_data[0] & (1<<2))
    {
//        Battery_bat_chg_current_false_flag_clear();
    }
    
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_CLEAR_PMS_ERROR_FLAG;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x01;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[3] = ENUM_COMM_ERROR_CODE_OK;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 4;

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    
    return 1;
}

bool slave_rs485_cmd_pms_reset(st_hub_cmd *pl_hub_cmd)
{
    //unsigned short vl_crc;
    if(pl_hub_cmd->stl_dataL < 2)
    {
        return 0;
    }

    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_PMS_RESET;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x01;

    
    if((pl_hub_cmd->stl_data[0] == 0x55) && (pl_hub_cmd->stl_data[1] == 0xAA))
    {
        //485�����ڷ�����ɺ�����
        slave_rs485_reboot_after_tx |= (1<< ENUM_COMM_INDEX_COMM);
        
        pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[3] = ENUM_COMM_ERROR_CODE_OK;
    }
    else
    {
        pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[3] = ENUM_COMM_ERROR_CODE_FAILED;
    }

    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 4;

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    
    return 1;
}

#ifdef TEST_DETECT_BAT_ERROR_EN
//Ĭ������1��������0���ǲ�����
static unsigned char sl_detect_bat_error_enable = 1;
bool slave_rs485_is_detect_bat_error_enable(void)
{
    return (sl_detect_bat_error_enable?TRUE:FALSE);
}

bool slave_rs485_cmd_detect_bat_error(st_hub_cmd *pl_hub_cmd)
{
    //unsigned short vl_crc;
    if(pl_hub_cmd->stl_dataL < 1)
    {
        return 0;
    }

    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_DETECT_BAT_ERROR;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x01;

    
    if(pl_hub_cmd->stl_data[0] == 0x00)
    {
        //����������ȥ���Ե���Ƿ��й���
        sl_detect_bat_error_enable = 0;
    }
    else
    {
        //����
        sl_detect_bat_error_enable = 1;
    }
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[3] = ENUM_COMM_ERROR_CODE_OK;

    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 4;

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    
    return 1;
}
#endif

#ifdef EVENT_TO_STOP_CHARGING_ENABLE

bool slave_rs485_cmd_event_to_stop_charging_flag_be_disable_flag(st_hub_cmd *pl_hub_cmd)
{
    //unsigned short vl_crc;
    unsigned int vl_disable_flag;
    if(pl_hub_cmd->stl_dataL < 4)
    {
        return 0;
    }

    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_EVENT_TO_STOP_CHARGING_FLAG_BE_DISABLE_FLAG;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x05;

    vl_disable_flag = (pl_hub_cmd->stl_data[0] |
                    (pl_hub_cmd->stl_data[1] << 8) |
                    (pl_hub_cmd->stl_data[2] << 16) |
                    (pl_hub_cmd->stl_data[3] << 24));

    #ifdef EVENT_TO_STOP_CHARGING_ENABLE
    Battery_set_event_to_stop_chg_flag_disable_flag(vl_disable_flag);
    #endif


    vl_disable_flag = Battery_get_event_to_stop_chg_flag_disable_flag();
    
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[3] = ENUM_COMM_ERROR_CODE_OK;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[4] = (vl_disable_flag & 0xFF);
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = ((vl_disable_flag >> 8) & 0xFF);
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[6] = ((vl_disable_flag >> 16)& 0xFF);
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[7] = ((vl_disable_flag >> 24)& 0xFF);    

    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 8;

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    
    return 1;
}

bool slave_rs485_cmd_event_to_stop_charging_flag_be_disable_flag_clear(st_hub_cmd *pl_hub_cmd)
{
    //unsigned short vl_crc;
    unsigned int vl_disable_flag;
    if(pl_hub_cmd->stl_dataL < 4)
    {
        return 0;
    }

    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_EVENT_TO_STOP_CHARGING_FLAG_BE_DISABLE_FLAG_CLEAR;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x05;

    vl_disable_flag = (pl_hub_cmd->stl_data[0] |
                    (pl_hub_cmd->stl_data[1] << 8) |
                    (pl_hub_cmd->stl_data[2] << 16) |
                    (pl_hub_cmd->stl_data[3] << 24));

    Battery_clear_event_to_stop_chg_flag_disable_flag(vl_disable_flag);


    vl_disable_flag = Battery_get_event_to_stop_chg_flag_disable_flag();
    
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[3] = ENUM_COMM_ERROR_CODE_OK;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[4] = (vl_disable_flag & 0xFF);
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = ((vl_disable_flag >> 8) & 0xFF);
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[6] = ((vl_disable_flag >> 16)& 0xFF);
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[7] = ((vl_disable_flag >> 24)& 0xFF);    

    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 8;

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    
    return 1;
}    

#endif



////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef TEST_TEST_CONDITION_CREATE_ENABLE
unsigned char gl_cmd_test_test[2] = {0x90, 0x01};
bool slave_rs485_cmd_test_test(st_hub_cmd *pl_hub_cmd)
{
    //unsigned short vl_crc;
    gl_cmd_test_test[0] = pl_hub_cmd->stl_data[0];//�������¶�
    gl_cmd_test_test[1] = pl_hub_cmd->stl_data[1];

    
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = 0x83;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x01;

    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[3] = ENUM_COMM_ERROR_CODE_OK;

    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 4;

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    
    return 1;
}
#endif
////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef BAT_PMS_AUTHORITY_FUNCTION_EN
bool slave_rs485_cmd_pms_bat_authority(st_hub_cmd *pl_hub_cmd)
{
    //unsigned short vl_crc;
    if(pl_hub_cmd->stl_dataL < 1)
    {
        return 0;
    }

    Authority_set_onoff(pl_hub_cmd->stl_data[0]);

    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_PMS_BAT_AUTHORITY_ENABLE;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x01;

    
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[3] = Authority_get_onoff_about_pms_bat();

    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 4;

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    
    return 1;
}
#endif

bool slave_rs485_cmd_read_charger_temp_value(st_hub_cmd *pl_hub_cmd)
{
    //unsigned short vl_crc;
    unsigned char *vl_p;
    unsigned int vl_index;
    
    
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_READ_CHARGER_TEMP_VALUE;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x00;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 3;

    vl_p = &pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx];
    vl_index = 0;

    vl_p[vl_index++] = gl_poll_cmd_cnf_param.charger_temp;
    
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = vl_index;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx += vl_index;

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    
    return 1;
}


#ifdef CABINET_WARMOR_SUPPLY_BY_CHARGER
bool slave_rs485_cmd_charger_warmer_enable(st_hub_cmd *pl_hub_cmd)
{
    //unsigned short vl_crc;
    if(pl_hub_cmd->stl_dataL < 1)
    {
        return 0;
    }

    master_rs485_set_charger_warmer_flag(pl_hub_cmd->stl_data[0]);

    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_CHARGER_WARMER_ENABLE;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x01;

    
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[3] = master_rs485_get_charger_warmer_flag();

    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 4;

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    
    return 1;
}

bool slave_rs485_cmd_charger_warmer_test_enable(st_hub_cmd *pl_hub_cmd)
{
    //unsigned short vl_crc;
    if(pl_hub_cmd->stl_dataL < 1)
    {
        return 0;
    }

    master_rs485_set_charger_warmer_test_flag(pl_hub_cmd->stl_data[0]);

    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_HEATER_ONOFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x01;

    
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[3] = master_rs485_is_charger_warmer_test_enable();

    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 4;

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    
    return 1;
}

#endif




static unsigned char sl_updating_bms_index = 0xFF;
unsigned char slave_rs485_get_updating_bms_index(void)
{
    return sl_updating_bms_index;
}

void slave_rs485_set_updating_bms_index(unsigned char bms_index)
{
    sl_updating_bms_index = bms_index;
}

bool slave_rs485_cmd_get_dev_version(st_hub_cmd *pl_hub_cmd)
{
    
    //unsigned short vl_crc;

    if(pl_hub_cmd->stl_dataL < 2)
        return 0;
    switch(pl_hub_cmd->stl_data[0])
    {
        case 0:
            break;
        case 1:
            //PMS
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_DEV_VERSION;
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 3+sizeof(st_version_desc) + 14;  //data len
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[3] = 1; //target PMS
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[4] = pl_hub_cmd->stl_data[1]; //port
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = ENUM_COMM_ERROR_CODE_OK; //error_code 
            memcpy(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[6], &gl_version_desc_PMS, sizeof(st_version_desc));
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 6+sizeof(st_version_desc);

            #ifdef UNMASK_CODE
            memcpy((char*)&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx], 
                main_get_PMS_SN(), 14);
            #endif
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx += 14;

            #if 0
            vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
            //crc
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
            #else
            slave_rs485_calc_TX_tail();
            #endif

            //���÷��ͳ�ʱ����ֵ����ʱ�Ͳ��ٷ�������
            slave_rs485_comm_TX_delay_cnt_max = SLAVE_RS485_COMM_TX_DELAY_CNT_MAX_LONG;
            slave_rs485_run_state[ENUM_COMM_INDEX_COMM]=MASTER_RS485_RUN_TX;
            return 0;
        case 2:
            //bat
            {
                unsigned char vl_bms_index;
                vl_bms_index = pl_hub_cmd->stl_data[1];

                slave_rs485_set_updating_bms_index(vl_bms_index);
                
                //ת���ַΪʵ�ʰ��ϵĵ�ַ
                vl_bms_index = slave_rs485_changed_bms_index_from_comm(vl_bms_index);
            
                if(gl_bms_app_bl_status[vl_bms_index].is_bl_mode)
                {
                    //����������Ч��أ�������bootloaderģʽ
                    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
                    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_DEV_VERSION;
                    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 3+sizeof(st_version_desc);  //data len
                    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[3] = pl_hub_cmd->stl_data[0]; //target PMS
                    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[4] = pl_hub_cmd->stl_data[1]; //port
                    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = ENUM_COMM_ERROR_CODE_OK; //error_code 


//                    Battery_get_bat_desc(0,(st_version_desc*)&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[6]);
                    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 6+sizeof(st_version_desc);

                    #if 0
                    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
                    //crc
                    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
                    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
                    #else
                    slave_rs485_calc_TX_tail();
                    #endif
    
                    //���÷��ͳ�ʱ����ֵ����ʱ�Ͳ��ٷ�������
                    slave_rs485_comm_TX_delay_cnt_max = SLAVE_RS485_COMM_TX_DELAY_CNT_MAX_LONG;                    
                    slave_rs485_run_state[ENUM_COMM_INDEX_COMM]=MASTER_RS485_RUN_TX;
                    return 0;
                }
//                else if((is_battery_voltage_OK(0))&&((BMS_DEV_INFO_FLAG_BIT&Battery_get_reg_valid_flag(0))))
//                {
//                    //����������Ч���
//                    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
//                    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_DEV_VERSION;
//                    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 3+sizeof(st_version_desc);  //data len
//                    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[3] = pl_hub_cmd->stl_data[0]; //target PMS
//                    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[4] = pl_hub_cmd->stl_data[1]; //port
//                    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = ENUM_COMM_ERROR_CODE_OK; //error_code 


////                    Battery_get_bat_desc(0,(st_version_desc*)&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[6]);
//                    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 6+sizeof(st_version_desc);

//                    #if 0
//                    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
//                    //crc
//                    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
//                    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
//                    #else
//                    slave_rs485_calc_TX_tail();
//                    #endif
//    
//                    //���÷��ͳ�ʱ����ֵ����ʱ�Ͳ��ٷ�������
//                    slave_rs485_comm_TX_delay_cnt_max = SLAVE_RS485_COMM_TX_DELAY_CNT_MAX_LONG;
//                    slave_rs485_run_state[ENUM_COMM_INDEX_COMM]=MASTER_RS485_RUN_TX;
//                    return 0;
//                }
            }
            break;
        case 4:
            if(sl_master_state.poll_charger_cmd_cnt > 1000)
            {
                //�ָ���ʱ��ѯ�����
                sl_master_state.poll_charger_cmd_cnt = 1000;
            }
            //CHARGER
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_DEV_VERSION;
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 3+sizeof(st_version_desc) + 14;  //data len
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[3] = 4; //target CHARGER
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[4] = pl_hub_cmd->stl_data[1]; //port
            if(gl_charger_dev_version.is_valid)
            {
                pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = ENUM_COMM_ERROR_CODE_OK; //error_code 
                memcpy(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[6], &gl_charger_dev_version.version_desc, sizeof(st_version_desc));
            }
            else
            {
                pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = ENUM_COMM_ERROR_CODE_NOTREADY; //error_code 
                memset(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[6], 0, sizeof(st_version_desc));
            }
            
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 6+sizeof(st_version_desc);

            if(gl_charger_dev_version.is_valid)
            {
                
                memcpy((char*)&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx], 
                    (char*)&gl_charger_dev_version.charger_sn, 
                    14);
            }
            else
            {
                memset((char*)&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx], 
                    0, 
                    14);
            }

            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx += 14;

            #if 0
            vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
            //crc
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
            #else
            slave_rs485_calc_TX_tail();
            #endif
            
            //���÷��ͳ�ʱ����ֵ����ʱ�Ͳ��ٷ�������
            slave_rs485_comm_TX_delay_cnt_max = SLAVE_RS485_COMM_TX_DELAY_CNT_MAX_LONG;
            slave_rs485_run_state[ENUM_COMM_INDEX_COMM]=MASTER_RS485_RUN_TX;
            return 0;
            break;
        default:
            break;
    }
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_DEV_VERSION;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 3+sizeof(st_version_desc);  //data len
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[3] = pl_hub_cmd->stl_data[0]; //target PMS
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[4] = pl_hub_cmd->stl_data[1]; //port
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = ENUM_COMM_ERROR_CODE_FAILED; //error_code 
    memset(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[6], 0, sizeof(st_version_desc));
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 6+sizeof(st_version_desc);

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    
    //���÷��ͳ�ʱ����ֵ����ʱ�Ͳ��ٷ�������
    slave_rs485_comm_TX_delay_cnt_max = SLAVE_RS485_COMM_TX_DELAY_CNT_MAX_LONG;
    slave_rs485_run_state[ENUM_COMM_INDEX_COMM]=MASTER_RS485_RUN_TX;
    return 0;
}



#define BOOTLOADER_SIZE         (4*1024)
#define BOOTLOADER_START_ADDR   (0)
//chenke
#define APP_DEVICE_INFO_START_ADDR BOOTLOADER_SIZE
#define APP_UPDATE_FLAG_ADDR (APP_DEVICE_INFO_START_ADDR+128)

static void slave_rs485_update_flag_mark(void)
{
    unsigned char vl_buff[128+6];
    unsigned int i;

    #ifdef UNMASK_CODE
    //�����ж�
     __disable_irq();
    
    //д������־
    // Initialize flash for writing.
    //FLASH_init();
    while (DMA->CHENS & DMA_CHENS_CH0ENS);
    memcpy(vl_buff, (unsigned char*)APP_DEVICE_INFO_START_ADDR, 128);
    memcpy(&vl_buff[128], "update", 6);
    FLASH_eraseOneBlock(APP_DEVICE_INFO_START_ADDR);
    while (DMA->CHENS & DMA_CHENS_CH0ENS);
    FLASH_writeBlock((void*)APP_DEVICE_INFO_START_ADDR, 0, 128+6, vl_buff);
    while (DMA->CHENS & DMA_CHENS_CH0ENS);
    for(i = 0; i < (128+6);)
    {
        FLASH_writeWord(APP_DEVICE_INFO_START_ADDR+i,*(uint32_t*)&vl_buff[i]);
        i+=4;
    }
    //�����ж�
    __enable_irq();
    #endif
}

bool slave_rs485_cmd_update_start(st_hub_cmd *pl_hub_cmd)
{
    unsigned char vl_bms_index;

    //����Ŀǰ�Ķ˿ں�
//    slave_rs485_cmd_update_port = pl_hub_cmd->stl_data[1];
    
    if((pl_hub_cmd->stl_data[0]) == ENUM_UPDATE_TAR_BMS_BOARD)
    {

        //�����ѯ�ڼ䣬ͬʱ��о��͵�ѹС��3V����ֹBMS����
        if((0 == sl_bat_poll_param.is_poll_allways_en_flag))// && // �����ѯ״̬
//            (sl_bat_poll_param.poll_interval_MAX >= BAT_POLL_BIG_INTERVAL_MAX)) // ��о��ѹС��3V
        {
            slave_rs485_cmd_update_cnf(pl_hub_cmd->stl_data[0], SLAVE_RS485_CMD_UPDATE_START, ENUM_COMM_ERROR_CODE_FAILED);
            return 0;
        }
        
        //�����Ч���ݱ�־
        gl_update_buf.data_valid_flag = 0;
    
        vl_bms_index = pl_hub_cmd->stl_data[1];

        slave_rs485_set_updating_bms_index(vl_bms_index);
        
        //ת���ַΪʵ�ʰ��ϵĵ�ַ
        vl_bms_index = slave_rs485_changed_bms_index_from_comm(vl_bms_index);
        
        if(vl_bms_index >= NFC_READER_COUNT_MAX)
        {
            slave_rs485_cmd_update_cnf(pl_hub_cmd->stl_data[0], SLAVE_RS485_CMD_UPDATE_START, ENUM_COMM_ERROR_CODE_FAILED);
            return 0;
        }
        

        //��λ�ܳ�ʱ����
        bms_xmodem_total_timeout_cnt_reload();

        //������ڼ����ѯ��ص�ʱ�䣬��������һֱ��ѯ10����
//        sl_bat_poll_param.poll_interval_for_bms_update_cnt = BAT_POLL_BMS_UPDATE_DELAY_MAX;
        
        //������״̬�����ڽ�����������
        gl_bms_update_state[vl_bms_index] = ENUM_BMS_UPDATE_CHECK_PROCESS;
        if(gl_bms_state_check[vl_bms_index] == ENUM_BMS_STATE_CHECK_IDLE)
           gl_bms_state_check[vl_bms_index] = ENUM_BMS_STATE_CHECK_REBOOT_BMS;
    }
    else if((pl_hub_cmd->stl_data[0]) == ENUM_UPDATE_TAR_PMS_BOARD)
    {

        //��ʶ������־
        slave_rs485_update_flag_mark();
        
        slave_rs485_cmd_update_cnf(pl_hub_cmd->stl_data[0], SLAVE_RS485_CMD_UPDATE_START, ENUM_COMM_ERROR_CODE_NOTREADY);

        //������ɺ�����
        slave_rs485_reboot_after_tx |= (1<< ENUM_COMM_INDEX_COMM);
    }
    else if((pl_hub_cmd->stl_data[0]) == ENUM_UPDATE_TAR_CHARGER)
    {
        if((FALSE == cmd_list_empty())
            ||(TRUE == master_rs485_is_485_error())
            ||(sl_master_state.cmd_processing_flag)
            ||(sl_master_state.cmd_timeout_cnt))
        {
            //unsigned short vl_crc;
            //Charger
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_UPDATE_START;
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 3;  //data len
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[3] = 4; //target Charger
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[4] = pl_hub_cmd->stl_data[1]; //port
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = ENUM_COMM_ERROR_CODE_NOTREADY; //error_code 
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 6;

            #if 0
            vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
            //crc
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
            #else
            slave_rs485_calc_TX_tail();
            #endif
            
            //���÷��ͳ�ʱ����ֵ����ʱ�Ͳ��ٷ�������
            slave_rs485_comm_TX_delay_cnt_max = SLAVE_RS485_COMM_TX_DELAY_CNT_MAX_LONG;
            slave_rs485_run_state[ENUM_COMM_INDEX_COMM]=MASTER_RS485_RUN_TX;
            return 0;
        }
        else
        {
            //�����485���ϵ����������ӳٷ���
            sl_master_state.normal_cmd_delay_when_chger_updating_cnt = 60000;
            
            //slave_rs485_get_rx_data_from_buf(ENUM_COMM_INDEX_COMM, &vl_data, &vl_data_len);
            //�Ӵ����л�ȡ�����ٷ��͵������
//            pl_rs485_rx[ENUM_COMM_INDEX_COMM]->rs485_rx_buf[0] = 1; //�������ַ��1
//            pl_rs485_rx[ENUM_COMM_INDEX_COMM]->rs485_rx_buf[4] = 0; //port����0
//            master_rs485_snd_data(&pl_rs485_rx[ENUM_COMM_INDEX_COMM]->rs485_rx_buf[0],pl_rs485_rx[ENUM_COMM_INDEX_COMM]->rs485_rx_indx-2);
            return 0;
        }
    }
    
    return 0;
}

/*
target: 
0: CTRL board(Port#������)
1: PMS(ָ��Port#�˿ںŵ�PMS)
2: BAT(ָ��Port#�˿ںŵĵ��)

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
    
    //���÷��ͳ�ʱ����ֵ����ʱ�Ͳ��ٷ�������
    slave_rs485_comm_TX_delay_cnt_max = SLAVE_RS485_COMM_TX_DELAY_CNT_MAX_LONG;
    slave_rs485_run_state[ENUM_COMM_INDEX_COMM]=MASTER_RS485_RUN_TX;
}



bool slave_rs485_cmd_update_exchange(st_hub_cmd *pl_hub_cmd)
{
    unsigned char vl_bms_index;
    //����Ŀǰ�Ķ˿ں�
//    slave_rs485_cmd_update_port = pl_hub_cmd->stl_data[1];
    
    if((pl_hub_cmd->stl_data[0]) == ENUM_UPDATE_TAR_BMS_BOARD)
    {         
        //vl_bms_index = (pl_hub_cmd->stl_data[0]>>4)&0x0F;
        //if(vl_bms_index >= NFC_READER_COUNT_MAX)
        //    return 0;
        vl_bms_index = slave_rs485_get_updating_bms_index();
        //ת���ַΪʵ�ʰ��ϵĵ�ַ
        vl_bms_index = slave_rs485_changed_bms_index_from_comm(vl_bms_index);
        
        if(0xFF == vl_bms_index)
        {
            slave_rs485_cmd_update_cnf(pl_hub_cmd->stl_data[0], SLAVE_RS485_CMD_UPDATE_EXCHANGE, ENUM_COMM_ERROR_CODE_FAILED);
            return 0;
        }

        //��λ�ܳ�ʱ����
        bms_xmodem_total_timeout_cnt_reload();
        
        if((gl_update_buf.data_valid_flag == 0)&&
            (gl_bms_update_state[vl_bms_index] == ENUM_BMS_UPDATE_XMODEM_PROCESS)&&
            (gl_xmodem_state[vl_bms_index] == ENUM_XMODEM_TRANS_DATA))
        {
            gl_update_buf.data_valid_flag = 1;
            //�������ݵ�����
            gl_update_buf.data_offset = pl_hub_cmd->stl_data[2];
            gl_update_buf.data_offset += (pl_hub_cmd->stl_data[3]<<8);
            gl_update_buf.data_offset += (pl_hub_cmd->stl_data[4]<<16);
            gl_update_buf.data_offset += (pl_hub_cmd->stl_data[5]<<24);
            memcpy(gl_update_buf.data_buf, &pl_hub_cmd->stl_data[6], 128);
            
        }
        else
        {
            //���ش���
            slave_rs485_cmd_update_cnf(pl_hub_cmd->stl_data[0], SLAVE_RS485_CMD_UPDATE_EXCHANGE, ENUM_COMM_ERROR_CODE_FAILED);
        }
    }
    else if ((pl_hub_cmd->stl_data[0]) == ENUM_UPDATE_TAR_PMS_BOARD)
    {
        //���ش���
        slave_rs485_cmd_update_cnf(pl_hub_cmd->stl_data[0], SLAVE_RS485_CMD_UPDATE_EXCHANGE, ENUM_COMM_ERROR_CODE_FAILED);
    }
    else if((pl_hub_cmd->stl_data[0]) == ENUM_UPDATE_TAR_CHARGER)
    {
        if((FALSE == cmd_list_empty())
            ||(TRUE == master_rs485_is_485_error())
            ||(sl_master_state.cmd_processing_flag)
            ||(sl_master_state.cmd_timeout_cnt))
        {
            //unsigned short vl_crc;
            //Charger
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_UPDATE_EXCHANGE;
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 3;  //data len
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[3] = 4; //target Charger
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[4] = pl_hub_cmd->stl_data[1]; //port
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = ENUM_COMM_ERROR_CODE_NOTREADY; //error_code 
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 6;

            #if 0
            vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
            //crc
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
            #else
            slave_rs485_calc_TX_tail();
            #endif
    
            //���÷��ͳ�ʱ����ֵ����ʱ�Ͳ��ٷ�������
            slave_rs485_comm_TX_delay_cnt_max = SLAVE_RS485_COMM_TX_DELAY_CNT_MAX_LONG;
            slave_rs485_run_state[ENUM_COMM_INDEX_COMM]=MASTER_RS485_RUN_TX;
            return 0;
        }
        else
        {
            //�����485���ϵ����������ӳٷ���
            sl_master_state.normal_cmd_delay_when_chger_updating_cnt = 60000;
            
            //slave_rs485_get_rx_data_from_buf(ENUM_COMM_INDEX_COMM, &vl_data, &vl_data_len);
            //�Ӵ����л�ȡ�����ٷ��͵������
//            pl_rs485_rx[ENUM_COMM_INDEX_COMM]->rs485_rx_buf[0] = 1; //�������ַ��1
//            pl_rs485_rx[ENUM_COMM_INDEX_COMM]->rs485_rx_buf[4] = 0; //port����0
//            master_rs485_snd_data(&pl_rs485_rx[ENUM_COMM_INDEX_COMM]->rs485_rx_buf[0],pl_rs485_rx[ENUM_COMM_INDEX_COMM]->rs485_rx_indx-2);
            return 0;
        }
    }
    return 0;
}

bool slave_rs485_cmd_update_done(st_hub_cmd *pl_hub_cmd)
{
//    unsigned char vl_bms_index;
//    //����Ŀǰ�Ķ˿ں�
//    slave_rs485_cmd_update_port = pl_hub_cmd->stl_data[1];
//    
//    if((pl_hub_cmd->stl_data[0]) == ENUM_UPDATE_TAR_BMS_BOARD)
//    {
//        vl_bms_index = slave_rs485_get_updating_bms_index();

//        //ת���ַΪʵ�ʰ��ϵĵ�ַ
//        vl_bms_index = slave_rs485_changed_bms_index_from_comm(vl_bms_index);
//        
//        if(0xFF == vl_bms_index)
//        {
//            slave_rs485_cmd_update_cnf(pl_hub_cmd->stl_data[0], SLAVE_RS485_CMD_UPDATE_DONE, ENUM_COMM_ERROR_CODE_FAILED);
//            return 0;
//        }

//        //��λ�ܳ�ʱ����
//        bms_xmodem_total_timeout_cnt_reload();
//            
//        if(gl_bms_update_state[vl_bms_index] == ENUM_BMS_UPDATE_XMODEM_PROCESS)
//        {
//            //���سɹ�
//            gl_xmodem_state[vl_bms_index] = ENUM_XMODEM_END;
//            gl_update_buf.data_valid_flag = 0;
//        }
//        else
//        {
//            //���ش���
//            slave_rs485_cmd_update_cnf(pl_hub_cmd->stl_data[0], SLAVE_RS485_CMD_UPDATE_DONE, ENUM_COMM_ERROR_CODE_FAILED);
//        }
//    }
//    else if ((pl_hub_cmd->stl_data[0]) == ENUM_UPDATE_TAR_PMS_BOARD)
//    {
//        //���ش���
//        slave_rs485_cmd_update_cnf(pl_hub_cmd->stl_data[0], SLAVE_RS485_CMD_UPDATE_DONE, ENUM_COMM_ERROR_CODE_FAILED);
//    }
//    else if((pl_hub_cmd->stl_data[0]) == ENUM_UPDATE_TAR_CHARGER)
//    {
//        if((FALSE == cmd_list_empty())
//            ||(TRUE == master_rs485_is_485_error())
//            ||(sl_master_state.cmd_processing_flag)
//            ||(sl_master_state.cmd_timeout_cnt))
//        {
//            //unsigned short vl_crc;
//            //Charger
//            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
//            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_UPDATE_EXCHANGE;
//            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 3;  //data len
//            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[3] = 4; //target Charger
//            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[4] = pl_hub_cmd->stl_data[1]; //port
//            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = ENUM_COMM_ERROR_CODE_NOTREADY; //error_code 
//            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 6;

//            #if 0
//            vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
//            //crc
//            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
//            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
//            #else
//            slave_rs485_calc_TX_tail();
//            #endif
//            
//            //���÷��ͳ�ʱ����ֵ����ʱ�Ͳ��ٷ�������
//            slave_rs485_comm_TX_delay_cnt_max = SLAVE_RS485_COMM_TX_DELAY_CNT_MAX_LONG;
//            slave_rs485_run_state[ENUM_COMM_INDEX_COMM]=MASTER_RS485_RUN_TX;
//            return 0;
//        }
//        else
//        {
//            //�����485���ϵ����������ӳٷ���
//            sl_master_state.normal_cmd_delay_when_chger_updating_cnt = 3000;
//            
//            //slave_rs485_get_rx_data_from_buf(ENUM_COMM_INDEX_COMM, &vl_data, &vl_data_len);
//            //�Ӵ����л�ȡ�����ٷ��͵������
//            pl_rs485_rx[ENUM_COMM_INDEX_COMM]->rs485_rx_buf[0] = 1; //�������ַ��1
//            pl_rs485_rx[ENUM_COMM_INDEX_COMM]->rs485_rx_buf[4] = 0; //port����0
//            master_rs485_snd_data(&pl_rs485_rx[ENUM_COMM_INDEX_COMM]->rs485_rx_buf[0],pl_rs485_rx[ENUM_COMM_INDEX_COMM]->rs485_rx_indx-2);
//            return 0;
//        }
//    }
//    return 0;
}

bool slave_rs485_cmd_cfg(st_hub_cmd *pl_hub_cmd)
{
    unsigned char vl_bms_index;
    unsigned short vl_crc;

    if(pl_hub_cmd->stl_dataL < 5)
    {
        return 0;
    }
    vl_bms_index = pl_hub_cmd->stl_data[1];

    //ת���ַΪʵ�ʰ��ϵĵ�ַ
    vl_bms_index = slave_rs485_changed_bms_index_from_comm(vl_bms_index);
    
    if(vl_bms_index >= NFC_READER_COUNT_MAX)
    {
        return 0;
    }

    
    // ����CRC�ı����ݴ�������������ֵ������
    vl_crc = pl_hub_cmd->stl_data[2]|(pl_hub_cmd->stl_data[3]<<8);

    if((4 <= vl_crc)&& (vl_crc <= 70))
    {
        master_rs485_set_chg_current_max_value(vl_crc);    
    }

    #ifdef UNMASK_CODE
    if(pl_hub_cmd->stl_data[4] <= 100)
    {
        main_led_set_SOC_full_threshold(vl_bms_index, pl_hub_cmd->stl_data[4]);
    }
    #endif

    if(pl_hub_cmd->stl_dataL >= 7)
    {
        //���ó�����ĳ���ѹ
        // ����CRC�ı����ݴ�����������ѹֵ������
        vl_crc = pl_hub_cmd->stl_data[5]|(pl_hub_cmd->stl_data[6]<<8);
        if(vl_crc <= 672)
        {
            master_rs485_set_chg_voltage_max_value(vl_crc);    
        }
    }

    if(pl_hub_cmd->stl_dataL >= 8)
    {
        #ifdef UNMASK_CODE
        if(pl_hub_cmd->stl_data[7]&0x01)
        {
            //���������־
            main_clear_pms_restart_flag();
        }

        if(pl_hub_cmd->stl_data[7]&(0x01<<1))
        {
            //��������"�����¶ȶ�̬���"���ܣ�
            if(FALSE == main_get_temp_charge_enable_flag())
            {
                main_set_temp_charge_enable_flag(TRUE);
            }
        }
        else
        {
            if(TRUE == main_get_temp_charge_enable_flag())
            {
                main_set_temp_charge_enable_flag(FALSE);
            }
        }
        #endif
    }
    
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_CFG;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x07;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[3] = pl_hub_cmd->stl_data[0];
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[4] = pl_hub_cmd->stl_data[1];
    // ����CRC�ı����ݴ�������������ֵ������
    vl_crc = master_rs485_get_chg_current_max_value();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[6] = (vl_crc>>8)&0xFF;
//    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[7] = main_led_get_SOC_full_threshold(vl_bms_index);
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 8;
    if(pl_hub_cmd->stl_dataL >= 7)
    {
        // ����CRC�ı����ݴ�����������ѹֵ������
        vl_crc = master_rs485_get_chg_voltage_max_value();
        pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
        pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    } 

    if(pl_hub_cmd->stl_dataL >= 8)
    {
        #ifdef UNMASK_CODE
        pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx] = 0;
        if(main_get_pms_restart_flag())
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx] |= 1;
        if(main_get_temp_charge_enable_flag())
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx] |= (1<<1);
        #endif
        pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++;
    }

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    
    return 1;
}

bool slave_rs485_cmd_read_bat_record(st_hub_cmd *pl_hub_cmd)
{
    //unsigned short vl_crc;

    if(pl_hub_cmd->stl_dataL < 4)
    {
        return 0;
    }

    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_READ_BAT_RECORD;
    
    // Target
    //pl_hub_cmd->stl_data[0];
    // Port
    //pl_hub_cmd->stl_data[1];
    // Data Type :0x02(��ʾ�����ʷ��¼)
    if(2 == pl_hub_cmd->stl_data[2])
    {
        // Data Len
        if(pl_hub_cmd->stl_data[3] == 20)
        {
            unsigned int vl_log_head;
            unsigned char* vp_log_head_datatime;
            unsigned int vl_log_cnt;
            unsigned int vl_log_read_cnt;
            unsigned char vl_bat_ID[6];
            
            //bat ID
            memcpy(vl_bat_ID, &pl_hub_cmd->stl_data[4], 6);
            //log HEAD
            vl_log_head = pl_hub_cmd->stl_data[10]
                        + (pl_hub_cmd->stl_data[11] << 8)
                        + (pl_hub_cmd->stl_data[12] << 16)
                        + (pl_hub_cmd->stl_data[13] << 24);
            //log HEAD datetime
            //6bytes
            vp_log_head_datatime = &pl_hub_cmd->stl_data[14];
            
            //log NO
            vl_log_cnt = pl_hub_cmd->stl_data[20]
                        + (pl_hub_cmd->stl_data[21] << 8)
                        + (pl_hub_cmd->stl_data[22] << 16)
                        + (pl_hub_cmd->stl_data[23] << 24);
            
            // ���ص����ݰ������ݳ���
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x03;
            
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[3] = pl_hub_cmd->stl_data[0];
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[4] = pl_hub_cmd->stl_data[1];
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = 0; //0x00:�յ����������ȡ����

            //ʱ��Ƚ�
            if(gl_bat_history_record_param.is_earliest_param_valid && gl_bat_history_record_param.is_lastest_param_valid)
            {
                //��ǰҪ��ȡ��index = head+read_cnt+1;
                if(0 == vl_log_cnt)
                {
                    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = 0x02; //0x02:�Ѿ����굱ǰָ����Ҫ��������
                }
//                else if(-1 == Battery_history_record_datetime_compare(vp_log_head_datatime, gl_bat_history_record_param.history_record_earliest_index_datetime))
//                {
//                    // Tb<Ts; //���earliest index��ʼ
//                    if(0xFFFFFFFF == vl_log_cnt)
//                    {   
//                        #if 1
//                        //��ʱ����λ���·���ʱ�����ŵ���ʵʱ�䲻ƥ�䣬�����߼�����

////                        if(TRUE == Battery_history_record_is_index_within_range(Battery_history_record_get_real_index(vl_log_head+1)))
////                        {
////                            //����·��ı���ڷ�Χ��
////                            vl_log_read_cnt = 0;
////                        }
////                        else
////                        {
////                            //���ڷ�Χ�ڣ��ӵ����ʷ��¼��ʼ��ſ�ʼ
////                            vl_log_read_cnt = 0;
////                            vl_log_head = gl_bat_history_record_param.history_record_earliest_index - 1;
////                        }
//                        pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = 0x00; //0x00:�յ����������ȡ����
//                        #else
//                        //vl_log_read_cnt = gl_bat_history_record_param.history_record_earliest_index - (vl_log_head + 1);
//                        vl_log_read_cnt = Battery_history_record_get_real_diff_value((vl_log_head + 1), gl_bat_history_record_param.history_record_earliest_index);
//                        pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = 0x00; //0x00:�յ����������ȡ����
//                        #endif
//                    }
//                    else
//                    {
//                        if((gl_bat_history_record_param.history_record_earliest_index - (vl_log_head+1)) >= vl_log_cnt)
//                        {
//                            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = 0x02; //0x02:�Ѿ����굱ǰָ����Ҫ��������
//                        }
//                        else
//                        {
//                            // ĩβ���ֵļ�¼�ڿ�ʼ��ź���
//                            //vl_log_read_cnt = gl_bat_history_record_param.history_record_earliest_index - (vl_log_head+1);
//                            vl_log_read_cnt = Battery_history_record_get_real_diff_value((vl_log_head + 1), gl_bat_history_record_param.history_record_earliest_index);
//                            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = 0x00; //0x00:�յ����������ȡ����
//                        }
//                    }
//                    
//                }
//                else if(-1 == Battery_history_record_datetime_compare(gl_bat_history_record_param.history_record_lastest_index_datetime, vp_log_head_datatime))
//                {
//                    // Tb>Te
//                    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = 0x03; //0x03:�Ѿ���ȡ�����е������
//                }
//                else if(0 == Battery_history_record_datetime_compare(vp_log_head_datatime, gl_bat_history_record_param.history_record_earliest_index_datetime))
//                {
//                    //Tb==Ts
//                    if(TRUE == Battery_history_record_is_within_range(gl_bat_history_record_param.history_record_earliest_index, 
//                                                            gl_bat_history_record_param.history_record_lastest_index, 
//                                                            vl_log_head+1))
//                    {
//                        vl_log_read_cnt = 0;
//                        pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = 0x00; //0x00:�յ����������ȡ����
//                    }
//                    else
//                    {
//                        // ĩβ���ֵļ�¼�ڿ�ʼ��ź���
//                        //vl_log_read_cnt = gl_bat_history_record_param.history_record_earliest_index - (vl_log_head+1);
//                        vl_log_read_cnt = Battery_history_record_get_real_diff_value((vl_log_head + 1), gl_bat_history_record_param.history_record_earliest_index);
//                        pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = 0x00; //0x00:�յ����������ȡ����
//                    }
//                }
//                else if(0 == Battery_history_record_datetime_compare(gl_bat_history_record_param.history_record_lastest_index_datetime, vp_log_head_datatime))
//                {
//                    // Tb==Te
//                    if(TRUE == Battery_history_record_is_within_range(gl_bat_history_record_param.history_record_earliest_index, 
//                                                            gl_bat_history_record_param.history_record_lastest_index, 
//                                                            vl_log_head+1))
//                    {
//                        vl_log_read_cnt = 0;
//                        pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = 0x00; //0x00:�յ����������ȡ����
//                    }
//                    else
//                    {
//                        pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = 0x03; //0x03:�Ѿ���ȡ�����е������
//                    }
//                }
                else
                {
                    // Ts<Tb<Te
                    #if 1
                    //��ʱ����λ���·���ʱ�����ŵ���ʵʱ�䲻ƥ�䣬�����߼�����

//                    if(TRUE == Battery_history_record_is_index_within_range(Battery_history_record_get_real_index(vl_log_head+1)))
//                    {
//                        //����·��ı���ڷ�Χ��
//                        vl_log_read_cnt = 0;
//                    }
//                    else
//                    {
//                        //���ڷ�Χ�ڣ��ӵ����ʷ��¼��ʼ��ſ�ʼ
//                        vl_log_read_cnt = 0;
//                        vl_log_head = gl_bat_history_record_param.history_record_earliest_index - 1;
//                    }
                    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = 0x00; //0x00:�յ����������ȡ����
                    #else
                    vl_log_read_cnt = 0;
                    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = 0x00; //0x00:�յ����������ȡ����
                    #endif
                }
            }
            else
            {
                if(0xFFFFFFFF == vl_log_cnt)
                {
                    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = 0x03; //0x03:�Ѿ���ȡ�����е������
                }
                else
                {
                    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = 0x02; //0x02:�Ѿ����굱ǰָ����Ҫ��������
                }
            }
            
            if(0 == pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5])
            {
                //������ʷ���ݵĿ�ʼ��ź�����
//                Battery_history_record_set_index_param(vl_bat_ID, vl_log_head, vp_log_head_datatime, vl_log_cnt, vl_log_read_cnt);
            }
            
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 6;
            
        }
        else if(pl_hub_cmd->stl_data[3] == 6)
        {
            unsigned char vl_bat_ID[6];

            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[3] = pl_hub_cmd->stl_data[0];
            pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[4] = pl_hub_cmd->stl_data[1];
                
            //bat ID
            memcpy(vl_bat_ID, &pl_hub_cmd->stl_data[4], 6);
            //�ش���ȡ������ʷ����
//            if(Battery_history_record_get_data(vl_bat_ID, &pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[14]))
            
            {
                /*
                            0x02:�Ѿ����굱ǰָ����Ҫ��������
                            0x03:�Ѿ���ȡ�����е������
                            0x04:��ȡʧ�ܣ��й��ϣ�
                            */
                pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = 0x04; //0x04:��ȡʧ�ܣ��й��ϣ�
                pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 6;

                // ���ص����ݰ������ݳ���
                pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x03;
            }
        }
    }

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    
    return 1;
}





bool slave_rs485_cmd_set_SN(st_hub_cmd *pl_hub_cmd)
{
    //unsigned short vl_crc;
    if(pl_hub_cmd->stl_dataL < 0x10)
    {
        return 0;
    }

    if((1 != pl_hub_cmd->stl_data[0]) || ((slave_rs485_get_dev_addr() - 1) != pl_hub_cmd->stl_data[1]))
    {
        //��������PMS SN
        return 0;
    }

    #ifdef UNMASK_CODE
    main_set_PMS_SN(&pl_hub_cmd->stl_data[2]);
    #endif

    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0] = slave_rs485_get_dev_addr();
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[1] = SLAVE_RS485_CMD_SN_SETTING;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[2] = 0x03;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[3] = 0x01;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[4] = (slave_rs485_get_dev_addr() - 1);
    #ifdef UNMASK_CODE
    if(TRUE == main_is_setting_PMS_param_OK()) //������Ҫȷ���Ƿ��Ѿ�д��FLASH��
    {
        //�ɹ�
        pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = 0;
    }
    else
    {
        //ʧ��
        pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[5] = 1;
    }
    #endif

    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx = 6;

    #if 0
    vl_crc = slave_rs485_calc_CRC(&pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[0], pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx);
    //crc
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = vl_crc&0xFF;
    pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_buf[pl_rs485_tx[ENUM_COMM_INDEX_COMM]->rs485_tx_indx++] = (vl_crc>>8)&0xFF;
    #else
    slave_rs485_calc_TX_tail();
    #endif
    
    return 1;
}



void slave_rs485_cmd_update_return_error(unsigned char bms_index, unsigned char cmd, enum_comm_error_code error_code)
{
    gl_xmodem_state[bms_index] = ENUM_XMODEM_IDLE;
    gl_bms_state_check[bms_index] = ENUM_BMS_STATE_CHECK_IDLE;
    gl_bms_update_state[bms_index] = ENUM_BMS_UPDATE_IDLE;
    slave_rs485_cmd_update_cnf(2, cmd, error_code); //������BMS���������Թ̶�target=2
}


//����485��ַת��0~(NFC_READER_COUNT_MAX-1)��ΧΪcomm��ʶ��ķ�Χ
static unsigned char slave_rs485_changed_bms_index_to_comm(unsigned char bms_index)
{
    unsigned char vl_base_addr;
    vl_base_addr = ((slave_rs485_get_dev_addr() - 1) * NFC_READER_COUNT_MAX);
    return (vl_base_addr + bms_index);
}


////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////

//��ʶ�Ƿ����°汾�ĳ����������ɰ汾��֧��Զ��������
static unsigned char sl_master_chg_new_protocol = 1;






static bool master_rs485_cmd_get_dev_version_cnf(st_hub_cmd *pl_hub_cmd);
static bool master_rs485_cmd_update_start_cnf(st_hub_cmd *pl_hub_cmd);
static bool master_rs485_cmd_update_exchange_cnf(st_hub_cmd *pl_hub_cmd);
static bool master_rs485_cmd_update_done_cnf(st_hub_cmd *pl_hub_cmd);
static void master_rs485_setup_charger_param(unsigned char is_charger_id_ctrled);











///////////////////////////////////////////////////////////////////
//�����������ȫ�ֱ������ơ�
//����һ���洢������ȫ�ֱ���
//st_poll_cmd_param gl_poll_cmd_param;

// ��������
static unsigned short sl_master_rs485_chg_current_max_value;
#define MASTER_RS485_CHG_CURRENT_MAX_DEFAULT 70 // 0.1AΪ��λ

static unsigned short sl_master_rs485_chg_voltage_max_value;
#define MASTER_RS485_CHG_VOLTAGE_MAX_DEFAULT 672 // 0.1VΪ��λ

///////////////////////////////////////////////////////////////////












unsigned short master_rs485_get_chg_current_max_value(void)
{
    /*return sl_master_rs485_chg_current_max_value;*/
    return 1;//gl_pms_param.OD_charging_current_max;
}

void master_rs485_set_chg_current_max_value(unsigned short value)
{
    /*sl_master_rs485_chg_current_max_value = value;*/
//    gl_pms_param.OD_charging_current_max = value;
}

unsigned short master_rs485_get_chg_voltage_max_value(void)
{
    /*return sl_master_rs485_chg_voltage_max_value;*/
    return 1;//gl_pms_param.OD_charging_voltage_max;
}

void master_rs485_set_chg_voltage_max_value(unsigned short value)
{
    /*sl_master_rs485_chg_voltage_max_value = value;*/
//    gl_pms_param.OD_charging_voltage_max = value; 
}

void master_rs485_set_chg_voltage_max_value_default(void)
{
//    gl_pms_param.OD_charging_voltage_max = MASTER_RS485_CHG_VOLTAGE_MAX_DEFAULT;
}




void master_rs485_snd_cmd_poll(unsigned char* data)
{
    unsigned char *pl_data;
    unsigned short vl_index;
    unsigned short vl_crc;

//////////////////////////////////////////////////////////////
    //����ֻ����һ��
    static unsigned char vl_debug_flag = 1;
    if(vl_debug_flag)
    {
        vl_debug_flag = 0;
        master_rs485_snd_MCU_restart_debug_info();
        return ;
    }
//////////////////////////////////////////////////////////////

    
    pl_data = &pl_rs485_tx[ENUM_COMM_INDEX_MASTER]->rs485_tx_buf[0];
    
    vl_index = 0;
    pl_data[0] = 0x01;
    pl_data[1] = ENUM_M_CMD_INFO;
    pl_data[2] = 16;//����������ݳ��ȹ̶�
    memcpy(&pl_data[3], data, 16);
    vl_index = 3+16;

    vl_crc = slave_rs485_calc_CRC(pl_data, vl_index);
    //crc
    pl_data[vl_index++] = vl_crc&0xFF;
    pl_data[vl_index++] = (vl_crc>>8)&0xFF;
   
    pl_rs485_tx[ENUM_COMM_INDEX_MASTER]->rs485_tx_indx = vl_index;
    slave_rs485_run_state[ENUM_COMM_INDEX_MASTER]=MASTER_RS485_RUN_TX;
}

//���﷢�Ϳ��������ĵ�����Ϣ(���ó������485��)
void master_rs485_snd_MCU_restart_debug_info(void)
{
    char *pl_data;
    pl_data = (char *)&(pl_rs485_tx[ENUM_COMM_INDEX_MASTER]->rs485_tx_buf[0]);
    
    strcpy(pl_data, "MCU Restart!");
    pl_rs485_tx[ENUM_COMM_INDEX_MASTER]->rs485_tx_indx = 12;
    slave_rs485_run_state[ENUM_COMM_INDEX_MASTER]=MASTER_RS485_RUN_TX;
}




bool master_rs485_is_485_error(void)
{
    return (sl_master_state.chg_485_cmd_poll_faild_cnt >= MASTER_485_CMD_POLL_ERROR_CNT_MAX)?TRUE:FALSE;
}
bool master_rs485_is_charger_OK(void)
{
    if(gl_poll_cmd_cnf_param.charger_OAP_flag||
            gl_poll_cmd_cnf_param.charger_OTP_flag||
            gl_poll_cmd_cnf_param.charger_OVP_flag||
            gl_poll_cmd_cnf_param.charger_UAP_flag||
            gl_poll_cmd_cnf_param.charger_UVP_flag||
            gl_poll_cmd_cnf_param.bit5_reserv||
            gl_poll_cmd_cnf_param.charger_fan_error_flag ||
            gl_poll_cmd_cnf_param.charger_low_AC_supply_flag ||
            (sl_master_state.chg_485_cmd_poll_faild_cnt >= MASTER_485_CMD_POLL_ERROR_CNT_MAX)
//            ||
//            (0 != gpio_input_bit_get(BATTERY_CHARGER_TEST_PORT, BATTERY_CHARGER_TEST_PIN)) // ID��û�м�⵽����
            )
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}

bool master_rs485_is_charger_status_changed(void)
{
    static bool sl_charger_status = FALSE;
    if(sl_charger_status != master_rs485_is_charger_OK())
    {
        sl_charger_status = master_rs485_is_charger_OK();
        return TRUE;
    }
    
    return FALSE;
}

bool master_rs485_is_charge_en(void)
{
    return (gl_poll_cmd_param.chg_en?1:0);
}

static void master_rs485_open_charger_by_cabinet_warmor(void)
{
    unsigned short vl_short_tmp;

    //�жϰ汾��֮�����ʹ���ĸ�����������
    if(gl_charger_dev_version.is_valid)
    {
        if((gl_charger_dev_version.version_desc.HW_main_ver == 2)
            && (gl_charger_dev_version.version_desc.HW_sub_ver == 1)
            && (gl_charger_dev_version.version_desc.FW_main_ver == 2)
            && (gl_charger_dev_version.version_desc.FW_sub_ver == 1)
            && (gl_charger_dev_version.version_desc.FW_revised_ver == 1)
            && (20190320 > (gl_charger_dev_version.version_desc.FW_build_ver[0] + 
                (gl_charger_dev_version.version_desc.FW_build_ver[1] << 8) +
                (gl_charger_dev_version.version_desc.FW_build_ver[2] << 16) +
                (gl_charger_dev_version.version_desc.FW_build_ver[3] << 24))))
        {
            //�챦�ɰ汾�̼��ĳ������ʹ�����ϻ�ģʽ����
            //master_rs485_setup_charger_param(0);
            gl_poll_cmd_param.chg_en = 1;

            vl_short_tmp = 70;    
            
            gl_poll_cmd_param.expect_current[0] = vl_short_tmp&0xFF;
            gl_poll_cmd_param.expect_current[1] = (vl_short_tmp>>8)&0xFF;

            
            vl_short_tmp = 550;
            gl_poll_cmd_param.expect_voltage[0] = vl_short_tmp&0xFF;
            gl_poll_cmd_param.expect_voltage[1] = (vl_short_tmp>>8)&0xFF;

            gl_poll_cmd_param.bat_temp = 0;

            // �����������λ1mAh��(ԭ���ĵ�λ0.1mAh��2���ֽڵ�short�ͱ����д治��)
            vl_short_tmp = 1;//10000;//�����ϻ�ģʽָ����������� = 1 ��Ƶ�ѹ = 1 
            gl_poll_cmd_param.rated_cap[0] = vl_short_tmp&0xFF;
            gl_poll_cmd_param.rated_cap[1] = (vl_short_tmp>>8)&0xFF;

            vl_short_tmp = 1;// 600; //�����ϻ�ģʽָ����������� = 1 ��Ƶ�ѹ = 1 
            gl_poll_cmd_param.rated_voltage[0] = vl_short_tmp&0xFF;
            gl_poll_cmd_param.rated_voltage[1] = (vl_short_tmp>>8)&0xFF;
        }
        else
        {
            gl_poll_cmd_param.chg_en = 2;// 0: ��ֹ��磻1: �����磻2:����ģʽ

            vl_short_tmp = 70;    
            gl_poll_cmd_param.expect_current[0] = vl_short_tmp&0xFF;
            gl_poll_cmd_param.expect_current[1] = (vl_short_tmp>>8)&0xFF;
            
            vl_short_tmp = 550;
            gl_poll_cmd_param.expect_voltage[0] = vl_short_tmp&0xFF;
            gl_poll_cmd_param.expect_voltage[1] = (vl_short_tmp>>8)&0xFF;

//            gl_poll_cmd_param.bat_temp = Battery_get_bat_temp(0);

            // �����������λ1mAh��(ԭ���ĵ�λ0.1mAh��2���ֽڵ�short�ͱ����д治��)
//            vl_short_tmp = Battery_get_bat_rated_cap(0);
            gl_poll_cmd_param.rated_cap[0] = vl_short_tmp&0xFF;
            gl_poll_cmd_param.rated_cap[1] = (vl_short_tmp>>8)&0xFF;

//            vl_short_tmp = Battery_get_bat_rated_volt(0);
//            gl_poll_cmd_param.rated_voltage[0] = vl_short_tmp&0xFF;
//            gl_poll_cmd_param.rated_voltage[1] = (vl_short_tmp>>8)&0xFF;
        }
    }
    else if(0 == sl_master_chg_new_protocol)
    {
        //�챦�ɰ汾�̼��ĳ������ʹ�����ϻ�ģʽ����
        //master_rs485_setup_charger_param(0);
        gl_poll_cmd_param.chg_en = 1;

        vl_short_tmp = 70;    
        
        gl_poll_cmd_param.expect_current[0] = vl_short_tmp&0xFF;
        gl_poll_cmd_param.expect_current[1] = (vl_short_tmp>>8)&0xFF;

        
        vl_short_tmp = 550;
        gl_poll_cmd_param.expect_voltage[0] = vl_short_tmp&0xFF;
        gl_poll_cmd_param.expect_voltage[1] = (vl_short_tmp>>8)&0xFF;

        gl_poll_cmd_param.bat_temp = 0;

        // �����������λ1mAh��(ԭ���ĵ�λ0.1mAh��2���ֽڵ�short�ͱ����д治��)
        vl_short_tmp = 1;//10000;//�����ϻ�ģʽָ����������� = 1 ��Ƶ�ѹ = 1 
        gl_poll_cmd_param.rated_cap[0] = vl_short_tmp&0xFF;
        gl_poll_cmd_param.rated_cap[1] = (vl_short_tmp>>8)&0xFF;

        vl_short_tmp = 1;// 600; //�����ϻ�ģʽָ����������� = 1 ��Ƶ�ѹ = 1 
        gl_poll_cmd_param.rated_voltage[0] = vl_short_tmp&0xFF;
        gl_poll_cmd_param.rated_voltage[1] = (vl_short_tmp>>8)&0xFF;
    }
}

//��ȡ���ó������������������; ��λ��0.1A
unsigned short master_rs485_get_expect_current(void)
{
    return (gl_poll_cmd_param.expect_current[0]|(gl_poll_cmd_param.expect_current[1] << 8));
}

//���ó��������
//is_charger_id_ctrled: 0, �ں����ﲻ����ID��
static void master_rs485_setup_charger_param(unsigned char is_charger_id_ctrled)
{
    unsigned short vl_short_tmp;
    unsigned short vl_current_tmp;
    gl_poll_cmd_param.chg_en = 1;
//    vl_short_tmp = Battery_get_max_chg_voltage(0);
    if(vl_short_tmp > master_rs485_get_chg_voltage_max_value())
    {
        //��ѹ����λ�����õĵ�ѹ(��ʼ��Ĭ����67.2V)
        vl_short_tmp = master_rs485_get_chg_voltage_max_value();
    }
    #ifdef PMS_AUTO_INCREASE_SETTING_VOLTAGE_ENABLE
    Battery_set_charger_setting_voltage(vl_short_tmp*10);
    if(4 >= Battery_get_charger_setting_voltage_offset()) //�������ɼ�0.4V
    {
        vl_short_tmp += Battery_get_charger_setting_voltage_offset();
    }
    #endif
    
    gl_poll_cmd_param.expect_voltage[0] = vl_short_tmp&0xFF;
    gl_poll_cmd_param.expect_voltage[1] = (vl_short_tmp>>8)&0xFF;

//    vl_short_tmp = Battery_get_max_chg_current(0);
    vl_current_tmp = master_rs485_get_chg_current_max_value();
//    if((sl_master_chg_new_protocol)//�ܶ����汾�ĳ�����ſ���ת��9A���
//        &&(540 > Battery_get_bat_rated_volt(0)))
//    {
//        //48V��ء���Ҫת��һ����λ��������ֵ��
//        //vl_current_tmp *= (9/7);
//        vl_current_tmp *= 9;
//        vl_current_tmp /=7;
//        
//    }
    if(vl_short_tmp > vl_current_tmp)
    {
        //��������λ�����õ�ֵ(��ʼ��Ĭ����1.8A)
        vl_short_tmp = vl_current_tmp;
    }

    #ifdef BAT_TEMP_CHARGER_ENABLE
    if(1 == Battery_is_bat_chg_enable_by_temp())
    {
        unsigned short vl_rated_cap;
        vl_rated_cap = Battery_get_bat_rated_cap(0);
        vl_rated_cap /= 100;// ��λ0.1A
        vl_rated_cap /=10; // 0.1C
        //����¶ȵͣ�������0.1C
        if(vl_short_tmp > vl_rated_cap)
        {
            vl_short_tmp = vl_rated_cap;
        }
    }
    #endif

    #ifdef PMS_AUTO_INCREASE_SETTING_VOLTAGE_ENABLE
    Battery_set_charger_setting_current(vl_short_tmp*10);
    #endif
    gl_poll_cmd_param.expect_current[0] = vl_short_tmp&0xFF;
    gl_poll_cmd_param.expect_current[1] = (vl_short_tmp>>8)&0xFF;

    //���Ҫ��ĳ�����Ϊ0����ֹͣ��������
    if(vl_short_tmp == 0)
    {
        gl_poll_cmd_param.chg_en = 0;
        if(is_charger_id_ctrled)
        {
            //�������������
//            main_charger_stop_by_id();
        }
    }

//    gl_poll_cmd_param.bat_temp = Battery_get_bat_temp(0);

    // �����������λ1mAh��(ԭ���ĵ�λ0.1mAh��2���ֽڵ�short�ͱ����д治��)
//    vl_short_tmp = Battery_get_bat_rated_cap(0);
    gl_poll_cmd_param.rated_cap[0] = vl_short_tmp&0xFF;
    gl_poll_cmd_param.rated_cap[1] = (vl_short_tmp>>8)&0xFF;

//    vl_short_tmp = Battery_get_bat_rated_volt(0);
    gl_poll_cmd_param.rated_voltage[0] = vl_short_tmp&0xFF;
    gl_poll_cmd_param.rated_voltage[1] = (vl_short_tmp>>8)&0xFF;
    
    memset(gl_poll_cmd_param.reserve, 0, 6);
}

st_charger_test_param sl_charger_test_param = {0, 0, 0, 0};

/*
*return: TRUE,��ʾ���ڽ��в���
*/
static bool master_rs485_charger_test_process(void)
{
    if(sl_charger_test_param.charger_test_enable_flag)
    {
        unsigned short vl_short_tmp;

        // �����������λ1mAh��
        vl_short_tmp = 15000;
        gl_poll_cmd_param.rated_cap[0] = vl_short_tmp&0xFF;
        gl_poll_cmd_param.rated_cap[1] = (vl_short_tmp>>8)&0xFF;

        vl_short_tmp = 600;
        gl_poll_cmd_param.rated_voltage[0] = vl_short_tmp&0xFF;
        gl_poll_cmd_param.rated_voltage[1] = (vl_short_tmp>>8)&0xFF;
        
        gl_poll_cmd_param.bat_temp = 10;
        if((sl_charger_test_param.charger_cmd_enable_flag)&&(sl_charger_test_param.charger_id_enable_flag))
        {
            gl_poll_cmd_param.chg_en = 1;// 0: ��ֹ��磻1: �����磻2:����ģʽ

            vl_short_tmp = 70;    
            gl_poll_cmd_param.expect_current[0] = vl_short_tmp&0xFF;
            gl_poll_cmd_param.expect_current[1] = (vl_short_tmp>>8)&0xFF;
            
            vl_short_tmp = 550;
            gl_poll_cmd_param.expect_voltage[0] = vl_short_tmp&0xFF;
            gl_poll_cmd_param.expect_voltage[1] = (vl_short_tmp>>8)&0xFF;            
        }
        else if((sl_charger_test_param.charger_cmd_enable_flag)&&(0 == sl_charger_test_param.charger_id_enable_flag))
        {
            gl_poll_cmd_param.chg_en = 0;// 0: ��ֹ��磻1: �����磻2:����ģʽ
//            main_charger_stop_by_id();

            vl_short_tmp = 70;    
            gl_poll_cmd_param.expect_current[0] = vl_short_tmp&0xFF;
            gl_poll_cmd_param.expect_current[1] = (vl_short_tmp>>8)&0xFF;
            
            vl_short_tmp = 550;
            gl_poll_cmd_param.expect_voltage[0] = vl_short_tmp&0xFF;
            gl_poll_cmd_param.expect_voltage[1] = (vl_short_tmp>>8)&0xFF;
        }
        else if((0 == sl_charger_test_param.charger_cmd_enable_flag)&&(sl_charger_test_param.charger_id_enable_flag))
        {
            gl_poll_cmd_param.chg_en = 1;// 0: ��ֹ��磻1: �����磻2:����ģʽ

            vl_short_tmp = 0;    
            gl_poll_cmd_param.expect_current[0] = vl_short_tmp&0xFF;
            gl_poll_cmd_param.expect_current[1] = (vl_short_tmp>>8)&0xFF;
            
            vl_short_tmp = 0;
            gl_poll_cmd_param.expect_voltage[0] = vl_short_tmp&0xFF;
            gl_poll_cmd_param.expect_voltage[1] = (vl_short_tmp>>8)&0xFF;            
        }
        else if((0 == sl_charger_test_param.charger_cmd_enable_flag)&&(0 == sl_charger_test_param.charger_id_enable_flag))
        {
            gl_poll_cmd_param.chg_en = 0;// 0: ��ֹ��磻1: �����磻2:����ģʽ
//            main_charger_stop_by_id();

            vl_short_tmp = 0;    
            gl_poll_cmd_param.expect_current[0] = vl_short_tmp&0xFF;
            gl_poll_cmd_param.expect_current[1] = (vl_short_tmp>>8)&0xFF;
            
            vl_short_tmp = 0;
            gl_poll_cmd_param.expect_voltage[0] = vl_short_tmp&0xFF;
            gl_poll_cmd_param.expect_voltage[1] = (vl_short_tmp>>8)&0xFF;            
        }
        return TRUE;
    }

    return FALSE;
}







////�������ݣ�����ʱ���Զ����ͷ��CRC��β
//void master_rs485_snd_data(unsigned char* data, unsigned int len)
//{
//    unsigned char *pl_data;
//    unsigned short vl_index;
//    unsigned short vl_crc;
//    
//    pl_data = &pl_rs485_tx[ENUM_COMM_INDEX_MASTER]->rs485_tx_buf[0];
//    
//    memcpy(pl_data, data, len);
//    vl_index = len;

//    vl_crc = slave_rs485_calc_CRC(pl_data, vl_index);
//    //crc
//    pl_data[vl_index++] = vl_crc&0xFF;
//    pl_data[vl_index++] = (vl_crc>>8)&0xFF;
//   
//    pl_rs485_tx[ENUM_COMM_INDEX_MASTER]->rs485_tx_indx = vl_index;
//    slave_rs485_run_state[ENUM_COMM_INDEX_MASTER]=MASTER_RS485_RUN_TX;

//    //
//    sl_master_state.cmd_processing_flag = 1;
//    sl_master_state.cmd_timeout_cnt = 3000;
//    sl_master_state.poll_charger_cmd_cnt = 10000;
//}


void master_rs485_snd_cmd_get_dev_version(unsigned char* data)
{
    unsigned char *pl_data;
    unsigned short vl_index;
    unsigned short vl_crc;
    
    pl_data = &pl_rs485_tx[ENUM_COMM_INDEX_MASTER]->rs485_tx_buf[0];
    
    vl_index = 0;
    pl_data[0] = 0x01;
    pl_data[1] = ENUM_M_CMD_INFO;
    pl_data[2] = 2;//���ݳ���
    pl_data[3] = 4;//target, Charger
    pl_data[4] = 0;//port
    vl_index = 5;

    vl_crc = slave_rs485_calc_CRC(pl_data, vl_index);
    //crc
    pl_data[vl_index++] = vl_crc&0xFF;
    pl_data[vl_index++] = (vl_crc>>8)&0xFF;
   
    pl_rs485_tx[ENUM_COMM_INDEX_MASTER]->rs485_tx_indx = vl_index;
    slave_rs485_run_state[ENUM_COMM_INDEX_MASTER]=MASTER_RS485_RUN_TX;
}




static bool master_rs485_cmd_get_dev_version_cnf(st_hub_cmd *pl_hub_cmd)
{
    unsigned char *pl_data;
    if(pl_hub_cmd->stl_dataL == 0x07)
    {
        //��������Ǿɰ汾���
        sl_master_chg_new_protocol = 0; //�����Ǿɰ汾�ĳ�����������֧��Զ������
        
        //485ͨѶ���ϱ�־���
        sl_master_state.chg_485_cmd_poll_faild_cnt = 0;
        
        master_rs485_rcv_ack_process();
        return FALSE;
    }
    //�����ص�����.
    pl_data = pl_hub_cmd->stl_data;
    //���صĴ�����
    if(ENUM_COMM_ERROR_CODE_OK == pl_data[2])
    {
        memcpy(&gl_charger_dev_version.version_desc, (pl_data+3), sizeof(st_version_desc));
        gl_charger_dev_version.is_valid = 1;
    }
    else
    {
        //gl_charger_dev_version.is_valid = 0;
        memset((char*)&gl_charger_dev_version, 0, sizeof(st_charger_dev_version));
    }

    if(pl_hub_cmd->stl_dataL >= 0x1A)
    {
        memcpy((char*)gl_charger_dev_version.charger_sn, (char*)&pl_hub_cmd->stl_data[12], 14);
    }
    else
    {
        memset((char*)gl_charger_dev_version.charger_sn, 0, 14);
    }
    
    //485ͨѶ���ϱ�־���
    sl_master_state.chg_485_cmd_poll_faild_cnt = 0;

    master_rs485_rcv_ack_process();
    return TRUE;
}





//������������bit0 �Ͷ�·����bit5
//�������������10�ξ�����������������
//�����ſ��Ը�λ
//Battery_set_event_to_stop_chg_flag
typedef struct{
unsigned char is_charger_ocp_flag; // �Ƿ����
unsigned char charger_ocp_cnt; //�����������
unsigned char is_charger_scp_flag; //�Ƿ��·
unsigned char charger_scp_cnt; //�����·����
unsigned char is_charger_ovp_flag; //�Ƿ��ѹ����
unsigned char charger_ovp_cnt; //�����ѹ��������
unsigned char is_charger_otp_flag; //�Ƿ���±���
unsigned char charger_otp_cnt; //������±�������

unsigned int charger_ocp_tick_cnt_old; //����ʱ�����
unsigned int charger_scp_tick_cnt_old; //��·ʱ�����
unsigned int charger_ovp_tick_cnt_old; //��ѹʱ�����
unsigned int charger_otp_tick_cnt_old; //����ʱ�����
}st_charger_error_cycle_lock;
st_charger_error_cycle_lock gl_charger_error_cycle_lock;
#define CHARGER_ERROR_CYCLE_LOCK_CNT_MAX 10

void master_rs485_charger_error_cycle_lock_init(void)
{
    memset((char*)&gl_charger_error_cycle_lock, 0, sizeof(st_charger_error_cycle_lock));
}

void master_rs485_charger_error_cycle_lock_process(unsigned char ocp_flag, 
                                                        unsigned char scp_flag, 
                                                        unsigned char ovp_flag, 
                                                        unsigned char otp_flag)
{
    if((0 == gl_charger_error_cycle_lock.is_charger_ocp_flag)
        &&(ocp_flag))
    {
        gl_charger_error_cycle_lock.is_charger_ocp_flag = 1;
        if(gl_charger_error_cycle_lock.charger_ocp_cnt < CHARGER_ERROR_CYCLE_LOCK_CNT_MAX)
        {
            gl_charger_error_cycle_lock.charger_ocp_cnt++;
            if(gl_charger_error_cycle_lock.charger_ocp_cnt == 1)
            {
                gl_charger_error_cycle_lock.charger_ocp_tick_cnt_old = gl_delay_cnt_test;
            }

            if((gl_delay_cnt_test - gl_charger_error_cycle_lock.charger_ocp_tick_cnt_old) > (1000*60*20))
            {
                gl_charger_error_cycle_lock.charger_ocp_tick_cnt_old = gl_delay_cnt_test;
                gl_charger_error_cycle_lock.charger_ocp_cnt = 0;
            }
        }
        else
        {
            #ifdef EVENT_TO_STOP_CHARGING_ENABLE
            Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_CHARGER_OCP_LOCK);
            #endif
        }
    }
    if(0 == ocp_flag)
    {
        gl_charger_error_cycle_lock.is_charger_ocp_flag = 0;
    }

    if((0 == gl_charger_error_cycle_lock.is_charger_scp_flag)
        &&(scp_flag))
    {
        gl_charger_error_cycle_lock.is_charger_scp_flag = 1;
        if(gl_charger_error_cycle_lock.charger_scp_cnt < CHARGER_ERROR_CYCLE_LOCK_CNT_MAX)
        {
            gl_charger_error_cycle_lock.charger_scp_cnt++;
            if(gl_charger_error_cycle_lock.charger_scp_cnt == 1)
            {
                gl_charger_error_cycle_lock.charger_scp_tick_cnt_old = gl_delay_cnt_test;
            }

            if((gl_delay_cnt_test - gl_charger_error_cycle_lock.charger_scp_tick_cnt_old) > (1000*60*20))
            {
                gl_charger_error_cycle_lock.charger_scp_tick_cnt_old = gl_delay_cnt_test;
                gl_charger_error_cycle_lock.charger_scp_cnt = 0;
            }
        }
        else
        {
            #ifdef EVENT_TO_STOP_CHARGING_ENABLE
            Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_CHARGER_SCP_LOCK);
            #endif
        }
    }
    if(0 == scp_flag)
    {
        gl_charger_error_cycle_lock.is_charger_scp_flag = 0;
    }

    if((0 == gl_charger_error_cycle_lock.is_charger_ovp_flag)
        &&(ovp_flag))
    {
        gl_charger_error_cycle_lock.is_charger_ovp_flag = 1;
        if(gl_charger_error_cycle_lock.charger_ovp_cnt < CHARGER_ERROR_CYCLE_LOCK_CNT_MAX)
        {
            gl_charger_error_cycle_lock.charger_ovp_cnt++;
            if(gl_charger_error_cycle_lock.charger_ovp_cnt == 1)
            {
                gl_charger_error_cycle_lock.charger_ovp_tick_cnt_old = gl_delay_cnt_test;
            }

            if((gl_delay_cnt_test - gl_charger_error_cycle_lock.charger_ovp_tick_cnt_old) > (1000*60*20))
            {
                gl_charger_error_cycle_lock.charger_ovp_tick_cnt_old = gl_delay_cnt_test;
                gl_charger_error_cycle_lock.charger_ovp_cnt = 0;
            }
        }
        else
        {
            #ifdef EVENT_TO_STOP_CHARGING_ENABLE
            Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_CHARGER_OVP_LOCK);
            #endif
        }
    }
    if(0 == ovp_flag)
    {
        gl_charger_error_cycle_lock.is_charger_ovp_flag = 0;
    }

    if((0 == gl_charger_error_cycle_lock.is_charger_otp_flag)
        &&(otp_flag))
    {
        gl_charger_error_cycle_lock.is_charger_otp_flag = 1;
        if(gl_charger_error_cycle_lock.charger_otp_cnt < CHARGER_ERROR_CYCLE_LOCK_CNT_MAX)
        {
            gl_charger_error_cycle_lock.charger_otp_cnt++;
            if(gl_charger_error_cycle_lock.charger_otp_cnt == 1)
            {
                gl_charger_error_cycle_lock.charger_otp_tick_cnt_old = gl_delay_cnt_test;
            }

            if((gl_delay_cnt_test - gl_charger_error_cycle_lock.charger_otp_tick_cnt_old) > (1000*60*20))
            {
                gl_charger_error_cycle_lock.charger_otp_tick_cnt_old = gl_delay_cnt_test;
                gl_charger_error_cycle_lock.charger_otp_cnt = 0;
            }
        }
        else
        {
            #ifdef EVENT_TO_STOP_CHARGING_ENABLE
            Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_CHARGER_OTP_LOCK);
            #endif
        }
    }
    if(0 == otp_flag)
    {
        gl_charger_error_cycle_lock.is_charger_otp_flag = 0;
    }
}


/*
��������������ڲ��ĳ����·��ֵ��
����ֵ����ĳ����ֵKʱ���������ϱ���־��

ֻ�ڳ�����жϣ�ֹͣ���ʱ��λ������
*/

typedef struct{
unsigned char over_key_cnt;
unsigned char resistance_value; // ��λ��200m��
unsigned char key_value; //��λ��0.1
unsigned char over_key_flag:1;
unsigned char reseved:7;
}st_chg_circuit_resistance_param;

static st_chg_circuit_resistance_param gl_chg_circuit_resistance_param;

//��ʼ��
void master_rs485_charger_resistance_check_init(void)
{
    memset(&gl_chg_circuit_resistance_param, 0, sizeof(st_chg_circuit_resistance_param));
    gl_chg_circuit_resistance_param.key_value = 3; // 300��ŷķ
}

void master_rs485_charger_resistance_check_process(void)
{
    //�������
    //���������
    //���ڳ��״̬
    //��صĵ����ͳ�����ĵ�����С��600mA
    //�ǴΣ�����20�μ�����ĵ���ֵ����0.3ŷķ�����ñ�־��ֻҪ��һ�ε���0.3ŷķ�������־��
    //��ص���Ϊ����״̬
    unsigned short vl_charger_current;
    unsigned short vl_charger_voltage;
    unsigned short vl_bat_current;
    unsigned short vl_bat_voltage;
    unsigned short vl_resistance;

//    if(0 == is_battery_voltage_OK(0))
//    {
//        gl_chg_circuit_resistance_param.over_key_cnt = 0;
//        gl_chg_circuit_resistance_param.over_key_flag = 0;
//        gl_chg_circuit_resistance_param.resistance_value = 0;
//        return;
//    }

    vl_charger_current = gl_poll_cmd_cnf_param.chg_current[0];
    vl_charger_current += (gl_poll_cmd_cnf_param.chg_current[1]<<8);
    vl_charger_voltage = gl_poll_cmd_cnf_param.chg_voltage[0];
    vl_charger_voltage += (gl_poll_cmd_cnf_param.chg_voltage[1]<<8);
    
//    vl_bat_current = Battery_get_TCurrent(0);
    if(vl_bat_current > 30000)
    {
        vl_bat_current -= 30000;
        vl_bat_current /= 10;
    }
    else
    {
        return;
    }
    
    vl_bat_voltage = Battery_get_voltage(0);
    vl_bat_voltage /= 10;

    //����600mA���
    if(((vl_charger_current <= vl_bat_current) && ((vl_bat_current - vl_charger_current) >= 6))
        ||((vl_charger_current > vl_bat_current) && ((vl_charger_current - vl_bat_current) >= 6)))
    {
        return;
    }

    if(vl_bat_current < 4)
    {
        gl_chg_circuit_resistance_param.over_key_cnt = 0;
        gl_chg_circuit_resistance_param.over_key_flag = 0;
        gl_chg_circuit_resistance_param.resistance_value = 0;
        return;
    }
        
    if((vl_charger_voltage >= vl_bat_voltage)
        &&(0 != vl_bat_current))
    {
        vl_resistance = ((vl_charger_voltage - vl_bat_voltage)*10)/(vl_bat_current);
        gl_chg_circuit_resistance_param.resistance_value = vl_resistance/2;
        if(gl_chg_circuit_resistance_param.resistance_value > 7)
        {
            gl_chg_circuit_resistance_param.resistance_value = 7;
        }
        
        if(vl_resistance >= gl_chg_circuit_resistance_param.key_value)
        {
            gl_chg_circuit_resistance_param.over_key_cnt++;
        }
        else
        {
            gl_chg_circuit_resistance_param.over_key_cnt = 0;
            gl_chg_circuit_resistance_param.over_key_flag = 0;
        }

        if(gl_chg_circuit_resistance_param.over_key_cnt >= 20)
        {
            gl_chg_circuit_resistance_param.over_key_flag = 1;
        }
    }
    
}

bool master_rs485_charger_resistance_check_get_flag(void)
{
    return gl_chg_circuit_resistance_param.over_key_flag?TRUE:FALSE;
}

unsigned char master_rs485_charger_resistance_check_get_value(void)
{
    return gl_chg_circuit_resistance_param.resistance_value;
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





////////////////////////////////////////////////////////////////////////////////////////////////////



enum_power_mode power_mode_get_mode(void)
{
    return gl_power_mode_param.mode;
}

void power_mode_process(void)
{
    switch(gl_power_mode_param.mode)
    {
        case ENUM_POWER_MODE_IDLE:
            
            break;
        case ENUM_POWER_MODE_CHARGE:
            
            break;
        case ENUM_POWER_MODE_HEATER:
            
            break;
        case ENUM_POWER_MODE_LOADER:
            
            break;
        default:
            
            break;
    }
    
}



void master_rs485_init(void)
{
    rt_pin_mode(RS485_DIR_PIN, PIN_MODE_OUTPUT);
	rt_pin_write(RS485_DIR_PIN, PIN_LOW);
	    
    memset(&gl_poll_cmd_cnf_param, 0, sizeof(st_poll_cmd_cnf_param));
    cmd_list_init();
    sl_master_state.ID_close_delay_cnt = 0;
    sl_master_state.chg_485_cmd_poll_faild_cnt = 0;
    sl_master_state.normal_cmd_delay_when_chger_updating_cnt = 0;
    memset(&gl_charger_dev_version, 0, sizeof(st_charger_dev_version));
    master_rs485_set_chg_voltage_max_value_default();

    master_rs485_charger_error_cycle_lock_init();

    master_rs485_charger_resistance_check_init();
//	sl_master_state.poll_charger_cmd_cnt=3000;
}

void thread_rs485_entry(void* parameter)
{
    rt_tick_t vl_delay_tick_cnt;
    master_rs485_init();    
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

    app_fifo_init(&gl_rs485_param.usart_rx_fifo, usart_rx_fifo_buf, CHARGER_USART_RX_BUF_MAX);
    app_fifo_init(&gl_rs485_param.usart_tx_fifo, usart_tx_fifo_buf, CHARGER_USART_RX_BUF_MAX);
    
    rt_sem_init(&gl_rs485_param.usart_rx_sem, "RS485_SEM", 0, RT_IPC_FLAG_FIFO);
      
    gl_rs485_param.usart_dev = rt_device_find(RS485_USART_DEV_NAME);
    rt_device_open(gl_rs485_param.usart_dev, RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_TX);
    rt_device_control(gl_rs485_param.usart_dev, RT_DEVICE_CTRL_CONFIG, &gl_rs485_param.usart_config_param);
    rt_device_set_tx_complete(gl_rs485_param.usart_dev, master_rs485_charger_tx_done_cb);
    rt_device_set_rx_indicate(gl_rs485_param.usart_dev, master_rs485_charger_input_cb);   

    master_rs485_communication(MASTER_RS485_RESET);
	
	#ifdef MOTOR_CONTROL_ENABLE
    motor_control_init();
    #endif 

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

