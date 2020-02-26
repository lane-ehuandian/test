

#define BATTERY_PROCESS_C
#include <string.h>
#include "nfc_cmd_process.h"
#include "modbus_master.h"
#include "battery_process.h"
#include "ntag_app.h"
#include "xmodem.h"
//#include "mis.h"
//#include "adc_pms.h"
#include "elock.h"
#include "comm_board_protocol.h"
#include "IO_config.h"



//#define UNMASK_CODE

extern unsigned char vl_BMS_index; //��0��ʼ
extern unsigned int gl_delay_cnt_test;

static void Battery_chg_hw_ctrl_process(void);
static unsigned short Battery_group_get_highest_voltage(unsigned char group_index);
static unsigned char Battery_group_get_group_index_by_bms_index(unsigned char bms_index);
static bool Battery_is_charge_enable_by_BatState(unsigned char bms_index);
static void Battery_bat_chg_current_false_process(unsigned char bms_index);

#ifdef EVENT_TO_STOP_CHARGING_ENABLE
#define EVENT_TO_STOP_CHG_FLAG_DELAY_MAX 8000
static unsigned int sl_src_of_stop_flag_high_temp_delay_cnt = 0;
static unsigned int sl_src_of_stop_flag_pre_current_delay_cnt = 0;
static unsigned int sl_src_of_stop_flag_low_voltage_delay_cnt = 0;
static unsigned int sl_src_of_stop_flag_soc_error_delay_cnt = 0;
static unsigned int sl_src_of_stop_flag_uvp_error_delay_cnt = 0;
static unsigned int sl_src_of_stop_flag_uvp_ovp_error_delay_cnt = 0;
static unsigned int sl_src_of_stop_flag_utp_otp_error_delay_cnt = 0;
static unsigned int sl_src_of_stop_flag_cell_voltage_low_delay_cnt = 0;
static unsigned int sl_src_of_stop_flag_hall_off_delay_cnt = 0;
static unsigned int sl_src_of_stop_flag_max_voltage_exceeded_delay_cnt = 0;
static unsigned int sl_src_of_stop_flag_cell_v_diff_value_exceeded_delay_cnt = 0;
#endif

#ifdef TEST_DETECT_BAT_ERROR_EN  
#define BAT_CONNECTED_FAILD_ERR_CNT_RELOAD_MAX 10
#define BAT_CONNECTED_FAILD_ERROR_CNT_MAX (1000*60*5)
#endif


static unsigned int gl_Battery_delay_process_cnt = 4000;
#define BATTERY_VOLTAGE_SHAKE_CNT_MAX 2000
static unsigned int gl_Battery_voltage_shake_cnt;
void Battery_timer_count_cb(void)
{
    if(gl_Battery_delay_process_cnt)
        --gl_Battery_delay_process_cnt;
    if(sl_bat_poll_param.poll_inerval_timer_cnt)
        --sl_bat_poll_param.poll_inerval_timer_cnt;
    if(sl_bat_poll_param.poll_allways_en_delay_cnt)
        --sl_bat_poll_param.poll_allways_en_delay_cnt;
    if(sl_bat_poll_param.poll_interval_for_bms_update_cnt)
        --sl_bat_poll_param.poll_interval_for_bms_update_cnt;
#ifdef TEST_BAT_V_VIA_CHARGER_EN
    if(gl_check_bat_v.bat_v_charger_comm_timeout_cnt)
    {
        gl_check_bat_v.bat_v_charger_comm_timeout_cnt--;
    }
    if(gl_check_bat_v.bat_v_charger_check_totol_time_cnt)
    {
        gl_check_bat_v.bat_v_charger_check_totol_time_cnt--;
    }
#endif

#ifdef TEST_DETECT_BAT_ERROR_EN    
    if(gl_check_bat_err.comm_counter_cnt)
    {
        gl_check_bat_err.comm_counter_cnt--;
        if(gl_check_bat_err.comm_counter_cnt == 0)
        {
            master_rs485_check_charger_by_cmd();
        }
    }
    if(gl_check_bat_err.door_close_delay_cnt)
    {
        gl_check_bat_err.door_close_delay_cnt--;
        if(gl_check_bat_err.door_close_delay_cnt == 0)
        {
            Bttery_check_bat_init();
        }
    }
    if(gl_check_bat_err.bat_process_cnt)
        gl_check_bat_err.bat_process_cnt--;

    if(gl_check_bat_err.checking_step_timeout_cnt)
    {
        gl_check_bat_err.checking_step_timeout_cnt--;
        if(gl_check_bat_err.checking_step_timeout_cnt == 0)
        {
            gl_check_bat_err.is_bat_error |= 0x40;
            gl_check_bat_err.is_need_bat_checking = 2;
            gl_check_bat_err.checking_step_timeout_cnt = 0;
            Battery_save_bat_reg_state_when_detect_bat_error(0);
            //30���Ӻ�����
            gl_check_bat_err.bat_step_timeout_err_cnt = 1800000; //30����
        }
    }

    if(gl_check_bat_err.bat_step_timeout_err_cnt)
    {
        gl_check_bat_err.bat_step_timeout_err_cnt--;
        if(0 == gl_check_bat_err.bat_step_timeout_err_cnt)
        {
            if(gl_check_bat_err.is_bat_error & 0x40)
            {
                //����PMS
                main_Reset_system();
            }
        }
    }

    if(gl_check_bat_err.bat_connected_faild_error_cnt)
    {
        gl_check_bat_err.bat_connected_faild_error_cnt--;
        if(gl_check_bat_err.bat_connected_faild_error_cnt == 0)
        {
            if(gl_check_bat_err.bat_connected_faild_error_cnt_reload_max == 0)
            {
                gl_check_bat_err.bat_connected_faild_error_cnt_reload_max = BAT_CONNECTED_FAILD_ERR_CNT_RELOAD_MAX;
                //gl_check_bat_err.bat_connected_faild_error_cnt = (1000*60*5);
                //�ٴ������������
                Bttery_check_bat_init();
            }
            else
            {
                gl_check_bat_err.bat_connected_faild_error_cnt_reload_max--;
                if(gl_check_bat_err.bat_connected_faild_error_cnt_reload_max)
                {
                    //gl_check_bat_err.bat_connected_faild_error_cnt = (1000*60*5);
                    //�ٴ������������
                    Bttery_check_bat_init();
                }
            }
        }
    }
    
#endif    
    #ifdef EVENT_TO_STOP_CHARGING_ENABLE
    if(gl_event_to_stop_chg_param.hall_miss_status_clear_delay_cnt)
    {
        gl_event_to_stop_chg_param.hall_miss_status_clear_delay_cnt--;
    }
    #endif

    #ifdef EVENT_TO_STOP_CHARGING_ENABLE

    if(sl_src_of_stop_flag_high_temp_delay_cnt)
    {
        sl_src_of_stop_flag_high_temp_delay_cnt--;
        if(sl_src_of_stop_flag_high_temp_delay_cnt == 0)
        {
            Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_HIGH_TEMP);
        }
    }

    if(sl_src_of_stop_flag_pre_current_delay_cnt)
    {
        sl_src_of_stop_flag_pre_current_delay_cnt--;
        if(sl_src_of_stop_flag_pre_current_delay_cnt == 0)
        {
            Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_PRE_CURRENT);
        }
    }

    if(sl_src_of_stop_flag_low_voltage_delay_cnt)
    {
        sl_src_of_stop_flag_low_voltage_delay_cnt--;
        if(sl_src_of_stop_flag_low_voltage_delay_cnt == 0)
        {
            Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_LOW_VOLTAGE);
        }
    }

    if(sl_src_of_stop_flag_soc_error_delay_cnt)
    {
        sl_src_of_stop_flag_soc_error_delay_cnt--;
        if(sl_src_of_stop_flag_soc_error_delay_cnt == 0)
        {
            Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_SOC_ERROR);
        }
    }

    
    if(sl_src_of_stop_flag_uvp_error_delay_cnt)
    {
        sl_src_of_stop_flag_uvp_error_delay_cnt--;
        if(sl_src_of_stop_flag_uvp_error_delay_cnt == 0)
        {
            Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_UVP_ERROR);
        }
    }

    if(sl_src_of_stop_flag_uvp_ovp_error_delay_cnt)
    {
        sl_src_of_stop_flag_uvp_ovp_error_delay_cnt--;
        if(sl_src_of_stop_flag_uvp_ovp_error_delay_cnt == 0)
        {
            Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_UVP_OVP_ERROR);
        }
    }

    if(sl_src_of_stop_flag_utp_otp_error_delay_cnt)
    {
        sl_src_of_stop_flag_utp_otp_error_delay_cnt--;
        if(sl_src_of_stop_flag_utp_otp_error_delay_cnt == 0)
        {
            Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_UTP_OTP_ERROR);
        }
    }

    if(sl_src_of_stop_flag_cell_voltage_low_delay_cnt)
    {
        sl_src_of_stop_flag_cell_voltage_low_delay_cnt--;
        if(sl_src_of_stop_flag_cell_voltage_low_delay_cnt == 0)
        {
            Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_CELL_VOLTAGE_LOW);
        }
    }

    if(sl_src_of_stop_flag_hall_off_delay_cnt)
    {
        sl_src_of_stop_flag_hall_off_delay_cnt--;
        if(sl_src_of_stop_flag_hall_off_delay_cnt == 0)
        {
            gl_event_to_stop_chg_param.hall_miss_status_clear_delay_cnt = 600000;
            Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_HALL_OFF);
        }
    }

    if(sl_src_of_stop_flag_max_voltage_exceeded_delay_cnt)
    {
        sl_src_of_stop_flag_max_voltage_exceeded_delay_cnt--;
        if(sl_src_of_stop_flag_max_voltage_exceeded_delay_cnt == 0)
        {
            Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_MAX_VOLTAGE_EXCEEDED);
        }
    }

    if(sl_src_of_stop_flag_cell_v_diff_value_exceeded_delay_cnt)
    {
        sl_src_of_stop_flag_cell_v_diff_value_exceeded_delay_cnt--;
        if(sl_src_of_stop_flag_cell_v_diff_value_exceeded_delay_cnt == 0)
        {
            Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_CELL_V_DIFF_VALUE_EXCEEDED);
        }
    }
    
    #endif

    if(sl_bat_error_check_delay_after_bat_update_cnt)
    {
        sl_bat_error_check_delay_after_bat_update_cnt--;
    }

    #ifdef BAT_TEMP_CHARGER_ENABLE
    if(gl_bat_low_temp_param.delay_counter_1)
    {
        gl_bat_low_temp_param.delay_counter_1--;
        if(0 == gl_bat_low_temp_param.delay_counter_1)
        {
            gl_bat_low_temp_param.is_bat_chg_enable = 1;
            gl_bat_low_temp_param.delay_counter_charge = gl_bat_low_temp_param.low_current_delay_count;
            gl_bat_low_temp_param.delay_counter_2 = 0;
        }
    }
    if(gl_bat_low_temp_param.delay_counter_2)
    {
        gl_bat_low_temp_param.delay_counter_2--;
        if(0 == gl_bat_low_temp_param.delay_counter_2)
        {
            gl_bat_low_temp_param.is_bat_chg_enable = 1;
            gl_bat_low_temp_param.delay_counter_charge = gl_bat_low_temp_param.low_current_delay_count;
            gl_bat_low_temp_param.delay_counter_1 = 0;
        }
    }
    if(gl_bat_low_temp_param.delay_counter_charge)
    {
        gl_bat_low_temp_param.delay_counter_charge--;
    }
    #endif
    Battery_bat_fault_debounce_param_timer_cb();

    #ifdef PMS_AUTO_INCREASE_SETTING_VOLTAGE_ENABLE
    if(gl_charger_setting_V_offset_param.debount_time_cnt)
    {
        gl_charger_setting_V_offset_param.debount_time_cnt--;
        if((!gl_charger_setting_V_offset_param.debount_time_cnt)&&
            (gl_charger_setting_V_offset_param.voltage_offset_value < 4))
        {
            gl_charger_setting_V_offset_param.voltage_offset_value += 2;
        }
    }
    #endif

    if(gl_bat_history_record_param.NFC_CMD_interval_cnt)
        gl_bat_history_record_param.NFC_CMD_interval_cnt--;

    /**/
    if((
            ((BMS_READONLY_DATA1_FLAG_BIT|BMS_READONLY_DATA2_FLAG_BIT)&
            Battery_get_reg_valid_flag(0)))||
            (gl_elock_param[0].elock_bms_flag&(1<<0))||
            (gl_bms_app_bl_status[0].is_bl_mode))//����bootloaderģʽ
    {
        gl_pms_param.is_bat_detected = 1;
    }
    else
    {
        gl_pms_param.is_bat_detected = 0;
    }
    
}

/*��ȡĿǰpms���е�ģʽ: 
Code	0x01	0x02	0x03
State	Sleep	Charge	Discharge
*/
unsigned char Battery_process_get_state(void)
{
    unsigned char vl_tmp;
    switch(gl_BP_info.bms_state)
    {
        case ENUM_DISCHARGE:
            vl_tmp = 0x03;
            break;
        case ENUM_CHARGE:
            vl_tmp = 0x02;
            break;
        case ENUM_SLEEP:
            vl_tmp = 0x01;
            break;
        default:
            //Ӧ�ò��ܴ�������ѡ��
            vl_tmp = 0;
            break;
    }
    return vl_tmp;
}

void Battery_Process_Init(void)
{
    gl_BP_info.bms_state = ENUM_CHARGE;
    sl_bat_poll_param.is_poll_allways_en_flag = 1;
    sl_bat_poll_param.poll_inerval_timer_cnt = 0;
    sl_bat_poll_param.poll_interval_MAX = BAT_POLL_INTERVAL_MAX;
    sl_bat_poll_param.poll_allways_en_delay_cnt = BAT_POLL_DELAY_MAX;
    sl_bat_poll_param.poll_interval_for_bms_update_cnt = 0;

    gl_pms_param.is_bat_detected = 0;
    gl_pms_param.OD_pms_cmd &= (~OD_PMS_CMD_MASK_CHARGE_EN);
    gl_pms_param.OD_swapping_soc = 80;
    gl_pms_param.OD_3010_max_subindex = 2;
    gl_pms_param.OD_3011_max_subindex = 4;
    gl_pms_param.OD_3013_max_subindex = 3;
    gl_pms_param.OD_3014_max_subindex = 6;
    gl_pms_param.OD_3015_max_subindex = 7;
    gl_pms_param.OD_3016_max_subindex = 3;
    gl_pms_param.OD_3017_max_subindex = 2;
    gl_pms_param.OD_3018_max_subindex = 5;

    /*���PMS�����ļ��Ƿ���Ч*/

    gl_pms_param.OD_pms_update_file_param[0].OD_update_file_max_subindex = 3;


    /*������������ļ��Ƿ���Ч*/

    gl_pms_param.OD_charger_update_file_param[0].OD_update_file_max_subindex = 3;
    gl_pms_param.OD_charger_update_file_param[1].OD_update_file_max_subindex = 3;
    gl_pms_param.OD_charger_update_file_param[2].OD_update_file_max_subindex = 3;


    /*����������ļ��Ƿ���Ч*/

    {
        unsigned char i;
        for(i = 0; i < 64; i++)
        {
            gl_pms_param.OD_bat_update_file_param[i].OD_update_file_max_subindex = 3;
        }
    }
        
}

void Battery_Process(void)
{
    //test
    //gl_BP_info.bms_state = ENUM_DISCHARGE;
    switch(gl_BP_info.bms_state)
    {
        case ENUM_DISCHARGE:
            Battery_discharge_process();
            break;
		#if 0
        case ENUM_CHARGE:
            Battery_charge_process();
            break;
        case ENUM_SLEEP:
            Battery_sleep_process();
            break;
		#endif
        default:
            Battery_discharge_process();
            break;
    }
#if 0 // ����������ŵ��״̬ ��2018-6-29
    if((0 == Battery_get_PMS_working_state()))
    {
        //������ŵ�
        if(0 == Battery_get_battery_sleep_state())
        {
            DEBUG_PRINTF("Force to sleep!\r\n");
            //Battery_set_battery_sleep();
            Battery_set_battery_sleep_by_delay(ENUM_BAT_SLEEP_SRC_FORCE_TO_SLP, 5000);
        }
    }
#endif    
}


unsigned char Battery_send_cmd(unsigned char bms_index)
{
    unsigned char i;
    i = bms_index;
    //for(i = 0; i < NFC_READER_COUNT_MAX; i++)
    {
        if(gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_BYPASS])
        {
            unsigned char *vl_data;
            unsigned int vl_data_len;
            slave_rs485_get_rx_data_from_buf(ENUM_COMM_INDEX_COMM, &vl_data, &vl_data_len);
            //����ͨѶ
            if(MM_set_BMS_index(i))
            {
                if((vl_data_len <= 7)|| // ��ʱ���ʾû��Ҫ͸��������
                    (MM_snd_bypass(MM_ADDR_VALUE, vl_data+5, vl_data_len-7)))
                {
                    gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_BYPASS] = 0;
                    return 1;
                }
            }
        }
        #ifdef BAT_PMS_AUTHORITY_FUNCTION_EN
        if(gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_AUTHORITY])
        {
            //����ͨѶ
            if(MM_set_BMS_index(i))
            {
                if(MM_snd_authority_process())
                {
                    gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_AUTHORITY] = 0;
                    return 1;
                }
            }
        }
        #endif
        if(gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_W_RESET])
        {
            unsigned short vl_k;
            unsigned short vl_cnt;
            st_write_reg_10 vl_write_reg_10;
            
            vl_cnt = 1; //CTRL_REG_TOTAL_SIZE
            vl_write_reg_10.reg_addr_begin[0]=(ENUM_REG_ADDR_RESET>>8)&0xFF;
            vl_write_reg_10.reg_addr_begin[1]=(ENUM_REG_ADDR_RESET)&0xFF;
            vl_write_reg_10.reg_cnt[0] = (vl_cnt>>8)&0xFF;
            vl_write_reg_10.reg_cnt[1] = (vl_cnt)&0xFF;
            vl_write_reg_10.byte_cnt = vl_cnt*2;

            vl_k = MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_RESET);
            
            //��������ֵ
            gl_bms_info_p[i]->reg_unit[vl_k].bits16_H = 0x7A;
            gl_bms_info_p[i]->reg_unit[vl_k].bits16_L= 0xB9;
            
            vl_write_reg_10.byte_data = (unsigned char*)&gl_bms_info_p[i]->reg_unit[vl_k];
            //����ͨѶ
            if(MM_set_BMS_index(i))
            {
                if(MM_snd_multi_write_cmd(MM_ADDR_VALUE, &vl_write_reg_10))
                {
                    gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_W_RESET] = 0;
                    return 1;
                }
            }
        }
    }
    
    //for(i = 0; i < NFC_READER_COUNT_MAX; i++)
    {
        if(gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_W_CTRL])
        {
            unsigned short vl_k;
            unsigned short vl_cnt;
            st_write_reg_10 vl_write_reg_10;

            if(0 == (BMS_CTRL_FLAG_BIT&Battery_get_reg_valid_flag(i)))
            {
                //�洢�Ŀ��ƼĴ���������Ч
                gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_W_CTRL] = 0;
                    return 1;
            }

            
            vl_cnt = 1; //CTRL_REG_TOTAL_SIZE
            vl_write_reg_10.reg_addr_begin[0]=(CTRL_REG_BIGIN_ADDR>>8)&0xFF;
            vl_write_reg_10.reg_addr_begin[1]=(CTRL_REG_BIGIN_ADDR)&0xFF;
            vl_write_reg_10.reg_cnt[0] = (vl_cnt>>8)&0xFF;
            vl_write_reg_10.reg_cnt[1] = (vl_cnt)&0xFF;
            vl_write_reg_10.byte_cnt = vl_cnt*2;

            vl_k = MM_get_reg_addr_position_in_table(CTRL_REG_BIGIN_ADDR);
            vl_write_reg_10.byte_data = (unsigned char*)&gl_bms_info_p[i]->reg_unit[vl_k];
            //����ͨѶ
            if(MM_set_BMS_index(i))
            {
                if(MM_snd_multi_write_cmd(MM_ADDR_VALUE, &vl_write_reg_10))
                {
                    gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_W_CTRL] = 0;
                    return 1;
                }
            }
        }
    }

    //for(i = 0; i < NFC_READER_COUNT_MAX; i++)
    {
        if(gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_R_VERSION])
        {
            st_read_reg_03 vl_read_reg_03;
            vl_read_reg_03.reg_addr_begin[0]=(DEV_INFO_REG_BIGIN_ADDR>>8)&0xFF;
            vl_read_reg_03.reg_addr_begin[1]=(DEV_INFO_REG_BIGIN_ADDR)&0xFF;
            vl_read_reg_03.reg_cnt[0] = (DEV_INFO_REG_TOTAL_SIZE>>8)&0xFF;
            vl_read_reg_03.reg_cnt[1] = (DEV_INFO_REG_TOTAL_SIZE)&0xFF;

            //����1ͨѶ
            if(MM_set_BMS_index(i))
            {
                if(MM_snd_multi_read_cmd(MM_ADDR_VALUE, &vl_read_reg_03))
                {
                    gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_R_VERSION] = 0;
                    return 1;
                }
            }
        }
        else if(gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_R_INFO1])
        {
            st_read_reg_03 vl_read_reg_03;
            vl_read_reg_03.reg_addr_begin[0]=(READ_ONLY_REG_BIGIN_ADDR>>8)&0xFF;
            vl_read_reg_03.reg_addr_begin[1]=(READ_ONLY_REG_BIGIN_ADDR)&0xFF;
            vl_read_reg_03.reg_cnt[0] = ((READ_ONLY_REG_TOTAL_SIZE/2)>>8)&0xFF;
            vl_read_reg_03.reg_cnt[1] = ((READ_ONLY_REG_TOTAL_SIZE/2))&0xFF;

            //����1ͨѶ
            if(MM_set_BMS_index(i))
            {
                if(MM_snd_multi_read_cmd(MM_ADDR_VALUE, &vl_read_reg_03))
                {
                    gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_R_INFO1] = 0;
                    return 1;
                }
            }
        }
        else if(gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_R_INFO2])
        {
            st_read_reg_03 vl_read_reg_03;
            vl_read_reg_03.reg_addr_begin[0]=((READ_ONLY_REG_BIGIN_ADDR+(READ_ONLY_REG_TOTAL_SIZE/2))>>8)&0xFF;
            vl_read_reg_03.reg_addr_begin[1]=((READ_ONLY_REG_BIGIN_ADDR+(READ_ONLY_REG_TOTAL_SIZE/2)))&0xFF;
            //�жϵ�ذ汾���Ƿ���ڵ���35�汾
            if(((BMS_DEV_INFO_FLAG_BIT)&Battery_get_reg_valid_flag(bms_index))&&
                (35 <= Battery_get_reg_value(bms_index, ENUM_REG_ADDR_FWREV)))
            {
                //֧�ּĴ�����ַ322
                vl_read_reg_03.reg_cnt[0] = ((READ_ONLY_REG_TOTAL_SIZE-(READ_ONLY_REG_TOTAL_SIZE/2))>>8)&0xFF;
                vl_read_reg_03.reg_cnt[1] = ((READ_ONLY_REG_TOTAL_SIZE-(READ_ONLY_REG_TOTAL_SIZE/2)))&0xFF;
            }
            else
            {
                vl_read_reg_03.reg_cnt[0] = ((READ_ONLY_REG_TOTAL_SIZE-(READ_ONLY_REG_TOTAL_SIZE/2) - 7)>>8)&0xFF;
                vl_read_reg_03.reg_cnt[1] = ((READ_ONLY_REG_TOTAL_SIZE-(READ_ONLY_REG_TOTAL_SIZE/2) - 7))&0xFF;
            }

            //����1ͨѶ
            if(MM_set_BMS_index(i))
            {
                if(MM_snd_multi_read_cmd(MM_ADDR_VALUE, &vl_read_reg_03))
                {
                    gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_R_INFO2] = 0;
                    return 1;
                }
            }
        }
        else if(gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_R_CTRL])
        {
            st_read_reg_03 vl_read_reg_03;
            vl_read_reg_03.reg_addr_begin[0]=(CTRL_REG_BIGIN_ADDR>>8)&0xFF;
            vl_read_reg_03.reg_addr_begin[1]=(CTRL_REG_BIGIN_ADDR)&0xFF;
            vl_read_reg_03.reg_cnt[0] = (CTRL_REG_TOTAL_SIZE>>8)&0xFF;
            vl_read_reg_03.reg_cnt[1] = (CTRL_REG_TOTAL_SIZE)&0xFF;

            //����1ͨѶ
            if(MM_set_BMS_index(i))
            {
                if(MM_snd_multi_read_cmd(MM_ADDR_VALUE, &vl_read_reg_03))
                {
                    gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_R_CTRL] = 0;
                    return 1;
                }
            }
        }
        else if(gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_R_PARAM1])
        {
            st_read_reg_03 vl_read_reg_03;
            vl_read_reg_03.reg_addr_begin[0]=(PARAM_REG_BIGIN_ADDR>>8)&0xFF;
            vl_read_reg_03.reg_addr_begin[1]=(PARAM_REG_BIGIN_ADDR)&0xFF;
            vl_read_reg_03.reg_cnt[0] = ((PARAM_REG_TOTAL_SIZE/2)>>8)&0xFF;
            vl_read_reg_03.reg_cnt[1] = ((PARAM_REG_TOTAL_SIZE/2))&0xFF;

            //����1ͨѶ
            if(MM_set_BMS_index(i))
            {
                if(MM_snd_multi_read_cmd(MM_ADDR_VALUE, &vl_read_reg_03))
                {
                    gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_R_PARAM1] = 0;
                    return 1;
                }
            }
        }
        else if(gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_R_PARAM2])
        {
            st_read_reg_03 vl_read_reg_03;
            vl_read_reg_03.reg_addr_begin[0]=((PARAM_REG_BIGIN_ADDR+(PARAM_REG_TOTAL_SIZE/2))>>8)&0xFF;
            vl_read_reg_03.reg_addr_begin[1]=((PARAM_REG_BIGIN_ADDR+(PARAM_REG_TOTAL_SIZE/2)))&0xFF;
            vl_read_reg_03.reg_cnt[0] = ((PARAM_REG_TOTAL_SIZE-(PARAM_REG_TOTAL_SIZE/2))>>8)&0xFF;
            vl_read_reg_03.reg_cnt[1] = ((PARAM_REG_TOTAL_SIZE-(PARAM_REG_TOTAL_SIZE/2)))&0xFF;

            //����1ͨѶ
            if(MM_set_BMS_index(i))
            {
                if(MM_snd_multi_read_cmd(MM_ADDR_VALUE, &vl_read_reg_03))
                {
                    gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_R_PARAM2] = 0;
                    return 1;
                }
            }
        }
        else if(gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_R_USERDATA])
        {
            st_read_reg_03 vl_read_reg_03;
            vl_read_reg_03.reg_addr_begin[0]=(USER_REG_BIGIN_ADDR>>8)&0xFF;
            vl_read_reg_03.reg_addr_begin[1]=(USER_REG_BIGIN_ADDR)&0xFF;
            vl_read_reg_03.reg_cnt[0] = (USER_REG_TOTAL_SIZE>>8)&0xFF;
            vl_read_reg_03.reg_cnt[1] = (USER_REG_TOTAL_SIZE)&0xFF;

            //����1ͨѶ
            if(MM_set_BMS_index(i))
            {
                if(MM_snd_multi_read_cmd(MM_ADDR_VALUE, &vl_read_reg_03))
                {
                    gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_R_USERDATA] = 0;
                    return 1;
                }
            }
        }
        else if(gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_R_BLV])
        {
            unsigned char vl_tmp;

            //����1ͨѶ
            if(MM_set_BMS_index(i))
            {
                vl_tmp = 'V';
                if(MM_snd_update_packet(i, MM_ADDR_VALUE, &vl_tmp, 1))
                {
                    gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_R_BLV] = 0;
                    return 1;
                }
            }
        }
        else if(gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_HISTORY_RECORD])
        {
            //����1ͨѶ
            if(MM_set_BMS_index(i))
            {
                if(0 == gl_bat_history_record_param.is_earliest_param_valid)
                {
                    //��ȡ�������������¼���
                    gl_bat_history_record_param.history_record_lastest_index = Battery_get_newest_record_index_number();
                    //��ȡ�������������¼���
                    gl_bat_history_record_param.history_record_earliest_index = Battery_get_oldest_record_index_number();
                
                    if(MM_snd_read_history_record_cmd(MM_ADDR_VALUE, 
                        gl_bat_history_record_param.history_record_earliest_index)
                        )
                    {
                        gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_HISTORY_RECORD] = 0;
                        //���ü��ʱ��3S
                        Batter_history_record_set_nfc_cmd_interval_cnt(1000);
                        return 1;
                    }
                }
                else if(0 == gl_bat_history_record_param.is_lastest_param_valid)
                {
                    //��ȡ�������������¼���
                    gl_bat_history_record_param.history_record_lastest_index = Battery_get_newest_record_index_number();
                    //��ȡ�������������¼���
                    gl_bat_history_record_param.history_record_earliest_index = Battery_get_oldest_record_index_number();
                    
                    if(MM_snd_read_history_record_cmd(MM_ADDR_VALUE, 
                        gl_bat_history_record_param.history_record_lastest_index)
                        )
                    {
                        gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_HISTORY_RECORD] = 0;
                        //���ü��ʱ��3S
                        Batter_history_record_set_nfc_cmd_interval_cnt(1000);
                        return 1;
                    }
                }
                else
                {
                    if(MM_snd_read_history_record_cmd(MM_ADDR_VALUE, 
                        Battery_history_record_get_real_index(gl_bat_history_record_param.history_record_begin_index + 1 + gl_bat_history_record_param.history_record_read_cnt))
                        )
                    {
                        gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_HISTORY_RECORD] = 0;
                        //���ü��ʱ��3S
                        Batter_history_record_set_nfc_cmd_interval_cnt(1000);
                        return 1;
                    }
                }
            }
        }
    }    

    return 0;
}


void Battery_cmd_read_all_reg(unsigned char bms_index)
{
    //������ѯһ�ε�ذ汾��Ϣ
    //if(vl_bat_info_flag[0]==0)
    {
        gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_VERSION] = 1;
    }
    
    //��ѯBMS ֻ����Ϣ���ӼĴ���256��308
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_INFO1] = 1;
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_INFO2] = 1;
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_CTRL] = 1;

    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_PARAM1] = 1;
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_PARAM2] = 1;

    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_USERDATA] = 1;

}


void Battery_cmd_buf_clear(unsigned char bms_index)
{
    memset(&gl_cmd_buf[bms_index][0], 0, ENUM_NFC_CMD_INDEX_MAX);
}

//���û�ȡ��ذ汾��Ϣ������
void Battery_cmd_buf_get_version(unsigned char bms_index)
{
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_VERSION] = 1;
}

//����NFC�������Ƿ����������Ѿ��������
bool Battery_is_one_cmd_buf_empty(unsigned char bms_index)
{
    unsigned char i;

    for(i = 0; i < (ENUM_NFC_CMD_INDEX_MAX); i++)
    {
        if(gl_cmd_buf[bms_index][i])
        {
            return FALSE;
        }
    }

    return TRUE;
}

static unsigned char sl_battery_need2sleep_flag = 0;// 1��ʾ�õ�ؽ�������״̬

//TEST, ��ӡ��Ч��ѹ����
static unsigned char sl_valid_A_V_print_flag = 0;

unsigned char gl_illegal_charging_in_flag = 0;

void Battery_discharge_process(void)
{
    unsigned short vl_voltage_diff;
    unsigned short vl_bms_0_V, vl_bms_1_V;
    int vl_bms_0_A, vl_bms_1_A;
    int vl_current_diff;
    unsigned short vl_bms_0_ctrl, vl_bms_1_ctrl;
    static unsigned char sl_supply_state = 0; // 1, ��ʾ��ѹ��С��0.5v��2,��ʾ��ѹ�����0.5v

    unsigned char vl_force_to_exe[2] = {0, 0};

    unsigned short vl_pos, vl_state_pos;

    //ÿ���ѯһ�ε�����ȷ���Ƿ���������
    static unsigned int sl_old_tick_cnt = 0;
    unsigned char vl_check_charger_current_in_flag_1_sec = 0; // 1���ӱ�־

    //���Դ�ӡ��ѹ��ȱ�־
    static unsigned char sl_both_voltage_equ_flag = 0;
    static unsigned char sl_both_voltage_diff_flag = 0;

    static unsigned char sl_bms0_vol_cmp_offset = 0; 
    static unsigned char sl_bms1_vol_cmp_offset = 0;


    #ifdef PMS_SELFOFF_TIME_COUNTER_ENABLE
    if(main_is_pms_selfoff_valid())
    {
        Battery_sleep_process();
        return ;
    }
    #endif

    #ifdef PMS_CMD_POWEROFF_TIME_COUNTER_ENABLE
    //��ӡDEBUG��Ϣ
    //Battery_poweroff_cmd_print_info();
    
    if(main_is_cmd_poweroff_valid_immediately())//����ִ��
    {
        Battery_sleep_process();
        return ;
    }  

    if((main_is_cmd_poweroff_valid())//ͣ��ִ��
        ||(main_cmd_poweroff_get_poweroff_flag_nonvolatilely())//�Ѿ�ͣ��ִ�й�һ��
        )
    {
        if((main_is_cmd_poweroff_valid()))
        {
            main_cmd_poweroff_set_poweroff_flag_nonvolatilely(1);
        }
        //����˯��ģʽ
        //Battery_set_battery_sleep_by_delay(ENUM_BAT_SLEEP_SRC_CMD_POWEROFF, 1000);
    }  
    #endif

    

//    if((gl_tick_cnt - sl_old_tick_cnt) >= 1000)
//    {
//        vl_check_charger_current_in_flag_1_sec = 1;
//        sl_old_tick_cnt = gl_tick_cnt;
//    }

//    if(sl_battery_need2sleep_flag)
//    {
//        
//        vl_pos = MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_CTRL);
//        vl_state_pos = MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_STATE);
//        if((1 == is_battery_voltage_OK(0))&&
//            (0 != (gl_bms_info_p[0]->reg_unit[vl_state_pos].bits16_L & (1<<3)))&& // �жϻ�������
//            ((0 != (gl_bms_info_p[0]->reg_unit[vl_state_pos].bits16_L & (1<<0)))|| // �ж��Ƿ�رճ��
//            //(0 == (gl_bms_info_p[0]->reg_unit[vl_pos].bits16_L&(1<<5))))&& // �ж��Ƿ�������ģʽ
//            //(0 == is_battery_error(0)))
//            (0 == (gl_bms_info_p[0]->reg_unit[vl_pos].bits16_L&(1<<5)))|| // �ж��Ƿ�������ģʽ
//            ((0 == (gl_bms_info_p[0]->reg_unit[vl_pos].bits16_L&(1<<6)))&& //��Ҫ�������˯��
//            (0 == Battery_get_PMS_working_state())&&
//            (main_get_have_been_ACC_ON_flag()))))
//        {
//            vl_force_to_exe[0] = 1;
//        }

//        if((1 == is_battery_voltage_OK(1))&&
//            (0 != (gl_bms_info_p[1]->reg_unit[vl_state_pos].bits16_L&(1<<3)))&& // �жϻ�������
//            ((0 != (gl_bms_info_p[1]->reg_unit[vl_state_pos].bits16_L&(1<<0)))|| // �ж��Ƿ�رճ��
//            //(0 == (gl_bms_info_p[1]->reg_unit[vl_pos].bits16_L&(1<<5))))&&
//            //(0 == is_battery_error(1)))
//            (0 == (gl_bms_info_p[1]->reg_unit[vl_pos].bits16_L&(1<<5)))||
//            ((0 == (gl_bms_info_p[1]->reg_unit[vl_pos].bits16_L&(1<<6)))&& //��Ҫ�������˯��
//            (0 == Battery_get_PMS_working_state())&&
//            (main_get_have_been_ACC_ON_flag()))))
//        {
//            vl_force_to_exe[1] = 1;
//        }
//    }
    
    //�����ϵ��ӳٴ�����������
    //if(gl_Battery_delay_process_cnt>2500)
    //    return;
//    if((0 != sl_info_center_enable_delay_cnt)&&
//        (!Battery_is_all_bms_reg_valid_flag())&&
//        (0 != gl_Battery_delay_process_cnt))
//    {
//        return;
//    }
    

    //���NFC���ڷ�������򲻴�����������
    if(gl_bms_info_p[0]->waiting_cmd_ack||gl_bms_info_p[1]->waiting_cmd_ack)
        return ;

    if((is_battery_voltage_OK(0))&&(0 == is_battery_error(0)))//&&(0 == Battery_is_high_voltage_bat_err(0))) 
    {
        //���0����

        //����صĵ�ѹ���������Ƿ���Ч
        if(FALSE == is_battery_A_V_reg_valid(0))
        {
//            if(sl_valid_A_V_print_flag == 0)
//            {
//                DEBUG_PRINTF("BAT A V reg err, 0!\r\n");
//            }
            sl_valid_A_V_print_flag = 1;
            return ;
        }
        else
        {
//            if(sl_valid_A_V_print_flag)
//            {
//                DEBUG_PRINTF("BAT A V reg resume, 0!\r\n");
//            }
            sl_valid_A_V_print_flag = 0;
        }

//        Battery_clear_high_voltage_bat_err_param(0);

        vl_bms_0_A = Battery_get_current(0);
        //��������������500mA���ϣ���λ�Ƿ�����־λ
        if((vl_bms_0_A) < (-50))
        {
            gl_illegal_charging_in_flag = 0;
        }

        #ifdef PMS_SELFOFF_TIME_COUNTER_ENABLE
        if((vl_bms_0_A) < (PMS_SELFOFF_CURRENT_MAX))// �����������2500mA
        {
            main_pms_selfoff_detect_bat_current_out();
        }
        #endif

        #ifdef PMS_CMD_POWEROFF_TIME_COUNTER_ENABLE
        // ������� ����2.5A������������2A����Ϊ�����г�״̬
        if(((vl_bms_0_A) < (main_cmd_poweroff_get_A_out_value_max()))// �����������2500mA
            || ((vl_bms_0_A) > (200))) //����2A�ĳ��
        {
            if(main_cmd_poweroff_get_poweroff_flag_nonvolatilely())
            {
                //��ʱ�����̶ϵ�
                main_cmd_poweroff_set_cmd_param(1);
                main_cmd_poweroff_set_exe_mode(1);
            }
            else if((main_get_bicycle_stopped_status_for_smart()) && //ͣ����
                (ENUM_ACC_OFF == ACC_get_status())
                )
            {
                main_cmd_poweroff_set_exe_mode(1);
            }
            else
            {
                main_cmd_poweroff_counter_reload();
            }
        }
        else
        {
            //ACC OFFʱ���ж�Ϊͣ��״̬
            main_cmd_poweroff_when_accoff();
        }
        #endif

        
        // һ�����һ�Σ�����Ƿ��ǳ���������ڳ��
        if((vl_check_charger_current_in_flag_1_sec))//&&(Battery_check_charger_current_in(0)))
        {
//            DEBUG_PRINTF("Illegal charger in. 0\r\n");
            gl_illegal_charging_in_flag = 1;
            
            vl_check_charger_current_in_flag_1_sec = 0;
            //������ض��ǳ��������ĳ�����״̬
            // ���õ�ز�����ŵ�
            //Battery_set_PMS_working_state(0);
            //Battery_set_battery_sleep();
            //masked, 2018-6-30, ��⵽���ֻ�س��ܡ�
            //Battery_set_battery_sleep_by_delay(ENUM_BAT_SLEEP_SRC_ILLEGAL_CHARGER_0, 5000);
        }

        vl_pos = MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_CTRL);

//        if(sl_battery_need2sleep_flag)
//        {
//            #if 0
//            unsigned short vl_tmp0;
//            vl_tmp0 = Battery_get_switch_state(0);
//            //��Ҫ����˯��
//            if((vl_force_to_exe[0] == 0)&&
//                ((gl_bms_info_p[0]->reg_need_read &((1<<1)|(1<<2))) == ((1<<1)|(1<<2)))&&
//                ((gl_bms_info_p[0]->reg_unit[vl_pos].bits16_L & (1<<5)) != 0)&&
//                ((vl_tmp0 & 1) == 0)&&
//                (gl_cmd_buf[0][ENUM_NFC_CMD_INDEX_W_CTRL] == 0)&&
//                (gl_cmd_buf[0][ENUM_NFC_CMD_INDEX_R_CTRL] == 0)&&
//                (gl_cmd_buf[0][ENUM_NFC_CMD_INDEX_R_INFO1] == 0)&&
//                (gl_cmd_buf[0][ENUM_NFC_CMD_INDEX_R_INFO2] == 0))
//            {
//                //���Խ���˯����
//                main_mcu_sleep();
//            }
//            #else
//            Battery_bat0_check_exe_sleep(vl_force_to_exe);
//            #endif
//        }

        //��������������ص�ѹ���ʱ��Ĵ���״̬
        sl_supply_state = 0;

        vl_bms_0_ctrl = Battery_get_switch_state(0);
        if((gl_illegal_charging_in_flag)&&((BMS_STATE_BIT_CHG_SWITCH&vl_bms_0_ctrl)))
        {
            Battery_clear_switch_state(0, (BMS_STATE_BIT_CHG_SWITCH)); //�رյ�س���
        }
        //�򿪵��0�ĳ�ŵ翪��
        else if(vl_force_to_exe[0]||
            (((!sl_battery_need2sleep_flag)&&((BMS_STATE_BIT_CHG_SWITCH&vl_bms_0_ctrl) == 0))||
            ((BMS_STATE_BIT_SUPPLY_SWITCH&vl_bms_0_ctrl) == 0))||
                ((!sl_battery_need2sleep_flag)&&(0 != (gl_bms_info_p[0]->reg_unit[vl_pos].bits16_L&(1<<5)))))//�ر�˯��ģʽ
        {
            if(gl_illegal_charging_in_flag)
            {
                Bttery_check_pre_error_and_set_switch_state(0, BMS_STATE_BIT_SUPPLY_SWITCH);
            }
            else
            {
                Bttery_check_pre_error_and_set_switch_state(0, BMS_STATE_BIT_CHG_SWITCH| BMS_STATE_BIT_SUPPLY_SWITCH);
            }
            //DEBUG_PRINTF("only bms 0 voltage, open all\r\n");
        }
        #ifdef PMS_CMD_POWEROFF_TIME_COUNTER_ENABLE
        if(0 == main_cmd_poweroff_get_poweroff_flag_nonvolatilely())
        {
        #endif
        //����е�ش򿪣��ʹ򿪻�����ACC
//        if((vl_bms_0_ctrl & BMS_STATE_BIT_SUPPLY_SWITCH)&&
//            (ENUM_ACC_ON == ACC_get_status())&&
//            (
//            (1 == Battery_get_PMS_working_state())
//            ||(gl_pms_working_state_param.PMS_ACC_out_on_mandatory)
//            ||(gl_pms_working_state_param.BAT_HW_version_in_testing_mode)
//            )
//            )
//        {
//            ACC_ctrl_out_enable(1);
//        }
//        // ACC OFF ����£� ACC OUT���off; 2018-6-29
//        else if(gl_acc_wake_status == 0)
//        {
//            ACC_ctrl_out_enable(0);
//        }
        #ifdef PMS_CMD_POWEROFF_TIME_COUNTER_ENABLE
        }
        else
        {
            ACC_ctrl_out_enable(0);
        }
        #endif
    }
/////////////////////////////////////////////////////////////////////////////////////////////////////        

        //����صĳ�ŵ�ܺ�Ԥ�Ź��б仯ʱ�򣬴�ӡ��Ϣ
//        {
//            if(is_battery_voltage_OK(0))
//            {
//                static unsigned short sl_bms_0_ctrl_old = 0;
//                vl_bms_0_ctrl = Battery_get_switch_state(0);
//                
//                if((vl_bms_0_ctrl&0x07) != sl_bms_0_ctrl_old)
//                {
//                    unsigned char vl_str[32];
//                    sl_bms_0_ctrl_old = (vl_bms_0_ctrl&0x07);
//                    strcpy((char*)vl_str, "Bat 0 state: ");
//                    vl_str[13] = 0x30+sl_bms_0_ctrl_old;
//                    vl_str[14] = '\r';
//                    vl_str[15] = '\n';
//                    vl_str[16] = 0;
//                    DEBUG_PRINTF((char*)vl_str);
//                }
//            }

//            if(is_battery_voltage_OK(1))
//            {
//                static unsigned short sl_bms_1_ctrl_old = 0;
//                vl_bms_1_ctrl = Battery_get_switch_state(1);
//                
//                if((vl_bms_1_ctrl&0x07) != sl_bms_1_ctrl_old)
//                {
//                    unsigned char vl_str[32];
//                    sl_bms_1_ctrl_old = (vl_bms_1_ctrl&0x07);
//                    strcpy((char*)vl_str, "Bat 1 state: ");
//                    vl_str[13] = 0x30+sl_bms_1_ctrl_old;
//                    vl_str[14] = '\r';
//                    vl_str[15] = '\n';
//                    vl_str[16] = 0;
//                    DEBUG_PRINTF((char*)vl_str);
//                }
//            }
//        }

/////////////////////////////////////////////////////////////////////////////////////////////////////   
    //�жϵ���Ƿ�����
    //if(((is_battery_voltage_OK(0))&&(0 == is_battery_error(0)))&&
    //    ((is_battery_voltage_OK(1))&&(0 == is_battery_error(1))))
//    if(//(TRUE == slave_rs485_is_bat_valid(0xFF, 0))&&
//        (0 == is_battery_error(0))&&
//        //(TRUE == slave_rs485_is_bat_valid(0xFF, 1))&&
//        (0 == is_battery_error(1)))
//    {
//        //������ض�����

//        //���жϼĴ����Ƿ���Ч
//        if((0 == is_battery_voltage_OK(0))||
//            (0 == is_battery_voltage_OK(0)))
//        {
//            gl_Battery_voltage_shake_cnt = gl_Battery_voltage_shake_cnt_old;
//            return ;
//        }
//        gl_Battery_voltage_shake_cnt_old = gl_Battery_voltage_shake_cnt;
//        
//        vl_pos = MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_CTRL);

//        if(sl_battery_need2sleep_flag)
//        {
//            Battery_two_bats_check_exe_sleep(vl_force_to_exe);
//        }

//        //����صĵ�ѹ���������Ƿ���Ч
//        if((FALSE == is_battery_A_V_reg_valid(0))||
//            (FALSE == is_battery_A_V_reg_valid(1)))
//        {
//            if(sl_valid_A_V_print_flag == 0)
//            {
//                DEBUG_PRINTF("BAT A V reg err, 2!\r\n");
//            }
//            sl_valid_A_V_print_flag = 1;
//            return ;
//        }
//        else
//        {
//            if(sl_valid_A_V_print_flag)
//            {
//                DEBUG_PRINTF("BAT A V reg resume, 2!\r\n");
//            }
//            sl_valid_A_V_print_flag = 0;
//        }
//        
//        vl_bms_0_V = Battery_get_voltage(0);
//        vl_bms_1_V = Battery_get_voltage(1);

//        vl_bms_0_A = Battery_get_current(0);
//        vl_bms_1_A = Battery_get_current(1);

//        //��������������1A���ϣ���λ�Ƿ�����־λ
//        if((vl_bms_0_A+vl_bms_1_A) < (-100))
//        {
//            gl_illegal_charging_in_flag = 0;
//        }


//        
//        #ifdef PMS_SELFOFF_TIME_COUNTER_ENABLE
//        if((vl_bms_0_A+vl_bms_1_A) < (PMS_SELFOFF_CURRENT_MAX))// �����������2500mA
//        {
//            main_pms_selfoff_detect_bat_current_out();
//        }
//        #endif

//        #ifdef PMS_CMD_POWEROFF_TIME_COUNTER_ENABLE
//        // ������� ����2.5A������������2A����Ϊ�����г�״̬
//        if(((vl_bms_0_A+vl_bms_1_A) < (main_cmd_poweroff_get_A_out_value_max()))// �����������2500mA
//            || ((vl_bms_0_A+vl_bms_1_A) > (200))) //����2A�ĳ��
//        {
//            if(main_cmd_poweroff_get_poweroff_flag_nonvolatilely())
//            {
//                //��ʱ�����̶ϵ�
//                main_cmd_poweroff_set_cmd_param(1);
//                main_cmd_poweroff_set_exe_mode(1);
//            }
//            else if((main_get_bicycle_stopped_status_for_smart()) && //ͣ����
//                (ENUM_ACC_OFF == ACC_get_status())
//                )
//            {
//                main_cmd_poweroff_set_exe_mode(1);
//            }
//            else
//            {
//                main_cmd_poweroff_counter_reload();
//            }
//            
//        }
//        else
//        {
//            //ACC OFFʱ���ж�Ϊͣ��״̬
//            main_cmd_poweroff_when_accoff();
//        }
//        #endif
//        
//        vl_bms_0_ctrl = Battery_get_switch_state(0);
//        vl_bms_1_ctrl = Battery_get_switch_state(1);
//        
//        if(vl_bms_0_V >= vl_bms_1_V)
//        {
//            vl_voltage_diff = vl_bms_0_V - vl_bms_1_V;

//            //��������Ǹ�ֵ������ʱ��ֱ
//            vl_current_diff = vl_bms_1_A - vl_bms_0_A;
//        }
//        else
//        {
//            vl_voltage_diff = vl_bms_1_V - vl_bms_0_V;

//            //��������Ǹ�ֵ������ʱ��ֱ
//            vl_current_diff = vl_bms_0_A - vl_bms_1_A;
//        }
//        /////////////////////////////ȥ����//////////////////////////////////////////////////////
//        if(sl_supply_state == 1)
//        {
//            if((vl_voltage_diff < BATTERY_VOLTAGE_DIFFER_VALUE_BIG)&&
//               (vl_current_diff < 2500) )//BATTERY_VOLTAGE_DIFFER_VALUE)
//            {
//                gl_Battery_voltage_shake_cnt = BATTERY_VOLTAGE_SHAKE_CNT_MAX;
//            }
//            else if(!gl_Battery_voltage_shake_cnt)
//            {
//                // ��Ϊ��������
//                sl_supply_state = 2;
//                DEBUG_PRINTF("Changed to one Bat!\r\n");

//                sl_both_voltage_equ_flag = 0;
//            }
//        }
//        else if(sl_supply_state == 2)
//        {
//            unsigned char vl_current_flag = 0;
//            if((vl_bms_0_A > 20)||(vl_bms_0_A < -20))
//            {
//                vl_current_flag |= 0x01;
//            }

//            if((vl_bms_1_A > 20)||(vl_bms_1_A < -20))
//            {
//                vl_current_flag |= 0x02;
//            }
//            
//            if(//(vl_voltage_diff >= BATTERY_VOLTAGE_DIFFER_VALUE_EQU)||//BATTERY_VOLTAGE_DIFFER_VALUE)
//                (vl_current_diff >= 2000)||
//                (vl_current_flag != 0x03))
//            {
//                gl_Battery_voltage_shake_cnt = (BATTERY_VOLTAGE_SHAKE_CNT_MAX+BATTERY_VOLTAGE_SHAKE_CNT_MAX);//����ȥ��ʱ��
//            }
//            else if(!gl_Battery_voltage_shake_cnt)
//            {
//                // ��Ϊͬʱ����
//                sl_supply_state = 1;
//                DEBUG_PRINTF("Changed to two Bat!\r\n");
//                //CmdLine_Printf("Changed to two Bat!\r\n");
//            }
//        }
//        else
//        {
//            gl_Battery_voltage_shake_cnt = BATTERY_VOLTAGE_SHAKE_CNT_MAX;
//            
//            //if(vl_voltage_diff < BATTERY_VOLTAGE_DIFFER_VALUE_EQU)
//            //{
//            //    sl_supply_state = 1;
//            //}
//            //else
//            {
//                sl_supply_state = 2;

//                sl_both_voltage_equ_flag = 0;
//                sl_both_voltage_diff_flag = 0;
//            }
//        }
//        ////////////////////////////////////////////////////////////////////////////////////////////


//        //������ص�ѹ�Ƚ�
//        if(1 == sl_supply_state)//(vl_voltage_diff < BATTERY_VOLTAGE_DIFFER_VALUE)
//        {
//            //����ص�ѹ��С��0.5v

//            Battery_clear_high_voltage_bat_err_param(2);

//            // һ�����һ�Σ�����Ƿ��ǳ���������ڳ��
//            if((vl_check_charger_current_in_flag_1_sec)&&
//                (1 == Battery_check_two_chargers_current_in()))
//            {
//                DEBUG_PRINTF("Illegal charger in. 2\r\n");
//                gl_illegal_charging_in_flag = 1;
//                
//                vl_check_charger_current_in_flag_1_sec = 0;
//                //������ض��ǳ��������ĳ�����״̬
//                // ���õ�ز�����ŵ�
//                //Battery_set_PMS_working_state(0);
//                //Battery_set_battery_sleep();
//                //masked, 2018-6-30, ��⵽���ֻ�س��ܡ�
//                //Battery_set_battery_sleep_by_delay(ENUM_BAT_SLEEP_SRC_ILLEGAL_CHARGER_2, 5000);
//            }
//            
//            if(sl_both_voltage_equ_flag == 0)
//            {
//                sl_both_voltage_equ_flag = 1;
//                DEBUG_PRINTF("Both voltage, small different!\r\n");
//            }
//            sl_both_voltage_diff_flag = 0;

//            
//            //�Ƚ�����ص�BMS���ƼĴ����Ŀ���״̬�Ƿ����Ҫ��
//            //ͬʱ������صĳ�ŵ翪��
//            //if(0 == is_battery_error(0))
//            {
//                //û�д���
//                if((gl_illegal_charging_in_flag)&&(BMS_STATE_BIT_CHG_SWITCH&vl_bms_0_ctrl))
//                {
//                    Battery_clear_switch_state(0, (BMS_STATE_BIT_CHG_SWITCH)); //�رյ�س���
//                }
//                else if((gl_illegal_charging_in_flag)&&(BMS_STATE_BIT_CHG_SWITCH&vl_bms_1_ctrl))
//                {
//                    Battery_clear_switch_state(1, (BMS_STATE_BIT_CHG_SWITCH)); //�رյ�س���
//                }
//                else if(vl_force_to_exe[0]||
//                    ((!sl_battery_need2sleep_flag)&&((BMS_STATE_BIT_CHG_SWITCH&vl_bms_0_ctrl) == 0))||
//                    ((BMS_STATE_BIT_SUPPLY_SWITCH&vl_bms_0_ctrl) == 0)||
//                    ((!sl_battery_need2sleep_flag)&&(0 != (gl_bms_info_p[0]->reg_unit[vl_pos].bits16_L&(1<<5)))))//�ر�˯��ģʽ
//                {
//                    if(gl_illegal_charging_in_flag)
//                    {
//                        Bttery_check_pre_error_and_set_switch_state(0, BMS_STATE_BIT_SUPPLY_SWITCH);
//                    }
//                    else
//                    {
//                        Bttery_check_pre_error_and_set_switch_state(0, BMS_STATE_BIT_CHG_SWITCH| BMS_STATE_BIT_SUPPLY_SWITCH);
//                    }
//                    //CmdLine_Printf("bms 0=1 voltage, open all 0\r\n");
//                }
//            }

//            //if(0 == is_battery_error(1))
//            {
//                //û�д���
//                if((gl_illegal_charging_in_flag)&&(BMS_STATE_BIT_CHG_SWITCH&vl_bms_0_ctrl))
//                {
//                    Battery_clear_switch_state(0, (BMS_STATE_BIT_CHG_SWITCH)); //�رյ�س���
//                }
//                else if((gl_illegal_charging_in_flag)&&(BMS_STATE_BIT_CHG_SWITCH&vl_bms_1_ctrl))
//                {
//                    Battery_clear_switch_state(1, (BMS_STATE_BIT_CHG_SWITCH)); //�رյ�س���
//                }
//                else if(vl_force_to_exe[1]||
//                    ((!sl_battery_need2sleep_flag)&&((BMS_STATE_BIT_CHG_SWITCH&vl_bms_1_ctrl) == 0))||
//                    ((BMS_STATE_BIT_SUPPLY_SWITCH&vl_bms_1_ctrl) == 0)||
//                    ((!sl_battery_need2sleep_flag)&&(0 != (gl_bms_info_p[1]->reg_unit[vl_pos].bits16_L&(1<<5)))))//�ر�˯��ģʽ
//                {
//                    if(gl_illegal_charging_in_flag)
//                    {
//                        Bttery_check_pre_error_and_set_switch_state(1, BMS_STATE_BIT_SUPPLY_SWITCH);
//                    }
//                    else
//                    {
//                        Bttery_check_pre_error_and_set_switch_state(1, BMS_STATE_BIT_CHG_SWITCH| BMS_STATE_BIT_SUPPLY_SWITCH);
//                    }
//                    //CmdLine_Printf("bms 0=1 voltage, open all 1\r\n");
//                }
//            }

//            sl_bms0_vol_cmp_offset = 0;
//            sl_bms1_vol_cmp_offset = 0;
//        }
//        else if(2 == sl_supply_state)//if (gl_Battery_delay_process_cnt) //gl_Battery_delay_process_cnt�ӳٺ󣬲����ٲ���
//        {
//             
//            //�����ѹ�����0.5V
//            if(sl_both_voltage_diff_flag == 0)
//            {
//                sl_both_voltage_diff_flag = 1;
//                DEBUG_PRINTF("Both voltage, big different!\r\n");
//            }
//            sl_both_voltage_equ_flag = 0;

//            Battery_check_high_voltage_bat_err(0);
//            Battery_check_high_voltage_bat_err(1);
//            
//            //�Ƚ�����ص�BMS���ƼĴ����Ŀ���״̬�Ƿ����Ҫ��

//            //�رյ͵�ѹ�ĵ�س�ŵ翪��
//            if((vl_bms_0_V - sl_bms0_vol_cmp_offset)> (vl_bms_1_V - sl_bms1_vol_cmp_offset))
//            {
//                sl_bms0_vol_cmp_offset = 0;
//                sl_bms1_vol_cmp_offset = 100;

//                // һ�����һ�Σ�����Ƿ��ǳ���������ڳ��
//                if((vl_check_charger_current_in_flag_1_sec)&&
//                    (Battery_check_charger_current_in(0)))
//                {
//                    DEBUG_PRINTF("Illegal charger in. 2.0\r\n");
//                    gl_illegal_charging_in_flag = 1;
//                    
//                    vl_check_charger_current_in_flag_1_sec = 0;
//                    //������ض��ǳ��������ĳ�����״̬
//                    // ���õ�ز�����ŵ�
//                    //Battery_set_PMS_working_state(0);
//                    //Battery_set_battery_sleep();
//                    //masked, 2018-6-30, ��⵽���ֻ�س��ܡ�
//                    //Battery_set_battery_sleep_by_delay(ENUM_BAT_SLEEP_SRC_ILLEGAL_CHARGER_2_0, 5000);
//                }
//                
//                //DEBUG_PRINTF("bms 1 voltage, small!\r\n");
//                if(Battery_is_high_voltage_bat_err(0))
//                {
//                    if((gl_illegal_charging_in_flag)&&(BMS_STATE_BIT_CHG_SWITCH&vl_bms_0_ctrl))
//                    {
//                        Battery_clear_switch_state(0, (BMS_STATE_BIT_CHG_SWITCH)); //�رյ�س���
//                    }
//                    else if((gl_illegal_charging_in_flag)&&(BMS_STATE_BIT_CHG_SWITCH&vl_bms_1_ctrl))
//                    {
//                        Battery_clear_switch_state(1, (BMS_STATE_BIT_CHG_SWITCH)); //�رյ�س���
//                    }
//                    else if(vl_force_to_exe[0]||
//                        (vl_bms_0_ctrl & (BMS_STATE_BIT_CHG_SWITCH|BMS_STATE_BIT_SUPPLY_SWITCH))||
//                    ((!sl_battery_need2sleep_flag)&&(0 != (gl_bms_info_p[0]->reg_unit[vl_pos].bits16_L&(1<<5)))))//�ر�˯��ģʽ
//                    {
//                        if((vl_bms_1_ctrl & BMS_STATE_BIT_SUPPLY_SWITCH) == 0)
//                        {
//                            //�͵�ѹ��صķŵ���ǹرյģ��ȴ�
//                            Bttery_check_pre_error_and_set_switch_state(1, BMS_STATE_BIT_SUPPLY_SWITCH);
//                        }
//                        else
//                        {
//                            //�ߵ�ѹ�ĳ��ܻ��߷ŵ���Ǵ򿪵ģ��رճ�ŵ��
//                            Battery_clear_switch_state(0, (BMS_STATE_BIT_CHG_SWITCH|BMS_STATE_BIT_SUPPLY_SWITCH));
//                        }
//                    }
//                    else if(vl_force_to_exe[1]||((vl_bms_1_ctrl & BMS_STATE_BIT_SUPPLY_SWITCH) == 0)||((vl_bms_1_ctrl & BMS_STATE_BIT_CHG_SWITCH) == 0))
//                    {
//                        //�͵�ѹ��صķŵ翪�ػ��߳�翪���ǹرյģ���ʱҪ�󶼴򿪴�
//                        if(gl_illegal_charging_in_flag)
//                        {
//                            Bttery_check_pre_error_and_set_switch_state(1, BMS_STATE_BIT_SUPPLY_SWITCH);
//                        }
//                        else
//                        {
//                            Bttery_check_pre_error_and_set_switch_state(1, BMS_STATE_BIT_SUPPLY_SWITCH|BMS_STATE_BIT_CHG_SWITCH);
//                        }
//                        
//                    }                
//                }
//                else
//                {
//                    if((gl_illegal_charging_in_flag)&&(BMS_STATE_BIT_CHG_SWITCH&vl_bms_0_ctrl))
//                    {
//                        Battery_clear_switch_state(0, (BMS_STATE_BIT_CHG_SWITCH)); //�رյ�س���
//                    }
//                    else if((gl_illegal_charging_in_flag)&&(BMS_STATE_BIT_CHG_SWITCH&vl_bms_1_ctrl))
//                    {
//                        Battery_clear_switch_state(1, (BMS_STATE_BIT_CHG_SWITCH)); //�رյ�س���
//                    }
//                    else if(vl_force_to_exe[1]||
//                        (vl_bms_1_ctrl & BMS_STATE_BIT_CHG_SWITCH)||
//                    ((!sl_battery_need2sleep_flag)&&(0 != (gl_bms_info_p[1]->reg_unit[vl_pos].bits16_L&(1<<5)))))//�ر�˯��ģʽ
//                    {
//                        //�͵�ѹ��صĳ�翪���ǵ�ͨ�ģ���ʱҪ��ر�
//                        Battery_clear_switch_state(1, BMS_STATE_BIT_CHG_SWITCH);
//                        //DEBUG_PRINTF("bms 1 voltage, small, close charge 1\r\n");
//                    }
//                    else if(vl_force_to_exe[0]||((vl_bms_0_ctrl & BMS_STATE_BIT_SUPPLY_SWITCH) == 0))
//                    {
//                        //�ߵ�ѹ��صķŵ翪���ǹرյģ���ʱҪ���
//                        Bttery_check_pre_error_and_set_switch_state(0, BMS_STATE_BIT_SUPPLY_SWITCH);
//                        //DEBUG_PRINTF("bms 1 voltage, small, open supply 0\r\n");
//                    }                
//                    else if((vl_bms_1_ctrl & BMS_STATE_BIT_SUPPLY_SWITCH) == 0)
//                    {
//                        //�͵�ѹ��صķŵ翪���ǹرյģ���ʱҪ���
//                        Bttery_check_pre_error_and_set_switch_state(1, BMS_STATE_BIT_SUPPLY_SWITCH);
//                        //DEBUG_PRINTF("bms 1 voltage, small, open supply 1\r\n");
//                    }                
//                    else if((vl_bms_0_ctrl & BMS_STATE_BIT_CHG_SWITCH) == 0)
//                    {
//                        //�ߵ�ѹ��صĳ�翪���ǹرյģ���ʱҪ���
//                        if(0 == gl_illegal_charging_in_flag)
//                        {
//                            Bttery_check_pre_error_and_set_switch_state(0, BMS_STATE_BIT_CHG_SWITCH);
//                        }
//                        //DEBUG_PRINTF("bms 1 voltage, small, open charge 0\r\n");
//                    }
//                }
//            }
//            else
//            {            
//                sl_bms0_vol_cmp_offset = 100;
//                sl_bms1_vol_cmp_offset = 0;
//                
//                //DEBUG_PRINTF("bms 0 voltage, small!\r\n");
//                // һ�����һ�Σ�����Ƿ��ǳ���������ڳ��
//                if((vl_check_charger_current_in_flag_1_sec)&&
//                    (Battery_check_charger_current_in(1)))
//                {
//                    DEBUG_PRINTF("Illegal charger in. 2.1\r\n");
//                    gl_illegal_charging_in_flag = 1;
//                    
//                    vl_check_charger_current_in_flag_1_sec = 0;
//                    //������ض��ǳ��������ĳ�����״̬
//                    // ���õ�ز�����ŵ�
//                    //Battery_set_PMS_working_state(0);
//                    //Battery_set_battery_sleep();
//                    //masked, 2018-6-30, ��⵽���ֻ�س��ܡ�
//                    //Battery_set_battery_sleep_by_delay(ENUM_BAT_SLEEP_SRC_ILLEGAL_CHARGER_2_1, 5000);
//                }

//                if(Battery_is_high_voltage_bat_err(1))
//                {
//                    if((gl_illegal_charging_in_flag)&&(vl_bms_0_ctrl & BMS_STATE_BIT_CHG_SWITCH))
//                    {
//                        Battery_clear_switch_state(0, (BMS_STATE_BIT_CHG_SWITCH));
//                    }
//                    else if((gl_illegal_charging_in_flag)&&(vl_bms_1_ctrl & BMS_STATE_BIT_CHG_SWITCH))
//                    {
//                        Battery_clear_switch_state(1, (BMS_STATE_BIT_CHG_SWITCH));
//                    }
//                    else if(vl_force_to_exe[1]||
//                        (vl_bms_1_ctrl & (BMS_STATE_BIT_CHG_SWITCH|BMS_STATE_BIT_SUPPLY_SWITCH))||
//                    ((!sl_battery_need2sleep_flag)&&(0 != (gl_bms_info_p[1]->reg_unit[vl_pos].bits16_L&(1<<5)))))//�ر�˯��ģʽ
//                    {
//                        if((vl_bms_0_ctrl & BMS_STATE_BIT_SUPPLY_SWITCH) == 0)
//                        {
//                            //�͵�ѹ��صķŵ���ǹرյģ��ȴ�
//                            Bttery_check_pre_error_and_set_switch_state(0, BMS_STATE_BIT_SUPPLY_SWITCH);
//                        }
//                        else
//                        {
//                            //�ߵ�ѹ�ĳ��ܻ��߷ŵ���Ǵ򿪵ģ��رճ�ŵ��
//                            Battery_clear_switch_state(1, (BMS_STATE_BIT_CHG_SWITCH|BMS_STATE_BIT_SUPPLY_SWITCH));
//                        }
//                    }
//                    else if(vl_force_to_exe[0]||
//                        ((vl_bms_0_ctrl & BMS_STATE_BIT_SUPPLY_SWITCH) == 0)||
//                        ((gl_illegal_charging_in_flag == 0)&&((vl_bms_0_ctrl & BMS_STATE_BIT_CHG_SWITCH) == 0)))
//                    {
//                        //�͵�ѹ��صķŵ翪�ػ��߳�翪���ǹرյģ���ʱҪ�󶼴򿪴�
//                        if(gl_illegal_charging_in_flag)
//                        {
//                            Bttery_check_pre_error_and_set_switch_state(0, BMS_STATE_BIT_SUPPLY_SWITCH);
//                        }
//                        else
//                        {
//                            Bttery_check_pre_error_and_set_switch_state(0, BMS_STATE_BIT_SUPPLY_SWITCH|BMS_STATE_BIT_CHG_SWITCH);
//                        }
//                    }                
//                }
//                else
//                {
//                    if((gl_illegal_charging_in_flag)&&(vl_bms_0_ctrl & BMS_STATE_BIT_CHG_SWITCH))
//                    {
//                        Battery_clear_switch_state(0, (BMS_STATE_BIT_CHG_SWITCH));
//                    }
//                    else if((gl_illegal_charging_in_flag)&&(vl_bms_1_ctrl & BMS_STATE_BIT_CHG_SWITCH))
//                    {
//                        Battery_clear_switch_state(1, (BMS_STATE_BIT_CHG_SWITCH));
//                    }
//                    else if(vl_force_to_exe[0]||
//                        (vl_bms_0_ctrl & BMS_STATE_BIT_CHG_SWITCH)||
//                    ((!sl_battery_need2sleep_flag)&&(0 != (gl_bms_info_p[0]->reg_unit[vl_pos].bits16_L&(1<<5)))))//�ر�˯��ģʽ
//                    {
//                        //�͵�ѹ��صĳ�翪���ǵ�ͨ�ģ���ʱҪ��ر�
//                        Battery_clear_switch_state(0, BMS_STATE_BIT_CHG_SWITCH);
//                        //DEBUG_PRINTF("bms 0 voltage, small, close charge 0\r\n");
//                    }
//                    else if(vl_force_to_exe[1]||((vl_bms_1_ctrl & BMS_STATE_BIT_SUPPLY_SWITCH) == 0))
//                    {
//                        //�ߵ�ѹ��صķŵ翪���ǹرյģ���ʱҪ���
//                        Bttery_check_pre_error_and_set_switch_state(1, BMS_STATE_BIT_SUPPLY_SWITCH);
//                        //DEBUG_PRINTF("bms 0 voltage, small, open supply 1\r\n");
//                    }
//                    else if((vl_bms_0_ctrl & BMS_STATE_BIT_SUPPLY_SWITCH) == 0)
//                    {
//                        //�͵�ѹ��صķŵ翪���ǹرյģ���ʱҪ���
//                        Bttery_check_pre_error_and_set_switch_state(0, BMS_STATE_BIT_SUPPLY_SWITCH);
//                        //DEBUG_PRINTF("bms 0 voltage, small, open supply 0\r\n");
//                    }
//                    else if((vl_bms_1_ctrl & BMS_STATE_BIT_CHG_SWITCH) == 0)
//                    {
//                        //�ߵ�ѹ��صĳ�翪���ǹرյģ���ʱҪ���
//                        if(gl_illegal_charging_in_flag == 0)
//                        {
//                            Bttery_check_pre_error_and_set_switch_state(1, BMS_STATE_BIT_CHG_SWITCH);
//                        }
//                        //DEBUG_PRINTF("bms 0 voltage, small, open charge 1\r\n");
//                    }
//                }
//            }
//            
//        }

//        #ifdef PMS_CMD_POWEROFF_TIME_COUNTER_ENABLE
//        if(0 == main_cmd_poweroff_get_poweroff_flag_nonvolatilely())
//        {
//        #endif
//        //����е�ش򿪷ŵ�ܣ��ʹ򿪻�����ACC
//        if(((vl_bms_0_ctrl & BMS_STATE_BIT_SUPPLY_SWITCH) || (vl_bms_1_ctrl & BMS_STATE_BIT_SUPPLY_SWITCH))&&
//            (ENUM_ACC_ON == ACC_get_status())&&
//            (
//            (1 == Battery_get_PMS_working_state())
//            ||(gl_pms_working_state_param.PMS_ACC_out_on_mandatory)
//            ||(gl_pms_working_state_param.BAT_HW_version_in_testing_mode)
//            )
//            )
//        {
//            ACC_ctrl_out_enable(1);
//        }
//        // ACC OFF ����£� ACC OUT���off; 2018-6-29
//        else if(gl_acc_wake_status == 0)
//        {
//            ACC_ctrl_out_enable(0);
//        }
//        #ifdef PMS_CMD_POWEROFF_TIME_COUNTER_ENABLE
//        }
//        else 
//        {
//            ACC_ctrl_out_enable(0);
//        }
//        #endif
//    }
    //else if(gl_Battery_delay_process_cnt && is_battery_voltage_OK(0)) //gl_Battery_delay_process_cnt�ӳٺ󣬲����ٲ���
//    else 
//		if((is_battery_voltage_OK(0))&&(0 == is_battery_error(0))&&(0 == Battery_is_high_voltage_bat_err(0))) 
//    {
//        //���0����

//        //����صĵ�ѹ���������Ƿ���Ч
//        if(FALSE == is_battery_A_V_reg_valid(0))
//        {
//            if(sl_valid_A_V_print_flag == 0)
//            {
//                DEBUG_PRINTF("BAT A V reg err, 0!\r\n");
//            }
//            sl_valid_A_V_print_flag = 1;
//            return ;
//        }
//        else
//        {
//            if(sl_valid_A_V_print_flag)
//            {
//                DEBUG_PRINTF("BAT A V reg resume, 0!\r\n");
//            }
//            sl_valid_A_V_print_flag = 0;
//        }

//        Battery_clear_high_voltage_bat_err_param(0);

//        vl_bms_0_A = Battery_get_current(0);
//        //��������������500mA���ϣ���λ�Ƿ�����־λ
//        if((vl_bms_0_A) < (-50))
//        {
//            gl_illegal_charging_in_flag = 0;
//        }

//        #ifdef PMS_SELFOFF_TIME_COUNTER_ENABLE
//        if((vl_bms_0_A) < (PMS_SELFOFF_CURRENT_MAX))// �����������2500mA
//        {
//            main_pms_selfoff_detect_bat_current_out();
//        }
//        #endif

//        #ifdef PMS_CMD_POWEROFF_TIME_COUNTER_ENABLE
//        // ������� ����2.5A������������2A����Ϊ�����г�״̬
//        if(((vl_bms_0_A) < (main_cmd_poweroff_get_A_out_value_max()))// �����������2500mA
//            || ((vl_bms_0_A) > (200))) //����2A�ĳ��
//        {
//            if(main_cmd_poweroff_get_poweroff_flag_nonvolatilely())
//            {
//                //��ʱ�����̶ϵ�
//                main_cmd_poweroff_set_cmd_param(1);
//                main_cmd_poweroff_set_exe_mode(1);
//            }
//            else if((main_get_bicycle_stopped_status_for_smart()) && //ͣ����
//                (ENUM_ACC_OFF == ACC_get_status())
//                )
//            {
//                main_cmd_poweroff_set_exe_mode(1);
//            }
//            else
//            {
//                main_cmd_poweroff_counter_reload();
//            }
//        }
//        else
//        {
//            //ACC OFFʱ���ж�Ϊͣ��״̬
//            main_cmd_poweroff_when_accoff();
//        }
//        #endif

//        
//        // һ�����һ�Σ�����Ƿ��ǳ���������ڳ��
//        if((vl_check_charger_current_in_flag_1_sec)&&(Battery_check_charger_current_in(0)))
//        {
//            DEBUG_PRINTF("Illegal charger in. 0\r\n");
//            gl_illegal_charging_in_flag = 1;
//            
//            vl_check_charger_current_in_flag_1_sec = 0;
//            //������ض��ǳ��������ĳ�����״̬
//            // ���õ�ز�����ŵ�
//            //Battery_set_PMS_working_state(0);
//            //Battery_set_battery_sleep();
//            //masked, 2018-6-30, ��⵽���ֻ�س��ܡ�
//            //Battery_set_battery_sleep_by_delay(ENUM_BAT_SLEEP_SRC_ILLEGAL_CHARGER_0, 5000);
//        }

//        vl_pos = MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_CTRL);

//        if(sl_battery_need2sleep_flag)
//        {
//            #if 0
//            unsigned short vl_tmp0;
//            vl_tmp0 = Battery_get_switch_state(0);
//            //��Ҫ����˯��
//            if((vl_force_to_exe[0] == 0)&&
//                ((gl_bms_info_p[0]->reg_need_read &((1<<1)|(1<<2))) == ((1<<1)|(1<<2)))&&
//                ((gl_bms_info_p[0]->reg_unit[vl_pos].bits16_L & (1<<5)) != 0)&&
//                ((vl_tmp0 & 1) == 0)&&
//                (gl_cmd_buf[0][ENUM_NFC_CMD_INDEX_W_CTRL] == 0)&&
//                (gl_cmd_buf[0][ENUM_NFC_CMD_INDEX_R_CTRL] == 0)&&
//                (gl_cmd_buf[0][ENUM_NFC_CMD_INDEX_R_INFO1] == 0)&&
//                (gl_cmd_buf[0][ENUM_NFC_CMD_INDEX_R_INFO2] == 0))
//            {
//                //���Խ���˯����
//                main_mcu_sleep();
//            }
//            #else
//            Battery_bat0_check_exe_sleep(vl_force_to_exe);
//            #endif
//        }

//        //��������������ص�ѹ���ʱ��Ĵ���״̬
//        sl_supply_state = 0;

//        vl_bms_0_ctrl = Battery_get_switch_state(0);
//        if((gl_illegal_charging_in_flag)&&((BMS_STATE_BIT_CHG_SWITCH&vl_bms_0_ctrl)))
//        {
//            Battery_clear_switch_state(0, (BMS_STATE_BIT_CHG_SWITCH)); //�رյ�س���
//        }
//        //�򿪵��0�ĳ�ŵ翪��
//        else if(vl_force_to_exe[0]||
//            (((!sl_battery_need2sleep_flag)&&((BMS_STATE_BIT_CHG_SWITCH&vl_bms_0_ctrl) == 0))||
//            ((BMS_STATE_BIT_SUPPLY_SWITCH&vl_bms_0_ctrl) == 0))||
//                ((!sl_battery_need2sleep_flag)&&(0 != (gl_bms_info_p[0]->reg_unit[vl_pos].bits16_L&(1<<5)))))//�ر�˯��ģʽ
//        {
//            if(gl_illegal_charging_in_flag)
//            {
//                Bttery_check_pre_error_and_set_switch_state(0, BMS_STATE_BIT_SUPPLY_SWITCH);
//            }
//            else
//            {
//                Bttery_check_pre_error_and_set_switch_state(0, BMS_STATE_BIT_CHG_SWITCH| BMS_STATE_BIT_SUPPLY_SWITCH);
//            }
//            //DEBUG_PRINTF("only bms 0 voltage, open all\r\n");
//        }
//        #ifdef PMS_CMD_POWEROFF_TIME_COUNTER_ENABLE
//        if(0 == main_cmd_poweroff_get_poweroff_flag_nonvolatilely())
//        {
//        #endif
//        //����е�ش򿪣��ʹ򿪻�����ACC
//        if((vl_bms_0_ctrl & BMS_STATE_BIT_SUPPLY_SWITCH)&&
//            (ENUM_ACC_ON == ACC_get_status())&&
//            (
//            (1 == Battery_get_PMS_working_state())
//            ||(gl_pms_working_state_param.PMS_ACC_out_on_mandatory)
//            ||(gl_pms_working_state_param.BAT_HW_version_in_testing_mode)
//            )
//            )
//        {
//            ACC_ctrl_out_enable(1);
//        }
//        // ACC OFF ����£� ACC OUT���off; 2018-6-29
//        else if(gl_acc_wake_status == 0)
//        {
//            ACC_ctrl_out_enable(0);
//        }
//        #ifdef PMS_CMD_POWEROFF_TIME_COUNTER_ENABLE
//        }
//        else
//        {
//            ACC_ctrl_out_enable(0);
//        }
//        #endif
//    }
    //else if(gl_Battery_delay_process_cnt && is_battery_voltage_OK(1))//gl_Battery_delay_process_cnt�ӳٺ󣬲����ٲ���
//    else if((is_battery_voltage_OK(1))&&(0 == is_battery_error(1))&&(0 == Battery_is_high_voltage_bat_err(1)))
//    {
//        //���1����

//        //����صĵ�ѹ���������Ƿ���Ч
//        if(FALSE == is_battery_A_V_reg_valid(1))
//        {
//            if(sl_valid_A_V_print_flag == 0)
//            {
//                DEBUG_PRINTF("BAT A V reg err, 1!\r\n");
//            }
//            sl_valid_A_V_print_flag = 1;
//            return ;
//        }
//        else
//        {
//            if(sl_valid_A_V_print_flag)
//            {
//                DEBUG_PRINTF("BAT A V reg resume, 1!\r\n");
//            }
//            sl_valid_A_V_print_flag = 0;
//        }

//        Battery_clear_high_voltage_bat_err_param(1);

//        vl_bms_1_A = Battery_get_current(1);
//        //��������������500mA���ϣ���λ�Ƿ�����־λ
//        if((vl_bms_1_A) < (-50))
//        {
//            gl_illegal_charging_in_flag = 0;
//        }

//        #ifdef PMS_SELFOFF_TIME_COUNTER_ENABLE
//        if((vl_bms_1_A) < (PMS_SELFOFF_CURRENT_MAX))// �����������2500mA
//        {
//            main_pms_selfoff_detect_bat_current_out();
//        }
//        #endif

//        #ifdef PMS_CMD_POWEROFF_TIME_COUNTER_ENABLE
//        // ������� ����2.5A������������2A����Ϊ�����г�״̬
//        if(((vl_bms_1_A) < (main_cmd_poweroff_get_A_out_value_max()))// �����������2500mA
//            || ((vl_bms_1_A) > (200))) //����2A�ĳ��
//        {
//            if(main_cmd_poweroff_get_poweroff_flag_nonvolatilely())
//            {
//                //��ʱ�����̶ϵ�
//                main_cmd_poweroff_set_cmd_param(1);
//                main_cmd_poweroff_set_exe_mode(1);
//            }
//            else if((main_get_bicycle_stopped_status_for_smart()) && //ͣ����
//                (ENUM_ACC_OFF == ACC_get_status())
//                )
//            {
//                main_cmd_poweroff_set_exe_mode(1);
//            }
//            else
//            {
//                main_cmd_poweroff_counter_reload();
//            }
//        }
//        else
//        {
//            //ACC OFFʱ���ж�Ϊͣ��״̬
//            main_cmd_poweroff_when_accoff();
//        }
//        #endif
//        
//        // һ�����һ�Σ�����Ƿ��ǳ���������ڳ��
//        if((vl_check_charger_current_in_flag_1_sec)&&(Battery_check_charger_current_in(1)))
//        {
//            DEBUG_PRINTF("Illegal charger in. 1\r\n");
//            gl_illegal_charging_in_flag = 1;
//            
//            vl_check_charger_current_in_flag_1_sec = 0;
//            //������ض��ǳ��������ĳ�����״̬
//            // ���õ�ز�����ŵ�
//            //Battery_set_PMS_working_state(0);
//            //Battery_set_battery_sleep();
//            //masked, 2018-6-30, ��⵽���ֻ�س��ܡ�
//            //Battery_set_battery_sleep_by_delay(ENUM_BAT_SLEEP_SRC_ILLEGAL_CHARGER_1, 5000);
//        }

//        vl_pos = MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_CTRL);

//        if(sl_battery_need2sleep_flag)
//        {

//            Battery_bat1_check_exe_sleep(vl_force_to_exe);
//        }

//        //��������������ص�ѹ���ʱ��Ĵ���״̬
//        sl_supply_state = 0;
//        
//        vl_bms_1_ctrl = Battery_get_switch_state(1);

//        //�򿪵��1�ĳ�ŵ翪��
//        if((gl_illegal_charging_in_flag)&&((BMS_STATE_BIT_CHG_SWITCH&vl_bms_1_ctrl)))
//        {
//            Battery_clear_switch_state(1, (BMS_STATE_BIT_CHG_SWITCH)); //�رյ�س���
//        }
//        else if(vl_force_to_exe[1]||
//            (((!sl_battery_need2sleep_flag)&&((BMS_STATE_BIT_CHG_SWITCH&vl_bms_1_ctrl) == 0))||
//            ((BMS_STATE_BIT_SUPPLY_SWITCH&vl_bms_1_ctrl) == 0))||
//                ((!sl_battery_need2sleep_flag)&&(0 != (gl_bms_info_p[1]->reg_unit[vl_pos].bits16_L&(1<<5)))))//�ر�˯��ģʽ
//        {
//            if(gl_illegal_charging_in_flag)
//            {
//                Bttery_check_pre_error_and_set_switch_state(1, BMS_STATE_BIT_SUPPLY_SWITCH);
//            }
//            else
//            {
//                Bttery_check_pre_error_and_set_switch_state(1, BMS_STATE_BIT_CHG_SWITCH| BMS_STATE_BIT_SUPPLY_SWITCH);
//            }
//            //DEBUG_PRINTF("only bms 1 voltage, open all\r\n");
//        }
//        #ifdef PMS_CMD_POWEROFF_TIME_COUNTER_ENABLE
//        if(0 == main_cmd_poweroff_get_poweroff_flag_nonvolatilely())
//        {
//        #endif
//        //����е�ش򿪣��ʹ򿪻�����ACC
//        if((vl_bms_1_ctrl & BMS_STATE_BIT_SUPPLY_SWITCH)&&
//            (ENUM_ACC_ON == ACC_get_status())&&
//            (
//            (1 == Battery_get_PMS_working_state())
//            ||(gl_pms_working_state_param.PMS_ACC_out_on_mandatory)
//            ||(gl_pms_working_state_param.BAT_HW_version_in_testing_mode)
//            )
//            )
//        {
//            ACC_ctrl_out_enable(1);
//        }
//        // ACC OFF ����£� ACC OUT���off; 2018-6-29
//        else if(gl_acc_wake_status == 0)
//        {
//            ACC_ctrl_out_enable(0);
//        }
//        #ifdef PMS_CMD_POWEROFF_TIME_COUNTER_ENABLE
//        }
//        else
//        {
//            ACC_ctrl_out_enable(0);
//        }
//        #endif
//    }
    else
    {
        //��ع���

//        if(sl_battery_need2sleep_flag)
//        {
//            
//            //��Ҫ����˯��
//            //���Խ���˯����
//            if((is_battery_voltage_OK(0))&&
//                (is_battery_voltage_OK(1)))
//            {
//                Battery_two_bats_check_exe_sleep(vl_force_to_exe);
//            }
//            else if(is_battery_voltage_OK(0))
//            {
//                //���0����

//                Battery_bat0_check_exe_sleep(vl_force_to_exe);
//            }
//            else if(is_battery_voltage_OK(1))
//            {
//                //���1����

//                Battery_bat1_check_exe_sleep(vl_force_to_exe);
//            }
//            else
//            {
//                DEBUG_PRINTF(".\r\n");
//            }
//        }    
    }
}


#if 0
//�Ƿ����������Ѿ��������
static bool Battery_is_cmd_buf_empty(void)
{
    unsigned short i, k;
    unsigned char *vl_p;

    vl_p = (unsigned char*)gl_cmd_buf;
    for(i = 0; i < (NFC_READER_COUNT_MAX*ENUM_NFC_CMD_INDEX_MAX); i++)
    {
        if(vl_p[i])
        {
            return FALSE;
        }
    }

    return TRUE;
}
#endif


void Battery_info_polling_Process(unsigned char bms_index)
{
    //������ѯһ�ε�ذ汾��Ϣ
    //if(vl_bat_info_flag[0]==0)
    {
        gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_VERSION] = 1;
    }
    
    //��ѯBMS ֻ����Ϣ���ӼĴ���256��308
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_INFO1] = 1;
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_INFO2] = 1;
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_CTRL] = 1;

    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_PARAM1] = 1;
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_PARAM2] = 1;
 

    //������ѯһ��
    {
        gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_USERDATA] = 1;
    }

//    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_BLV] = 1;

    #ifdef BAT_PMS_AUTHORITY_FUNCTION_EN
    //���˫����֤����
    Authority_process();
    #endif

    //��ȡ�����ʷ��¼����
//    if((0 == gl_bat_history_record_param.is_earliest_param_valid)
//        ||(0 == gl_bat_history_record_param.is_lastest_param_valid))
//    {
//        gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_HISTORY_RECORD] = 1;
//    }
    
    return ;

}


/*

battery_index: 1����ʾ���1�� 2����ʾ���2
return: 0����ʾ��ѹ��Ϣ�����ã�1����ʾ�����Ϣ��Ч
*/
unsigned char is_battery_voltage_OK(unsigned char bms_index)
{
    if((gl_bms_info_p[bms_index]->online)&&
        ((BMS_READONLY_DATA1_FLAG_BIT|BMS_READONLY_DATA2_FLAG_BIT)&Battery_get_reg_valid_flag(bms_index))&&
        (BMS_CTRL_FLAG_BIT&Battery_get_reg_valid_flag(bms_index)))
        return 1;
    else
        return 0;
}

//���ǲ����ٴ򿪳�ŵ��
unsigned char is_battery_error(unsigned char bms_index)
{
    unsigned short vl_pos, vl_value;
    vl_pos = MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_OPFT1);
    vl_value = ((gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_H<<8) + 
        gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_L);
    if(vl_value&(~((1<<15)|(1<<14)|(1<<0)|(1<<3)|(1<<6)|(1<<8)|(1<<10))))
    {
        return 1;
    }

    vl_pos = MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_OPFT2);
    vl_value = ((gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_H<<8) + 
        gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_L);
    if(vl_value&(~(1<<2)))
    {
        return 1;
    }

    vl_pos = MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_DEVFT1);
    vl_value = ((gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_H<<8) + 
        gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_L);
    if(vl_value&(~((1<<7)|(1<<8))))
    {
        return 1;
    }

    vl_pos = MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_DEVFT2);
    vl_value = ((gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_H<<8) + 
        gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_L);
    if(vl_value)
    {
        return 1;
    }
    
    return 0;
}
//����Ƿ��صĵ�ѹ�͵����Ĵ�����Ч
bool is_battery_A_V_reg_valid(unsigned char bms_index)
{
    
    unsigned short vl_pos;
    //unsigned int vl_V;
    //int vl_A;
    unsigned short vl_tmp;
    vl_pos = MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_TVOLT);

    vl_tmp = ((gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_H<<8) + 
        gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_L);
    if(vl_tmp == 0xFFFF)
        return FALSE;
    
    vl_pos = MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_TCURR);
    vl_tmp = ((gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_H<<8) + 
        gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_L);
    if(vl_tmp == 0xFFFF)
        return FALSE;

    return TRUE;
}

//�Ƿ�Ԥ��ʧ�ܣ���Ҫ�����ŵ�Ĵ���������������ܴ򿪳�ŵ��
unsigned char is_battery_presupply_error(unsigned char bms_index)
{
    unsigned short vl_pos, vl_value;
    vl_pos = MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_OPFT1);
    vl_value = ((gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_H<<8) + 
        gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_L);
    if(vl_value&(1<<15))
    {
        return 1;
    }

    return 0;
}





static unsigned short gl_bat_critical_error_reg_state[12];
void Battery_save_bat_reg_state_when_critical_error(unsigned char bms_index)
{
#ifndef CABINET_WARMOR_SUPPLY_BY_CHARGER

    gl_bat_critical_error_reg_state[0] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_SN34);
    gl_bat_critical_error_reg_state[1] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_SN56);
    gl_bat_critical_error_reg_state[2] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_SN78);
    gl_bat_critical_error_reg_state[3] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_STATE);
    gl_bat_critical_error_reg_state[4] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_CTRL);
    gl_bat_critical_error_reg_state[5] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_DEVFT1);
    gl_bat_critical_error_reg_state[6] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_DEVFT2);    
    gl_bat_critical_error_reg_state[7] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_OPFT1);
    gl_bat_critical_error_reg_state[8] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_OPFT2);
    gl_bat_critical_error_reg_state[9] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_SOC);
    gl_bat_critical_error_reg_state[10] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_LVOLT);
    gl_bat_critical_error_reg_state[11] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_HTEMP);
#endif
}

#ifndef CABINET_WARMOR_SUPPLY_BY_CHARGER
unsigned int Battery_get_bat_reg_state_when_critical_error(unsigned char* p)
{
    //���ش洢��ָ��
    memcpy(p, (unsigned char*)gl_bat_critical_error_reg_state, 12*2);
    return (12*2);
}
#endif

// �豸���ϣ�ͬʱ���ֹ�ѹ��Ƿѹ��ͬʱ���ֹ��º͵��£�
//��о��ѹ����2V���������¶ȳ���80���϶�
//�е�ѹ������SOC��0%
//����PMS������⵽�ĵ��Ӳ������(��ʱû�мӽ���)
unsigned char is_battery_critical_error(unsigned char bms_index)
{
    unsigned short vl_value;
    
    vl_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_DEVFT1);
    if(vl_value&(~((1<<7)|(1<<8))))
    {
        return 1;
    }

    vl_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_DEVFT2);    
    if(vl_value)
    {
        return 1;
    }
    #if 0
    vl_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_OPFT1);
    if((vl_value&(1<<0))&&(vl_value&(3<<1)))
    {
        /*                
            Bit0  ��ѹ
            Bit1  ��������
            Bit2  ��������
            */
        //ͬʱ���ֹ�ѹ��Ƿѹ
        return 1;
    }
    if((vl_value&(3<<6))&&(vl_value&(3<<8)))
    {
        /*
                Bit6    ����¶ȹ���
                Bit7    �ŵ��¶ȹ���
                Bit8    ����¶ȹ���
                Bit9    �ŵ��¶ȹ���
                */
        //ͬʱ���ֹ��º͵���
        return 1;
    }

    vl_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_LVOLT);
    //����С��2V
    if((vl_value != 0xFFFF) && (vl_value < 2000))
        return 1;

    vl_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_HTEMP);
    //�¶ȳ���80���϶�
    if((vl_value != 0xFFFF) && (vl_value > (800+400)))
        return 1;

    //soc��0,�����ܵ�ѹ����ͦ�ߵ�(��о��ѹ����3.8V)
    vl_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_SOC);
    if(vl_value == 0)
    {
        vl_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_LVOLT);
        if((vl_value != 0xFFFF) && (vl_value > 3800))
        {
            return 1;
        }
    }
    #endif
    return 0;
}

/*
�Ƚ�����ص�ѹ
return : 0����ʾ����ص�ѹ�Ƚ�С��0.5v
            1����ʾ���1�ĵ�ѹ�Ƚϸ�
            2����ʾ���2�ĵ�ѹ�Ƚϸ�
*/
static unsigned char Batterys_voltage_compare(void)
{
    return 0;
}

#define CHG_IDLE_PORT_INDEX 0x7F
#define CHG_PORT_INDEX_MASK 0x7F
#define CHG_PORT_INDEX_CHARGE_ENABLE_BIT 0x80

#define CHG_PORT_INDEX_MAX 1
typedef struct{
//�ֽڵ�bit7��ʾ�Ƿ������磬0,������1,����
unsigned char chg_port_index[CHG_PORT_INDEX_MAX];
}st_chg_port_index;

#define CHG_GROUP_INDEX_MAX 1
typedef struct{
    st_chg_port_index chg_group_index[CHG_GROUP_INDEX_MAX];
}st_chg_group_param;

st_chg_group_param gl_chg_group_param;

#if (NFC_READER_COUNT_MAX != 1)
    Have to modify "CHG_GROUP_INDEX_MAX" "CHG_PORT_INDEX_MAX" "Battery_chg_param_init()"
#endif

// 10mAΪ��λ
int Battery_get_current(unsigned char bms_index)
{
    unsigned short vl_pos;
    int vl_A;
    vl_pos = MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_TCURR);
    vl_A = ((gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_H<<8) + 
        gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_L);

    vl_A -= (300*100);
    
    return vl_A;
}


unsigned short Battery_get_switch_state(unsigned char bms_index)
{
    //��ŵ綼��ʱ������
    unsigned short vl_pos, vl_ctrl_value;
    vl_pos = MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_STATE);
    vl_ctrl_value = ((gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_H<<8) + 
        gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_L);

    return vl_ctrl_value;
}

void Battery_set_switch_state(unsigned char bms_index, unsigned short on)
{
    unsigned short vl_pos, vl_ctrl_value;

    //������������ڲ�ѯ���ƼĴ��������˳�

    gl_battery_set_switch_flag[bms_index] = 1;
    if(!Battery_is_all_cmd_buf_empty())
        return;

    //�ж�ACC ״̬����ȥ�����״̬; �ж��Ƿ�����ŵ�
    //if ((0 == gl_acc_wake_status)||(0 == Battery_get_PMS_working_state()))
    //if ((sl_wake1_update_flag == 0)&&(0 == Battery_get_PMS_working_state())) //����ʱ��������
    //    return ;
    
    gl_battery_set_switch_flag[bms_index] = 0;
    //���NFC��������û�з������
    if((0 == MM_is_all_snd_done())||
        (gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_INFO1] == 1)||
        (gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_INFO2] == 1))
    {
        return ;
    }
    if((gl_bms_info_p[bms_index]->reg_need_read & ((1<<1)|(1<<2))) != ((1<<1)|(1<<2)))
    {
        if(!(gl_bms_info_p[bms_index]->reg_need_read & (1<<0)))
        {
            gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_INFO1] = 1;
            gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_INFO2] = 1;
        }
        if(!(gl_bms_info_p[bms_index]->reg_need_read & (1<<1)))
            gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_CTRL] = 1;
        return ;
    }
    #if 0
    Battery_info_polling_Process();
    #endif
    
    vl_pos = MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_CTRL);
    vl_ctrl_value = ((gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_H<<8) + 
        gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_L);

    vl_ctrl_value |= on;

    if(ENUM_CHARGE == gl_BP_info.bms_state)
    {
        //���������
        vl_ctrl_value |= (1<<7);
    }
    else
    {
        //������γ�
        vl_ctrl_value &= ~(1<<7);
    }

    //���ͽ�������״̬������
    if(sl_battery_need2sleep_flag)
    {
        // bit5�ǽ���ǳ��˯�ߵı�־
        vl_ctrl_value |= (1<<5);
        //˯��ʱ�򣬳��ر�
        vl_ctrl_value &= ~BMS_STATE_BIT_CHG_SWITCH;

        #if 0 // �������õ�ؽ������˯�ߣ�2018-6-29
        //����ǲ�����ŵ粢����������ACC ON״̬
        if((0 == Battery_get_PMS_working_state())&&(main_get_have_been_ACC_ON_flag()))
        {
            //�������˯��// bit6�ǽ������˯�ߵı�־
            vl_ctrl_value |= (1<<6);
        }
        #else
        //2019-1-24
        #ifdef PMS_CMD_POWEROFF_TIME_COUNTER_ENABLE
        //if(
        //    main_cmd_poweroff_get_poweroff_flag_nonvolatilely()//�Ѿ�ͣ��ִ�й�һ��
        //    )
        //{
            //�������˯��// bit6�ǽ������˯�ߵı�־
        //    vl_ctrl_value |= (1<<6);
        //}  
        #endif
        #endif
    }
    else
    {
        // bit5�ǽ���ǳ��˯�ߵı�־, ����Ҫ˯��ʱ������
        vl_ctrl_value &= (~(1<<5));
    }

    gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_H = (vl_ctrl_value>>8)&0xff;
    gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_L = (vl_ctrl_value>>0)&0xff;

    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_W_CTRL] = 1;

    gl_bms_info_p[bms_index]->reg_need_read &= ~((1<<1)|(1<<2));

    //Battery_cmd_read_all_reg(0);
    Battery_cmd_read_all_reg(bms_index);
    
}

//����Ƿ���Ԥ�Ŵ���Ȼ��������
void Bttery_check_pre_error_and_set_switch_state(unsigned char bms_index, unsigned short on)
{
    unsigned short vl_pos;
    vl_pos = MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_CTRL);
    if(is_battery_presupply_error(0)&&//�Ƿ���Ԥ�ŵ����
        (gl_bms_info_p[0]->reg_unit[vl_pos].bits16_L&0x03))//���ƼĴ����ϵĳ�ŵ�λ��Ϊ��
    {
        //��Ҫ�����ŵ���ƼĴ���
        Battery_clear_switch_state(bms_index,0xFFFF);
        //CmdLine_Printf("pre error, clear ctrl!\r\n");
    }
    else
    {
        Battery_set_switch_state(bms_index, on);
    }
}





//�޸�NFC����������ʱ����Ҫ�ֶ��޸ķ�������
void Battery_chg_param_init(void)
{
    //��ʱ��˳���
    unsigned char i, k;
    for(i = 0; i < CHG_GROUP_INDEX_MAX; i++)
    {
        for(k = 0; k < CHG_PORT_INDEX_MAX; k++)
        {
            gl_chg_group_param.chg_group_index[i].chg_port_index[k] = i+k;
        }
    }
    #ifdef TEST_BAT_V_VIA_CHARGER_EN
    //��ʼ��
    Battery_check_bat_v_via_charger_init();
    //����Ĭ�Ͻ������
    gl_check_bat_v.is_need_bat_v_checking = 2;
    gl_check_bat_v.is_bat_v_valid = 1;
    #endif
    
    #ifdef TEST_DETECT_BAT_ERROR_EN
    memset(&gl_check_bat_err, 0, sizeof(st_check_bat_err));
    #endif

    
    #ifdef EVENT_TO_STOP_CHARGING_ENABLE
    Battery_event_to_stop_chg_param_init();
    #endif

    #ifdef BAT_ERROR_CHECK_DELAY_AFTER_BAT_UPDATE
    sl_bat_error_check_delay_after_bat_update_cnt = 0;
    #endif

    Battery_bat_fault_debounce_param_init();
    #ifdef BAT_TEMP_CHARGER_ENABLE
    Battery_low_temp_param_reset();
    Battery_set_bat_temp_value_reflash_flag();
    #endif

    #ifdef PMS_AUTO_INCREASE_SETTING_VOLTAGE_ENABLE
    Battery_charger_setting_voltage_offset_init();
    #endif

    Battery_history_record_param_init();
}

void Battery_chg_hw_ctrl_init(void)
{
#if 0
    GPIO_PinModeSet(BATTERY_PRECHARGE0_PORT, BATTERY_PRECHARGE0_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(BATTERY_PRECHARGE1_PORT, BATTERY_PRECHARGE1_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(BATTERY_PRECHARGE2_PORT, BATTERY_PRECHARGE2_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(BATTERY_PRECHARGE3_PORT, BATTERY_PRECHARGE3_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(BATTERY_PRECHARGE4_PORT, BATTERY_PRECHARGE4_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(BATTERY_PRECHARGE5_PORT, BATTERY_PRECHARGE5_PIN, gpioModePushPull, 0);

    GPIO_PinModeSet(BATTERY_CHARGE0_PORT, BATTERY_CHARGE0_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(BATTERY_CHARGE1_PORT, BATTERY_CHARGE1_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(BATTERY_CHARGE2_PORT, BATTERY_CHARGE2_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(BATTERY_CHARGE3_PORT, BATTERY_CHARGE3_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(BATTERY_CHARGE4_PORT, BATTERY_CHARGE4_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(BATTERY_CHARGE5_PORT, BATTERY_CHARGE5_PIN, gpioModePushPull, 0);
#else

    #ifdef BATTERY_CHG_ENABLE_PORT
    //��Ϊֻ��һ�����ƽţ���Ϊֻ��һ����ز�
    GPIO_PinModeSet(BATTERY_CHG_ENABLE_PORT, BATTERY_CHG_ENABLE_PIN, gpioModePushPull, 0);
    #endif
#endif
    #if (NFC_READER_COUNT_MAX > 6)
    Have to add control pins for Battery Charge.
    #endif
}

//Ԥ����
static void Battery_prechg_hw_ctrl_on(unsigned char bms_index)
{
    #if (NFC_READER_COUNT_MAX > 6)
    Have to add control pins for Battery Charge, too.
    #endif

    #if 0
    switch(bms_index)
    {
        case 0:
            GPIO_PinOutSet(BATTERY_PRECHARGE0_PORT, BATTERY_PRECHARGE0_PIN);
            GPIO_PinOutClear(BATTERY_CHARGE0_PORT, BATTERY_CHARGE0_PIN);
            break;
        case 1:
            GPIO_PinOutSet(BATTERY_PRECHARGE1_PORT, BATTERY_PRECHARGE1_PIN);
            GPIO_PinOutClear(BATTERY_CHARGE1_PORT, BATTERY_CHARGE1_PIN);
            break;
        case 2:
            GPIO_PinOutSet(BATTERY_PRECHARGE2_PORT, BATTERY_PRECHARGE2_PIN);
            GPIO_PinOutClear(BATTERY_CHARGE2_PORT, BATTERY_CHARGE2_PIN);
            break;
        case 3:
            GPIO_PinOutSet(BATTERY_PRECHARGE3_PORT, BATTERY_PRECHARGE3_PIN);
            GPIO_PinOutClear(BATTERY_CHARGE3_PORT, BATTERY_CHARGE3_PIN);
            break;
        case 4:
            GPIO_PinOutSet(BATTERY_PRECHARGE4_PORT, BATTERY_PRECHARGE4_PIN);
            GPIO_PinOutClear(BATTERY_CHARGE4_PORT, BATTERY_CHARGE4_PIN);
            break;
        case 5:
            GPIO_PinOutSet(BATTERY_PRECHARGE5_PORT, BATTERY_PRECHARGE5_PIN);
            GPIO_PinOutClear(BATTERY_CHARGE5_PORT, BATTERY_CHARGE5_PIN);
            break;
        default:
            break;
    }
    #else
    //�����Ԥ������ɳ�������յ����������

    #endif
}

//����
static void Battery_chg_hw_ctrl_on(unsigned char bms_index)
{
    #if (NFC_READER_COUNT_MAX > 6)
    Have to add control pins for Battery Charge, too.
    #endif
    
    switch(bms_index)
    {
        case 0:
            #if 0
            GPIO_PinOutClear(BATTERY_PRECHARGE0_PORT, BATTERY_PRECHARGE0_PIN);
            GPIO_PinOutSet(BATTERY_CHARGE0_PORT, BATTERY_CHARGE0_PIN);
            #else
            #ifdef BATTERY_CHG_ENABLE_PORT
            //�򿪳���·���������Ҫ���ϳ���������߼�
            GPIO_PinOutSet(BATTERY_CHG_ENABLE_PORT, BATTERY_CHG_ENABLE_PIN);
            #endif
            #endif
            break;
        #if 0
        case 1:
            GPIO_PinOutClear(BATTERY_PRECHARGE1_PORT, BATTERY_PRECHARGE1_PIN);
            GPIO_PinOutSet(BATTERY_CHARGE1_PORT, BATTERY_CHARGE1_PIN);
            break;
        case 2:
            GPIO_PinOutClear(BATTERY_PRECHARGE2_PORT, BATTERY_PRECHARGE2_PIN);
            GPIO_PinOutSet(BATTERY_CHARGE2_PORT, BATTERY_CHARGE2_PIN);
            break;
        case 3:
            GPIO_PinOutClear(BATTERY_PRECHARGE3_PORT, BATTERY_PRECHARGE3_PIN);
            GPIO_PinOutSet(BATTERY_CHARGE3_PORT, BATTERY_CHARGE3_PIN);
            break;
        case 4:
            GPIO_PinOutClear(BATTERY_PRECHARGE4_PORT, BATTERY_PRECHARGE4_PIN);
            GPIO_PinOutSet(BATTERY_CHARGE4_PORT, BATTERY_CHARGE4_PIN);
            break;
        case 5:
            GPIO_PinOutClear(BATTERY_PRECHARGE5_PORT, BATTERY_PRECHARGE5_PIN);
            GPIO_PinOutSet(BATTERY_CHARGE5_PORT, BATTERY_CHARGE5_PIN);
            break;
        #endif
        default:
            break;
    }
}

//���ر�
static void Battery_chg_hw_ctrl_off(unsigned char bms_index)
{
    #if (NFC_READER_COUNT_MAX > 6)
    Have to add control pins for Battery Charge, too.
    #endif
    
    switch(bms_index)
    {
        case 0:
            #if 0
            GPIO_PinOutClear(BATTERY_PRECHARGE0_PORT, BATTERY_PRECHARGE0_PIN);
            GPIO_PinOutClear(BATTERY_CHARGE0_PORT, BATTERY_CHARGE0_PIN);
            #else
            #ifdef BATTERY_CHG_ENABLE_PORT
            //�رճ���·�����������Ҫ���Ϲس�������߼�
            GPIO_PinOutClear(BATTERY_CHG_ENABLE_PORT, BATTERY_CHG_ENABLE_PIN);
            #endif
            #endif
            break;
        #if 0    
        case 1:
            GPIO_PinOutClear(BATTERY_PRECHARGE1_PORT, BATTERY_PRECHARGE1_PIN);
            GPIO_PinOutClear(BATTERY_CHARGE1_PORT, BATTERY_CHARGE1_PIN);
            break;
        case 2:
            GPIO_PinOutClear(BATTERY_PRECHARGE2_PORT, BATTERY_PRECHARGE2_PIN);
            GPIO_PinOutClear(BATTERY_CHARGE2_PORT, BATTERY_CHARGE2_PIN);
            break;
        case 3:
            GPIO_PinOutClear(BATTERY_PRECHARGE3_PORT, BATTERY_PRECHARGE3_PIN);
            GPIO_PinOutClear(BATTERY_CHARGE3_PORT, BATTERY_CHARGE3_PIN);
            break;
        case 4:
            GPIO_PinOutClear(BATTERY_PRECHARGE4_PORT, BATTERY_PRECHARGE4_PIN);
            GPIO_PinOutClear(BATTERY_CHARGE4_PORT, BATTERY_CHARGE4_PIN);
            break;
        case 5:
            GPIO_PinOutClear(BATTERY_PRECHARGE5_PORT, BATTERY_PRECHARGE5_PIN);
            GPIO_PinOutClear(BATTERY_CHARGE5_PORT, BATTERY_CHARGE5_PIN);
            break;
        #endif
        default:
            break;
    }
}

//�ҳ����ڳ��ĵ�أ����жϵ�ѹ��Ȼ��ѡ����ģʽ
static void Battery_chg_hw_ctrl_process(void)
{
    unsigned char i;
    unsigned short vl_bms_tmp_V = 0;
    unsigned char vl_group_index;

    for(i = 0; i < NFC_READER_COUNT_MAX; i++)
    {
        //�ҳ����ߵĵ��
        //if(is_battery_voltage_OK(i))
        if(
            (ADC_is_CHG_enable())&&
            (master_rs485_is_charger_OK())&&
            ((BMS_READONLY_DATA1_FLAG_BIT|BMS_READONLY_DATA2_FLAG_BIT)&
            Battery_get_reg_valid_flag(i))&&
            (Battery_chg_is_charge_enable(i))
            &&
            (ELOCK_LOCK == ELock_get_status((ENUM_ELOCK_INDEX)i))//������Ǵ򿪵ģ��򲻳��
            )
        {
            //vl_bms_tmp_V = Battery_get_voltage(i);

            vl_group_index = Battery_group_get_group_index_by_bms_index(i);
            if(vl_group_index == 0xFF)
                continue;
            //�����������ߵ�ѹֵ����Ϊ���ʱ���ȳ���ߵ�ѹ�ĵ��
            vl_bms_tmp_V = Battery_group_get_highest_voltage(vl_group_index);

            if(vl_bms_tmp_V == 0)
            {
                //��Ч�ĵ�ѹ
                //���ر�
                //Battery_chg_hw_ctrl_off(i);
                continue;
            }
            #if 0
            if(vl_bms_tmp_V < 4800)
            {
                //С��48v, �ȿ�Ԥ��
                Battery_prechg_hw_ctrl_on(i);
            }
            else
            {
                //��ʽ����
                Battery_chg_hw_ctrl_on(i);
            }
            #else
            //��ʽ���򿪣���Ϊû��Ԥ���Ӳ������(���ڳ�����й���)��ֱ�ӳ��
            Battery_chg_hw_ctrl_on(i);
            #endif
        }
        else
        {
            //���ر�
            Battery_chg_hw_ctrl_off(i);
            
        }

        if(((BMS_READONLY_DATA1_FLAG_BIT|BMS_READONLY_DATA2_FLAG_BIT)&
            Battery_get_reg_valid_flag(i))
            &&
            (ELOCK_LOCK == ELock_get_status((ENUM_ELOCK_INDEX)i))
            )
        {
            
        }
        else
        {
            //���رգ��ȴ���λ�����·��������������
            Battery_chg_set_charge_disable(i);
        }
    }
}

void Battery_chg_set_charge_enable(unsigned char bms_index)
{
    #if 0
    unsigned char i, k;
    for(i = 0; i < CHG_GROUP_INDEX_MAX; i++)
    {
        for(k = 0; k < CHG_PORT_INDEX_MAX; k++)
        {
            if(bms_index == (CHG_PORT_INDEX_MASK & gl_chg_group_param.chg_group_index[i].chg_port_index[k]))
            {
                //���ж��Ƿ�����
                if(gl_bms_info_p[bms_index]->online)
                {
                    gl_chg_group_param.chg_group_index[i].chg_port_index[k] |= CHG_PORT_INDEX_CHARGE_ENABLE_BIT;
                }
                return;
            }
        }
    }
    #else
    gl_pms_param.OD_pms_cmd |= OD_PMS_CMD_MASK_CHARGE_EN;
    #endif
}

void Battery_chg_set_charge_disable(unsigned char bms_index)
{
    #if 0
    unsigned char i, k;
    for(i = 0; i < CHG_GROUP_INDEX_MAX; i++)
    {
        for(k = 0; k < CHG_PORT_INDEX_MAX; k++)
        {
            if(bms_index == (CHG_PORT_INDEX_MASK & gl_chg_group_param.chg_group_index[i].chg_port_index[k]))
            {
                gl_chg_group_param.chg_group_index[i].chg_port_index[k] &= (~(CHG_PORT_INDEX_CHARGE_ENABLE_BIT));
                return;
            }
        }
    }
    #else
    gl_pms_param.OD_pms_cmd &= (~OD_PMS_CMD_MASK_CHARGE_EN);
    #endif
}

//�Ƿ�������
bool Battery_chg_is_charge_enable(unsigned char bms_index)
{
    unsigned char i, k;
    for(i = 0; i < CHG_GROUP_INDEX_MAX; i++)
    {
        for(k = 0; k < CHG_PORT_INDEX_MAX; k++)
        {
            if(bms_index == (CHG_PORT_INDEX_MASK & gl_chg_group_param.chg_group_index[i].chg_port_index[k]))
            {
                if(gl_chg_group_param.chg_group_index[i].chg_port_index[k]&CHG_PORT_INDEX_CHARGE_ENABLE_BIT)
                    return TRUE;
                else
                    return FALSE;
            }
        }
    }

    return FALSE;
}

//�������������ĸ����
static unsigned char Battery_group_get_group_index_by_bms_index(unsigned char bms_index)
{
    unsigned char k, i;
    unsigned char vl_bms_index;
    for(k = 0; k < CHG_GROUP_INDEX_MAX; k++)
    {
        for(i = 0; i < CHG_PORT_INDEX_MAX; i++)
        {
            vl_bms_index = CHG_PORT_INDEX_MASK & gl_chg_group_param.chg_group_index[k].chg_port_index[i];
            if(vl_bms_index == bms_index)
                return k;
        }
    }
    //û���ҵ�
    return 0xFF; 
}

static unsigned short Battery_group_get_highest_voltage(unsigned char group_index)
{
    unsigned char vl_bms_index;
    unsigned short vl_bms_V = 0;
    unsigned short vl_bms_tmp_V = 0;
    unsigned char i;
    for(i = 0; i < CHG_PORT_INDEX_MAX; i++)
    {
        vl_bms_index = CHG_PORT_INDEX_MASK & gl_chg_group_param.chg_group_index[group_index].chg_port_index[i];

        //�Ƿ���Ч��
        if(CHG_IDLE_PORT_INDEX == vl_bms_index)
            continue;

        //�ҳ����ߵĵ��
        if(is_battery_voltage_OK(vl_bms_index))
        {
            vl_bms_tmp_V = Battery_get_voltage(vl_bms_index);
            //�ҳ���ߵĵ�ѹ
            if(vl_bms_tmp_V > vl_bms_V)
            {
                vl_bms_V = vl_bms_tmp_V;
            }
        }
    }

    return vl_bms_V;
}

static void Battery_group_charge_process(unsigned char group_index)
{
    unsigned char vl_bms_index;
    unsigned short vl_bms_V = 0;
    unsigned short vl_bms_tmp_V = 0;
    unsigned short vl_bms_ctrl_status;
    unsigned char i;
    //��������ȫ�ּ�ʱ�������������Ҫ��BAT���ؾ�reloadһ�Ρ�
    static unsigned int gl_bat_switch_close_from_open_state_cnt_old;
    unsigned int vl_tmp_int_value;
    static unsigned char sl_elock_status_old = 0; //0����ʾ�򿪣�1��ʾ�ر�
    
    //�����ϵ��ӳٴ�����������
    if(gl_Battery_delay_process_cnt>2500)
        return;

    //���NFC���ڷ�������򲻴�����������
    for(i = 0; i < CHG_PORT_INDEX_MAX; i++)
    {
        vl_bms_index = CHG_PORT_INDEX_MASK & gl_chg_group_param.chg_group_index[group_index].chg_port_index[i];

        //�Ƿ���Ч��
        if(CHG_IDLE_PORT_INDEX == vl_bms_index)
            continue;
        
        if(gl_bms_info_p[vl_bms_index]->waiting_cmd_ack)
            return ;
    }


    vl_bms_V = Battery_group_get_highest_voltage(group_index);
    for(i = 0; i < CHG_PORT_INDEX_MAX; i++)
    {
        vl_bms_index = CHG_PORT_INDEX_MASK & gl_chg_group_param.chg_group_index[group_index].chg_port_index[i];


        //�Ƿ���Ч��
        if(CHG_IDLE_PORT_INDEX == vl_bms_index)
            continue;

        //�����ɿ����ʱ�򣬴�������Ĳ���
        {
            static unsigned char sl_elock_status_old_for_checking = 1; //0����ʾ�򿪣�1��ʾ�ر�
            if((sl_elock_status_old_for_checking != 0)&&
                (ELOCK_OPEN== ELock_get_status((ENUM_ELOCK_INDEX)vl_bms_index)))
            {
                sl_elock_status_old_for_checking = 0;
                memset(&gl_bat_theft_prevention_param, 0, sizeof(st_bat_theft_prevention_param));
            }
            else if(!sl_elock_status_old_for_checking)//�����Ÿչر�ʱ��
            {
                sl_elock_status_old_for_checking = 1;

                #if 0 // �����ж���û�е�ص���������θù��ܡ�2019-7-23
                //������Ƿ����ߣ���������ߣ����ټ��������������״̬��
                //Ĭ���ϴ�������û�����Ӻõ�״̬��
                if((0 == gl_bat_theft_prevention_param.connector_default_state)&&
                    (0 == is_battery_voltage_OK(vl_bms_index)))
                {
                    gl_bat_theft_prevention_param.connector_default_state = 1;
                }
                #endif
                //���΢�������Ƿ�λ������Ѿ���λ�����ټ��΢�������Ƿ�λ��
                //Ĭ���ϴ�΢������û�е�λ��״̬��
                #ifdef CHECKING_BAT_IN_IO_FLAG_ENABLE
                if((0 == gl_bat_theft_prevention_param.switch_default_state) && 
                    (TRUE == Battery_is_checking_mode_on_bat_in_by_HW()) && 
                    (TRUE == Battery_is_bat_in_via_IO_switch()))
                {
                    gl_bat_theft_prevention_param.switch_default_state = 1;
                }
                #endif
            }
        }

        //���Ŵ򿪵�ʱ�����step timeout��ʱ���������Զ�����PMS
        if(gl_check_bat_err.bat_step_timeout_err_cnt && 
            (ELOCK_OPEN== ELock_get_status((ENUM_ELOCK_INDEX)vl_bms_index)))
        {
            gl_check_bat_err.bat_step_timeout_err_cnt = 0;
        }

        //�ҳ����ߵĵ��
        if(is_battery_voltage_OK(vl_bms_index))
        {
            unsigned short vl_bms_reg_value;

            //���Ź������ڼ��΢�������ڼ��ػ�����ⲻ��������
            //����м���ֵ������źŻ���ż���������źţ�
            //���ټ��������������״̬��Ĭ���ϴ�������û�����Ӻõ�״̬��
            if((0 == gl_bat_theft_prevention_param.connector_default_state) &&
                (ELOCK_LOCK == ELock_get_status((ENUM_ELOCK_INDEX)vl_bms_index))
                #ifdef UNMASK_CODE
                &&(SM_is_closing())
                #endif
                &&(!((1<<3) & (Battery_get_switch_state(vl_bms_index)))))
            {
                //Ĭ��û�����Ӻ�������
                gl_bat_theft_prevention_param.connector_default_state = 1;
            }

            #ifdef TEST_BAT_V_VIA_CHARGER_EN
            if(
                (ELOCK_LOCK== ELock_get_status((ENUM_ELOCK_INDEX)vl_bms_index)) 
                #ifdef CHECKING_BAT_IN_IO_FLAG_ENABLE
                // �����΢�����أ���Ҫ��⵽΢�����ص�λ�����������������Ƿ��õļ��
                &&(((TRUE == Battery_is_checking_mode_on_bat_in_by_HW()) && (TRUE == Battery_is_bat_in_via_IO_switch()))
                    ||(FALSE == Battery_is_checking_mode_on_bat_in_by_HW()))
                #endif
                && (Battery_check_bat_v_process(vl_bms_index))
                )
            {
                //���ڽ��йز��ż���ع���
                continue;
            }
            #endif
            
            #ifdef TEST_DETECT_BAT_ERROR_EN
            if((ELOCK_LOCK == ELock_get_status((ENUM_ELOCK_INDEX)vl_bms_index)) &&
                (slave_rs485_is_detect_bat_error_enable())
                )
            {
                if(Battery_check_bat_process())
                {
                    //���ڴ����ع���
                    continue;
                }
            }
            else if(FALSE == slave_rs485_is_detect_bat_error_enable())
            {
                //������������⡣����ռ���״̬
                Battery_check_bat_clear_on_and_error();
            }
            #endif
            
            //�����������ж��Ƿ���Ҫ�����ѯ
            #if 0
            if((sl_bat_poll_param.poll_allways_en_delay_cnt == 0)&&
                (is_battery_error(vl_bms_index, BATTERY_ERROR_OPTION_IGNORE_NONE)|| // ����в����ٴ򿪳�ŵ�ܵĴ���
                (FALSE == Battery_is_charge_enable_by_BatState(vl_bms_index))|| // ��ز��ó��
                (FALSE == master_rs485_is_charger_OK())|| // ���������
                (FALSE == Battery_chg_is_charge_enable(vl_bms_index))//��λ�������ֹ���ʱ��
                )
                )
            #else
            // 2019-7-22 
            if((sl_bat_poll_param.poll_allways_en_delay_cnt == 0)&&
                (!((Battery_chg_is_charge_enable(vl_bms_index))
                    &&(ADC_is_CHG_enable())
                    &&(master_rs485_is_charger_OK())
                    &&(0 == is_battery_error(vl_bms_index))
                    #ifdef EVENT_TO_STOP_CHARGING_ENABLE
                    &&(!Battery_get_event_to_stop_chg_flag())
                    #endif
                    #ifdef TEST_DETECT_BAT_ERROR_EN
                    &&(!Battery_check_bat_is_bat_error())
                    #endif
                    #ifdef BAT_TEMP_CHARGER_ENABLE
                    &&(Battery_is_bat_chg_enable_by_temp())
                    #endif
                    #ifdef BAT_PMS_AUTHORITY_FUNCTION_EN
                    &&((Is_Authority_done_about_bat_pms()) 
                        #ifdef AUTHORITY_PMS_BAT_FUNC_EXE_ENABLE
                        &&((0 == Authority_get_onoff_about_pms_bat()) ||
                        (Is_Authority_done_about_pms_bat()))
                        #endif
                    )
                    #endif
                    &&(0 != Battery_get_max_chg_current(vl_bms_index)) // 2019-3-30 ������������Ϊ0ʱ�򣬹رյ�س�ŵ�ܡ�
                    //&&(0 == (BMS_STATE_BIT_FULLY_CHARGE & vl_bms_ctrl_status))//��صĳ�����־û�б���λ��2018-8-30  // Bit14   1--�������־(���״̬��Ч)
                    &&(1000 > Battery_get_reg_value(vl_bms_index, ENUM_REG_ADDR_SOC)) // SOCС��100%���ܳ��
                    #ifdef CABINET_WARMOR_SUPPLY_BY_CHARGER
                    &&(0 == master_rs485_get_charger_warmer_fsm_state()) //���ڳ��ģʽ
                    #endif
                )))
            #endif
            {
                sl_bat_poll_param.poll_interval_MAX = BAT_POLL_INTERVAL_MAX;
                //��ȡ��������͵�ѹ
                vl_bms_reg_value = Battery_get_reg_value(vl_bms_index, ENUM_REG_ADDR_LVOLT);

                // ��Ъ����ѯ
                sl_bat_poll_param.is_poll_allways_en_flag = 0;
                //�Ƿ��е����ص�ѹ���ͣ�С��2.7V
                if(0xFFFF == vl_bms_reg_value)
                {
                    sl_bat_poll_param.poll_interval_MAX = BAT_POLL_BIG_INTERVAL_MAX;
                }
                else if(vl_bms_reg_value < 2900)
                {
                    sl_bat_poll_param.poll_interval_MAX = BAT_POLL_BIG_BIG_INTERVAL_MAX;
                }
                else if(vl_bms_reg_value < 3000)
                {
                    sl_bat_poll_param.poll_interval_MAX = BAT_POLL_BIG_INTERVAL_MAX;
                }
                else if(vl_bms_reg_value > 3900) // ��о��ѹ����3.9V�󣬱���һֱ��ѯ
                {
                    //������ѯ
                    sl_bat_poll_param.is_poll_allways_en_flag = 1;
                    sl_bat_poll_param.poll_interval_MAX = 0;
                }
            }
            else
            {
                //������ѯ
                sl_bat_poll_param.is_poll_allways_en_flag = 1;
                sl_bat_poll_param.poll_interval_MAX = 0;
            }
            
        }
        else
        {
            #ifdef TEST_DETECT_BAT_ERROR_EN
            if(FALSE == Battery_check_bat_is_end())
            {
                Battery_check_bat_clear_on_and_error();
            }
            #endif

            //fault�����ӳ��ϱ����ܣ��������
            Battery_bat_fault_debounce_param_init();
        }

        #ifdef BAT_WARMOR_BY_SELF_ENABLE
        //����Լ��Ȳ��Բ��򿪼����ѯ��
        //������ѯ
        sl_bat_poll_param.is_poll_allways_en_flag = 1;
        sl_bat_poll_param.poll_interval_MAX = 0;
        #endif

        if(ELOCK_OPEN== ELock_get_status((ENUM_ELOCK_INDEX)vl_bms_index))
        {
            sl_elock_status_old = 0;
            //������ѯ
            sl_bat_poll_param.is_poll_allways_en_flag = 1;
            sl_bat_poll_param.poll_interval_MAX = 0;
            
            #ifdef TEST_DETECT_BAT_ERROR_EN
            Battery_check_bat_clear_on_and_error();
            #endif

            #ifdef TEST_BAT_V_VIA_CHARGER_EN
            Battery_check_bat_v_via_charger_init();
            #endif

            #ifdef EVENT_TO_STOP_CHARGING_ENABLE
            //������¼���ֹͣ���ı�־��
            Battery_clear_event_to_stop_chg_flag();
            #endif

            #ifdef PMS_AUTO_INCREASE_SETTING_VOLTAGE_ENABLE
            //�����ţ������ѹƫ����
            Battery_clear_charger_setting_voltage_offset();
            #endif

            //����������ϵı�־
            Battery_bat_chg_current_false_flag_clear();

            //fault�����ӳ��ϱ����ܣ��������
            Battery_bat_fault_debounce_param_init();

            //������������߶�·���ϱ������������ܣ��������
            master_rs485_charger_error_cycle_lock_init();

            master_rs485_set_chg_voltage_max_value_default();
        }
        else if(!sl_elock_status_old)//�����Ÿչر�ʱ��
        {
            sl_elock_status_old = 1;

            #ifdef BAT_TEMP_CHARGER_ENABLE
            //���ǣ��ڶ�ȡ����¶���������"����ʱ����¶�ֵ"
            //Battery_set_bat_temp_value_when_plus_in();
            Battery_set_bat_temp_value_reflash_flag();
            Battery_low_temp_param_reset();
            #endif
        }
        #ifdef BAT_TEMP_CHARGER_ENABLE
        else 
        {
            Battery_bat_low_temp_process();
        }
        #endif

        if(FALSE == Battery_chg_is_charge_enable(vl_bms_index))
        {
            //�������ϵļ�ʱ�����㡣
            Battery_bat_chg_current_false_timer_reset();
        }
        
        //�ҳ����ߵĵ��
        if((is_battery_voltage_OK(vl_bms_index))
            //&&(0 == is_battery_error(vl_bms_index, BATTERY_ERROR_OPTION_IGNORE_NONE))
            )
        {

            vl_bms_ctrl_status = Battery_get_switch_state(vl_bms_index);
            //�ҳ���Ҫ���ĵ�غͲ���Ҫ���ĵ��(0.5V��ѹ�Ƚ�)
            vl_bms_tmp_V = Battery_get_voltage(vl_bms_index);

            if((vl_bms_V < vl_bms_tmp_V))
                return ;//�쳣������˳�
            
            if((vl_bms_V - vl_bms_tmp_V) >= 50)
            {
                //�رղ���Ҫ���ĵ��
                if((BMS_CTRL_BIT_CHG_SWITCH| BMS_CTRL_BIT_SUPPLY_SWITCH)&vl_bms_ctrl_status)
                {
                    Battery_clear_switch_state(vl_bms_index, BMS_CTRL_BIT_CHG_SWITCH| BMS_CTRL_BIT_SUPPLY_SWITCH);
                }
            }
            else
            {
                //����Ҫ���ĵ��
                if((Battery_chg_is_charge_enable(vl_bms_index))
                    &&(ADC_is_CHG_enable())
                    &&(master_rs485_is_charger_OK())
                    &&(ELOCK_LOCK == ELock_get_status((ENUM_ELOCK_INDEX)vl_bms_index))
                    &&(0 == is_battery_error(vl_bms_index))
                    #ifdef EVENT_TO_STOP_CHARGING_ENABLE
                    &&(!Battery_get_event_to_stop_chg_flag())
                    #endif
                    #ifdef TEST_DETECT_BAT_ERROR_EN
                    &&(!Battery_check_bat_is_bat_error())
                    #endif
                    #ifdef BAT_TEMP_CHARGER_ENABLE
                    &&(Battery_is_bat_chg_enable_by_temp())
                    #endif
                    #ifdef BAT_PMS_AUTHORITY_FUNCTION_EN
                    &&((Is_Authority_done_about_bat_pms()) 
                        #ifdef AUTHORITY_PMS_BAT_FUNC_EXE_ENABLE
                        && 
                        ((0 == Authority_get_onoff_about_pms_bat()) ||
                        (Is_Authority_done_about_pms_bat()))
                        #endif
                        )
                    #endif
                    &&(0 != Battery_get_max_chg_current(vl_bms_index)) // 2019-3-30 ������������Ϊ0ʱ�򣬹رյ�س�ŵ�ܡ�
                    //&&(0 == (BMS_STATE_BIT_FULLY_CHARGE & vl_bms_ctrl_status))//��صĳ�����־û�б���λ��2018-8-30  // Bit14   1--�������־(���״̬��Ч)
                    &&(1000 > Battery_get_reg_value(vl_bms_index, ENUM_REG_ADDR_SOC)) // SOCС��100%���ܳ��
                    #ifdef CABINET_WARMOR_SUPPLY_BY_CHARGER
                    &&(0 == master_rs485_get_charger_warmer_fsm_state()) //���ڳ��ģʽ
                    #endif
                )
                {
                    if(((BMS_CTRL_BIT_CHG_SWITCH| BMS_CTRL_BIT_SUPPLY_SWITCH) & vl_bms_ctrl_status) != 
                        (BMS_CTRL_BIT_CHG_SWITCH| BMS_CTRL_BIT_SUPPLY_SWITCH))
                    {
                        Battery_set_switch_state(vl_bms_index, BMS_CTRL_BIT_CHG_SWITCH| BMS_CTRL_BIT_SUPPLY_SWITCH);
                    }
                    gl_bat_switch_close_from_open_state_cnt_old = gl_delay_cnt_test;
                }
                else
                {
                    //�������磬��ŵ�ر�
                    if(((BMS_CTRL_BIT_PRESUPPLY_SWITCH | BMS_CTRL_BIT_CHG_SWITCH | BMS_CTRL_BIT_SUPPLY_SWITCH) 
                        & vl_bms_ctrl_status)
                        #ifdef TEST_BAT_V_VIA_CHARGER_EN
                        &&(!Battery_check_bat_v_is_on())
                        #endif
                        )
                    {
                        
                        //�ȹرճ����ID���ܹرյ�ؿ���
                        if(FALSE == main_is_charger_id_enable())
                        {
                            Battery_clear_switch_state(vl_bms_index, BMS_CTRL_BIT_CHG_SWITCH| BMS_CTRL_BIT_SUPPLY_SWITCH|BMS_CTRL_BIT_PRESUPPLY_SWITCH);
                        }
                        else 
                        {
                            vl_tmp_int_value = gl_delay_cnt_test - gl_bat_switch_close_from_open_state_cnt_old;
                            if(vl_tmp_int_value >= 4000) // 4��
                            {
                                Battery_clear_switch_state(vl_bms_index, BMS_CTRL_BIT_CHG_SWITCH| BMS_CTRL_BIT_SUPPLY_SWITCH|BMS_CTRL_BIT_PRESUPPLY_SWITCH);
                            }
                        }
                    }
                }
            }
        }
        
        
    }

}


void Battery_charge_process(void)
{
    unsigned char i;
    for(i = 0; i < CHG_GROUP_INDEX_MAX;i++)
    {
        Battery_group_charge_process(i);
    }
}

unsigned short Battery_get_reg_value(unsigned char bms_index, unsigned short reg_index)
{
    unsigned short vl_pos;
    vl_pos = MM_get_reg_addr_position_in_table(reg_index);

    vl_pos = ((gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_H<<8) + 
        gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_L);

    return vl_pos;
}

bool Battery_get_bat_desc(unsigned char bms_index, st_version_desc * version_desc)
{
    unsigned short vl_tmp;
    if(gl_bms_app_bl_status[bms_index].is_bl_mode)
    {
        //����bootloaderģʽ
        memset((unsigned char*)version_desc, 0, sizeof(st_version_desc));
        version_desc->HW_main_ver = gl_bms_app_bl_status[bms_index].hw_main_version;//Ӳ�����汾
        version_desc->HW_sub_ver = gl_bms_app_bl_status[bms_index].hw_sub_version;//Ӳ���ΰ汾
    }
    else
    {
        vl_tmp = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_HWVER);

        version_desc->HW_main_ver = (vl_tmp>>8)&0xFF;//Ӳ�����汾
        version_desc->HW_sub_ver = vl_tmp&0xFF;//Ӳ���ΰ汾

        vl_tmp = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_FWMSV);
        version_desc->FW_main_ver = (vl_tmp>>8)&0xFF; //�̼����汾
        version_desc->FW_sub_ver = vl_tmp&0xFF; //�̼��ΰ汾

        vl_tmp = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_FWREV);
        version_desc->FW_revised_ver = vl_tmp&0xFF; //�̼������汾

        vl_tmp = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_FWBNH);
        version_desc->FW_build_ver[3] = (vl_tmp>>8)&0xFF;
        version_desc->FW_build_ver[2] = vl_tmp&0xFF;
        
        vl_tmp = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_FWBNL);
        version_desc->FW_build_ver[1] = (vl_tmp>>8)&0xFF;
        version_desc->FW_build_ver[0] = vl_tmp&0xFF;
    }
    return 1;
}

//��ȡ��ʷ��¼�������¼��
unsigned int Battery_get_oldest_record_index_number(void)
{
    unsigned short vl_H;
    unsigned short vl_L;
    vl_H = Battery_get_reg_value(0, ENUM_REG_ADDR_ERECH);
    vl_L = Battery_get_reg_value(0, ENUM_REG_ADDR_ERECL);
    return ((vl_H << 16) + vl_L);
}

//��ȡ��ʷ��¼�������¼��
unsigned int Battery_get_newest_record_index_number(void)
{
    unsigned short vl_H;
    unsigned short vl_L;
    vl_H = Battery_get_reg_value(0, ENUM_REG_ADDR_LRECH);
    vl_L = Battery_get_reg_value(0, ENUM_REG_ADDR_LRECL);
    return ((vl_H << 16) + vl_L);
}


void Battery_get_bat_ID(unsigned char bms_index, unsigned char bat_ID[6])
{
    st_modbus_reg_unit vl_unit_tmp;
    vl_unit_tmp = gl_bms_info_p[bms_index]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_SN34)];
    bat_ID[0] = vl_unit_tmp.bits16_H;
    bat_ID[1] = vl_unit_tmp.bits16_L;
    
    vl_unit_tmp = gl_bms_info_p[bms_index]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_SN56)];
    bat_ID[2] = vl_unit_tmp.bits16_H;
    bat_ID[3] = vl_unit_tmp.bits16_L;
    
    vl_unit_tmp = gl_bms_info_p[bms_index]->reg_unit[MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_SN78)];
    bat_ID[4] = vl_unit_tmp.bits16_H;
    bat_ID[5] = vl_unit_tmp.bits16_L;
}
//����ֵ�ľ�����0.1V
unsigned short Battery_get_max_chg_voltage(unsigned char bms_index)
{
    unsigned short vl_pos;
    vl_pos = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_MAXCHGV);

    if(vl_pos == 0xFFFF)
        vl_pos = 0;
    else
        vl_pos /= 10;
    
    return vl_pos;
}

//����ֵ�ľ�����0.1A
unsigned short Battery_get_max_chg_current(unsigned char bms_index)
{
    unsigned short vl_pos;
    vl_pos = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_CSOP);

    if(vl_pos == 0xFFFF)
        vl_pos = 0;
    else
        vl_pos /= 10;
    
    return vl_pos;
}

//����ֵû��ƫ�ƣ�����С��0���϶ȵĻ�ͳһ����0; ��λ1 ��C
unsigned short Battery_get_bat_temp(unsigned char bms_index)
{
    unsigned short vl_pos;
    vl_pos = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_HTEMP);
    if((vl_pos == 0xFFFF)||(vl_pos < 400))
    {
        vl_pos = 0;
    }
    else
    {
        vl_pos /= 10;
        vl_pos -= 40;
    }
    return vl_pos;
}

//����ֵ0.1C/bitΪ��λ
short Battery_get_bat_lowest_temp(unsigned char bms_index)
{
    #ifdef TEST_TEST_CONDITION_CREATE_ENABLE
    extern unsigned char gl_cmd_test_test[2];
    unsigned short vl_ret;
    vl_ret = (((gl_cmd_test_test[1]<<8)|(gl_cmd_test_test[0])) - 400);
    return vl_ret;
    #else
    unsigned short vl_value;
    short vl_ret;
    vl_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_LTEMP);
    if(vl_value == 0xFFFF)
    {
        vl_ret = 0;
    }
    else
    {
        vl_ret = vl_value - 400;
    }
    return vl_ret;
    #endif
}


//����ֵ��1mA/hΪ��λ
unsigned short Battery_get_bat_rated_cap(unsigned char bms_index)
{
    unsigned short vl_pos;
    vl_pos = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_BCAP);
    if(vl_pos == 0xFFFF)
        vl_pos = 0;
    
    return vl_pos;
}

//����ֵ��0.1VΪ��λ
unsigned short Battery_get_bat_rated_volt(unsigned char bms_index)
{
    unsigned short vl_pos;
    vl_pos = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_BVOLT);

    if(vl_pos == 0xFFFF)
        vl_pos = 0;
    else
    {
        vl_pos /= 10;
    }
    return vl_pos;
}


unsigned short Battery_get_voltage(unsigned char bms_index)
{
    unsigned short vl_pos;
    //vl_pos = MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_TVOLT);

    //vl_pos = ((gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_H<<8) + 
    //    gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_L);

    vl_pos = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_TVOLT);

    if(vl_pos == 0xFFFF)
        vl_pos = 0;
    
    return vl_pos;
}

unsigned short Battery_get_TCurrent(unsigned char bms_index)
{
    unsigned short vl_pos;
    //vl_pos = MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_TVOLT);

    //vl_pos = ((gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_H<<8) + 
    //    gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_L);

    vl_pos = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_TCURR);

    if(vl_pos == 0xFFFF)
        vl_pos = 0;
    
    return vl_pos;
}

//return : 0.1%Ϊ��λ
unsigned short Battery_get_SOC(unsigned char bms_index)
{
    unsigned short vl_pos;
    vl_pos = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_SOC);

    if(vl_pos == 0xFFFF)
        vl_pos = 0;
    
    return vl_pos;
}

//��ȡ����ж��ٸ���о����
unsigned char Battery_get_cell_numbers(unsigned char bms_index)
{
    unsigned short vl_reg_value;
    unsigned char i, k;
    
    for(i = 0, k = 0; i < 16; i++)
    {
        vl_reg_value = Battery_get_reg_value(bms_index, (ENUM_REG_ADDR_BVOLT1+i));
        if(vl_reg_value != 0xFFFF)
            k++;
    }
    return k;
}


//���ݵ���ڲ�״̬�ж��Ƿ��ܳ�硣�ú���ֻ������CTRL�Ĵ���ǰʹ��
static bool Battery_is_charge_enable_by_BatState(unsigned char bms_index)
{
    unsigned short vl_ctrl_value;
    unsigned short vl_state_value;

    vl_ctrl_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_CTRL);
    vl_state_value = Battery_get_switch_state(bms_index);

    //���ÿ��ƼĴ����г��bit��ֵ��״̬�Ĵ����г��bit��ֵ��ȣ���������
    if(((vl_ctrl_value&BMS_CTRL_BIT_CHG_SWITCH) && (vl_state_value&BMS_STATE_BIT_CHG_SWITCH))||
        ((!(vl_ctrl_value&BMS_CTRL_BIT_CHG_SWITCH)) && (!(vl_state_value&BMS_STATE_BIT_CHG_SWITCH))))
    {
        //������ѯ
        //sl_bat_poll_param.is_poll_allways_en_flag = 1;
        return TRUE;
    }
    // ��Ъ����ѯ
    //sl_bat_poll_param.is_poll_allways_en_flag = 0;
    return FALSE;
}


/*

1--��ѹ
1--��������
1--��������
1--������
1--�����ŵ����
1--�����ŵ����
1--����¶ȹ���
1--�ŵ��¶ȹ���
1--����¶ȹ���
1--�ŵ��¶ȹ���
1--���MOS�¶ȹ���
1--�ŵ�MOS�¶ȹ���
1--�������¶ȹ���
����
1--Ԥ����ʧ�ܳ�ʱ
1--Ԥ����ʧ���ⲿ����/��·

*/

//�жϵ���Ƿ���Էŵ�
bool Battery_is_discharge_enable(unsigned char bms_index)
{
    unsigned short vl_value;
    vl_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_OPFT1);
    if(vl_value&(
    (1<<1)
    |(1<<2)
    |(1<<4)
    |(1<<5)
    |(1<<7)
    |(0x7F<<9)
    ))
    {
        //���ܷŵ�
        return FALSE;
    }

    vl_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_OPFT2);
    if(vl_value&(
        (1<<0)
    |(1<<1)
    |(1<<3)
    |(1<<4)
    |(1<<5)
    ))
    {
        //���ܷŵ�
        return FALSE;
    }

    vl_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_DEVFT1);
    if(vl_value&(
        (0x7F<<0)
        |(7<<9)
        |(1<<15)
    ))
    {
        //���ܷŵ�
        return FALSE;
    }

    vl_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_DEVFT2);
    if(vl_value&(
        (0x3F<<0)
    ))
    {
        //���ܷŵ�
        return FALSE;
    }
    return TRUE;
}




void Battery_clear_switch_state(unsigned char bms_index, unsigned short on)
{
    unsigned short vl_pos, vl_ctrl_value;
    unsigned char i;
    //������������ڲ�ѯ���ƼĴ��������˳�
    for(i = 0; i < NFC_READER_COUNT_MAX; i++)
    {
        if(gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_R_CTRL])
            return;
    }

    //��������������Լ�����棬���˳�
    for(i = 0; i < ENUM_NFC_CMD_INDEX_MAX; i++)
    {
        if(gl_cmd_buf[bms_index][i])
            return;
    }

    #ifdef BAT_PMS_AUTHORITY_FUNCTION_EN
    //���û����֤��ػ��ߵ����֤�����˳�
    //if(FALSE == Is_Authority_done())
    //{
    //    return ;
    //}
    #endif
    
    vl_pos = MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_CTRL);
    vl_ctrl_value = ((gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_H<<8) + 
        gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_L);

    vl_ctrl_value &=(~(on));

    
    if(gl_BP_info.bms_state == ENUM_CHARGE)
    {
        //֪ͨ������
        vl_ctrl_value |= BMS_STATE_BIT_CHG_IN;
    }
    else
    {
        //֪ͨ���γ�
        vl_ctrl_value &= (~BMS_STATE_BIT_CHG_IN);
    }

    
    #ifdef BAT_WARMOR_BY_SELF_ENABLE
    vl_ctrl_value |= (1<<14); // ���Ե���Զ����ȹ���
    #endif
    
    gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_H = (vl_ctrl_value>>8)&0xff;
    gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_L = (vl_ctrl_value>>0)&0xff;
    
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_W_CTRL] = 1;
    //�ٲ�ѯһ��
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_INFO1] = 1;
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_INFO2] = 1;
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_CTRL] = 1;
    //���ڼ���Ƿ���bootloader״̬
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_BLV] = 1;

    //��ȡ��ذ汾��
    #ifndef BAT_PMS_AUTHORITY_FUNCTION_EN
    if(!(BMS_DEV_INFO_FLAG_BIT & Battery_get_reg_valid_flag(bms_index)))
    #endif
    {
        gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_VERSION] = 1;
    }

    #ifdef BAT_PMS_AUTHORITY_FUNCTION_EN
    //���˫����֤����
    Authority_process();
    #endif
}

void Battery_clear_reg_valid_flag(unsigned char bms_index)
{
    gl_bms_info_p[bms_index]->reg_value_ready = 0;
    //if(gl_elock_clear_bms_flag_cnt[bms_index])
    //    Elock_clear_bms_flag(bms_index);
    #ifdef BAT_PMS_AUTHORITY_FUNCTION_EN
    //û�е�أ���λ�����֤����
    if((Is_Authority_done_about_bat_pms()) && 
            ((0 == Authority_get_onoff_about_pms_bat()) ||
            (Is_Authority_done_about_pms_bat())))
    {
        Authority_reset();
    }
    #endif
}

void Battery_clear_reg_valid_flag_on_version_info(unsigned char bms_index)
{
    gl_bms_info_p[bms_index]->reg_value_ready &= (~(BMS_DEV_INFO_FLAG_BIT));
}

//


unsigned char Battery_get_reg_valid_flag(unsigned char bms_index)
{
    return gl_bms_info_p[bms_index]->reg_value_ready;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
static unsigned int sl_bat_chg_current_false_cnt_old = 0;
static unsigned char sl_bat_chg_current_false_flag = 0;
//ȥ��
static unsigned int sl_bat_chg_current_disable_debounce_cnt_old = 0;

/*
    30�����жϵ�ز���磬�жϷ����ǣ�
    SOCС�ڽ��ֵ����90%�����Ƿ��г�����(����50mA)��
    �ù��Ϲ��ൽDamage bit2�ı�־�����ǲ���رյ�غͳ������һ���ָ���磬����������
     */
static void Battery_bat_chg_current_false_process(unsigned char bms_index)
{
    unsigned short vl_reg_value;

    vl_reg_value = Battery_get_switch_state(bms_index);
    if((vl_reg_value & BMS_STATE_BIT_CHG_SWITCH)&& //��س��ܴ�
        (900 > Battery_get_SOC(bms_index)) //SOCС��90%
//        &&
//        ((main_led_get_SOC_full_threshold(0) * 10) > Battery_get_SOC(bms_index)) //SOCС�ڽ��ֵ
        )
    {
        if(//(0 == Battery_get_TCurrent(bms_index))||
            (30005 > Battery_get_TCurrent(bms_index))) // ������С��50mA
        {
            //���ܳ���ˡ�
            if((gl_delay_cnt_test - sl_bat_chg_current_false_cnt_old) > (30*60*1000))
            {
                sl_bat_chg_current_false_flag = 1;
            }
        }
        else
        {
            //�������
            sl_bat_chg_current_false_cnt_old = gl_delay_cnt_test;
            sl_bat_chg_current_false_flag = 0;
        }

        //ȥ����ʱ��λ
        sl_bat_chg_current_disable_debounce_cnt_old = gl_delay_cnt_test;
    }
    else
    {
        //����Ҫ�жϲ����ʱ��

        //ȥ����3��
        if((gl_delay_cnt_test - sl_bat_chg_current_disable_debounce_cnt_old) > 3000)
        {
            sl_bat_chg_current_false_cnt_old = gl_delay_cnt_test;
        }
    }
    
}

//�Ƿ�ʱ����û�е���
unsigned char Battery_is_chg_current_false(void)
{
    return sl_bat_chg_current_false_flag;
}

void Battery_bat_chg_current_false_flag_clear(void)
{
    sl_bat_chg_current_false_cnt_old = gl_delay_cnt_test;
    sl_bat_chg_current_false_flag = 0;
}

void Battery_bat_chg_current_false_timer_reset(void)
{
    sl_bat_chg_current_false_cnt_old = gl_delay_cnt_test;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef CHECKING_BAT_IN_IO_FLAG_ENABLE
//TRUE��ʾʹ��Ӳ�����ظ�������ز���
static bool sl_bat_checking_mode_flag = FALSE;
void Battery_checking_mode_on_bat_in_by_HW_init(void)
{
    //��ʼ��
    if(0 == GPIO_PinInGet(MICRO_SWITCH_OPEN_TEST_PORT, MICRO_SWITCH_OPEN_TEST_PIN))
    {
        //�͵�ƽ��ʾʹ��Ӳ�����ؼ���ز���
        sl_bat_checking_mode_flag = TRUE;
    }
    else
    {
        sl_bat_checking_mode_flag = FALSE;
    }
}

extern bool sl_bat_checking_IO_status;
//ͨ������ϵ�΢�������ж��Ƿ����Ѿ�����
bool Battery_is_bat_in_via_IO_switch(void)
{    
    return sl_bat_checking_IO_status;
}

// ʶ���Ƿ����Ӳ������������ز���γ���״̬
bool Battery_is_checking_mode_on_bat_in_by_HW(void)
{
    return sl_bat_checking_mode_flag;
}

#endif

#ifdef EVENT_TO_STOP_CHARGING_ENABLE

static unsigned short gl_bat_event_to_stop_chg_reg_state[18];
static void Battery_save_bat_reg_state_when_event_to_stop_chg(unsigned char bms_index)
{
#ifndef CABINET_WARMOR_SUPPLY_BY_CHARGER
    gl_bat_event_to_stop_chg_reg_state[0] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_SN34);
    gl_bat_event_to_stop_chg_reg_state[1] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_SN56);
    gl_bat_event_to_stop_chg_reg_state[2] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_SN78);
    gl_bat_event_to_stop_chg_reg_state[3] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_STATE);
    gl_bat_event_to_stop_chg_reg_state[4] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_CTRL);
    gl_bat_event_to_stop_chg_reg_state[5] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_DEVFT1);
    gl_bat_event_to_stop_chg_reg_state[6] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_DEVFT2);    
    gl_bat_event_to_stop_chg_reg_state[7] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_OPFT1);
    gl_bat_event_to_stop_chg_reg_state[8] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_OPFT2);
    gl_bat_event_to_stop_chg_reg_state[9] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_SOC);
    gl_bat_event_to_stop_chg_reg_state[10] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_LVOLT);
    gl_bat_event_to_stop_chg_reg_state[11] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_HTEMP);
    gl_bat_event_to_stop_chg_reg_state[12] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_TVOLT);
    gl_bat_event_to_stop_chg_reg_state[13] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_TCURR);

    gl_bat_event_to_stop_chg_reg_state[17] = 0;
    memcpy((unsigned char*)&gl_bat_event_to_stop_chg_reg_state[14], 
            (unsigned char*)&gl_poll_cmd_cnf_param, 
            7);//sizeof(st_poll_cmd_cnf_param));
#endif
}

#ifndef CABINET_WARMOR_SUPPLY_BY_CHARGER
unsigned int Battery_get_bat_reg_state_when_event_to_stop_chg(unsigned char* p)
{
    //���ش洢��ָ��
    memcpy(p, (unsigned char*)gl_bat_event_to_stop_chg_reg_state, 18*2);
    return (18*2);
}
#endif

void Battery_event_to_stop_chg_param_init(void)
{
    memset(&gl_event_to_stop_chg_param, 0, sizeof(st_event_to_stop_chg_param));

    /*
        ����Ĭ�Ϲرռ��Ĺ��ϣ�����:
        ENUM_SCR_OF_STOP_FLAG_CELL_V_DIFF_VALUE_EXCEEDED, // ��о��ѹ�������ߵ�о��ѹ����͵�о��ѹ��ֵ���ڵ���0.1V
    */
    gl_event_to_stop_chg_param.src_of_stop_flag_disable_flag |= (1 << ENUM_SCR_OF_STOP_FLAG_CELL_V_DIFF_VALUE_EXCEEDED);
}

void Battery_set_event_to_stop_chg_flag(enum_src_of_stop_flag src )
{
    #ifdef BAT_ERROR_CHECK_DELAY_AFTER_BAT_UPDATE
    if(sl_bat_error_check_delay_after_bat_update_cnt)
        return;
    #endif

    //����ֹ���Ĺ��ϣ��������ù��ϱ�־
    if(gl_event_to_stop_chg_param.src_of_stop_flag_disable_flag & (1<<src))
    {
        return ;
    }
    
    
    if(ELOCK_OPEN== ELock_get_status((ENUM_ELOCK_INDEX)0))
    {
        //������ſ���ʱ�򣬲����øù��ϱ�־
        return;
    }

    if(ENUM_SCR_OF_STOP_FLAG_CHARGER_OTP_LOCK != src) // �������¹��ϻ����������
    {
        gl_event_to_stop_chg_param.stop_flag = 1;
    }
    gl_event_to_stop_chg_param.src_of_stop_flag |= (1<<src);
    Battery_save_bat_reg_state_when_event_to_stop_chg(0);
}

bool Battery_is_event_to_stop_chg_flag_be_set(enum_src_of_stop_flag src )
{
    return ((gl_event_to_stop_chg_param.src_of_stop_flag & (1 << src))?TRUE:FALSE);
}

void Battery_clear_event_to_stop_chg_flag(void)
{
    gl_event_to_stop_chg_param.stop_flag = 0;
    gl_event_to_stop_chg_param.src_of_stop_flag = 0;
    gl_event_to_stop_chg_param.current_diff_counter = 0;
    gl_event_to_stop_chg_param.bat_current_out_counter = 0;
}

//���ý�ֹ���й����¼��������ı�־
void Battery_set_event_to_stop_chg_flag_disable_flag(unsigned int flag)
{
    gl_event_to_stop_chg_param.src_of_stop_flag_disable_flag |= flag;
}
//�����ֹ���й����¼��������ı�־
void Battery_clear_event_to_stop_chg_flag_disable_flag(unsigned int flag)
{
    gl_event_to_stop_chg_param.src_of_stop_flag_disable_flag &= flag;
}

unsigned int Battery_get_event_to_stop_chg_flag_disable_flag(void)
{
    return gl_event_to_stop_chg_param.src_of_stop_flag_disable_flag;
}

unsigned int Battery_get_src_of_stop_flag(void)
{
    return gl_event_to_stop_chg_param.src_of_stop_flag;
}

unsigned char Battery_get_event_to_stop_chg_flag(void)
{
    return gl_event_to_stop_chg_param.stop_flag;
}

void Battery_set_event_to_stop_chg_bat_ID(unsigned char * p_id)
{
    memcpy(gl_event_to_stop_chg_param.bat_ID, p_id, 6);
}

//�Ƚϵ��ID�Ƿ��б仯
void Battery_event_to_stop_chg_bat_ID_compare_and_set(unsigned char* p_in_id)
{
    unsigned char i;
    for(i = 0; i < 6; i++)
    {
        if(gl_event_to_stop_chg_param.bat_ID[i] != p_in_id[i])
        {
            Battery_set_event_to_stop_chg_bat_ID(p_in_id);
            Battery_clear_event_to_stop_chg_flag();
        }
    }
}

#endif


////////////////////////////////////////////////////////////////////////////////////////////////////
//2.  H,M,L�����ϣ�����10s���ϱ�����λ��

//�������Ͱ�ʱ��
//����total_counts, error_counts, total_counts�������10, ͬʱerror_counts/total_counts  > 0.8ʱ��
//��ʱ��total_time, total_time�������10S

//total_time���������ӣ�ֱ��10S

//����ʱ������total_time, total_counts, error_counts;


//�������10�붼�ǹ��ϣ����ϱ��ù��ϡ��м�����ù��������������¼�ʱ��
//��������ڿ���״̬���߼�ⲻ����أ����¼�ʱ��

typedef struct{
//unsigned short total_counts;
//unsigned short error_counts;
unsigned int total_time_cnt; //ms
}st_bat_fault_debounce_param;
st_bat_fault_debounce_param gl_bat_fault_debounce_param[8];

#define BATTERY_BAT_FAULT_DEBOUNCE_CNT_MAX 12000


void Battery_bat_fault_debounce_param_init(void)
{
    memset((char*)gl_bat_fault_debounce_param, 0, sizeof(gl_bat_fault_debounce_param));
}

void Battery_bat_fault_debounce_param_timer_cb(void)
{
    unsigned char i;
    for(i = 0; i < 8; i++)
    {
        if(gl_bat_fault_debounce_param[i].total_time_cnt < BATTERY_BAT_FAULT_DEBOUNCE_CNT_MAX)
            gl_bat_fault_debounce_param[i].total_time_cnt++;
    }
}

//�Ƿ���������ǿ����ϱ���
//nb : �ڼ���λ�Ĺ���
bool Battery_is_bat_fault_debounce_valid(unsigned char nb)
{
    return (gl_bat_fault_debounce_param[nb].total_time_cnt >= BATTERY_BAT_FAULT_DEBOUNCE_CNT_MAX)?TRUE:FALSE;
}


void Battery_bat_fault_debounce_process(void)
{
    unsigned short vl_tmp_value;
    unsigned short vl_value_opft2, vl_value_devft1, vl_value_devft2;
    vl_tmp_value = Battery_get_reg_value(0, ENUM_REG_ADDR_OPFT1);
    //pl_bat_info_data->stl_fault = 0;
    if(!(vl_tmp_value&(1<<0)))
    {
        //����ѹ
        gl_bat_fault_debounce_param[0].total_time_cnt = 0;
    }

    if(!(vl_tmp_value&((1<<1)|(1<<2))))
    {
        //��Ƿѹ

        gl_bat_fault_debounce_param[1].total_time_cnt = 0;
    }
    
    if(!(vl_tmp_value&((1<<3)|((1<<4)|(1<<5)))))
    {
        //������
        gl_bat_fault_debounce_param[2].total_time_cnt = 0;
    }

    if(!(vl_tmp_value &((1<<6)|(1<<7)|(1<<10)|(1<<11)|(1<<12))))
    {
        //������
        gl_bat_fault_debounce_param[3].total_time_cnt = 0;
    }

    if(!(vl_tmp_value &((1<<8)|(1<<9))))
    {
        //������
        gl_bat_fault_debounce_param[4].total_time_cnt = 0;
    }

    vl_value_opft2 = Battery_get_reg_value(0, ENUM_REG_ADDR_OPFT2);

    vl_value_devft1 = Battery_get_reg_value(0, ENUM_REG_ADDR_DEVFT1);
    vl_value_devft1 &= (~((1<<7)|(1<<8)));
    
    vl_value_devft2 = Battery_get_reg_value(0, ENUM_REG_ADDR_DEVFT2);
    
    if((vl_tmp_value &((1<<14)|(1<<15)))||
        vl_value_opft2||
        vl_value_devft1||
        vl_value_devft2// ���ǵ����û����ȫ���ȥ��ʱ����NFC��û�л��������Բ��ӵ�صĻ����ж�
        )
    {
        //����

        gl_bat_fault_debounce_param[5].total_time_cnt = 0;
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
��ȡ�����ʷ��¼
*/


//����������¼��š���Ϊ��¼�ķ�Χ����0~0xFFFFFFFF
unsigned int Battery_history_record_get_real_index(unsigned int index)
{
    if(index > HISTORY_RECORD_INDEX_END)
    {
        return (index - HISTORY_RECORD_INDEX_END);
    }
    else
    {
        return index;
    }
}

//������֮��Ĳ�ֵ(end-begin)����Ϊ��¼�ķ�Χ����0~0xFFFFFFFF
unsigned int Battery_history_record_get_real_diff_value(unsigned int begin_index, unsigned int end_index)
{
    if(begin_index > end_index)
    {
        return (HISTORY_RECORD_INDEX_END - begin_index) + end_index;
    }
    else
    {
        return (end_index - begin_index);
    }
}

void Battery_history_record_param_init(void)
{
    memset(&gl_bat_history_record_param, 0, sizeof(st_bat_history_record_param));
}

//���³�ʼ��
void Battery_history_record_param_reset(void)
{
    Battery_history_record_param_init();
}

//���ڵ��ID�����ü�¼�еĵ��ID�Ƿ�һ��
bool Battery_history_record_is_bat_ID_same(unsigned char bat_ID[6])
{
    if((bat_ID[0] != gl_bat_history_record_param.bat_ID[0])
    ||(bat_ID[1] != gl_bat_history_record_param.bat_ID[1])
    ||(bat_ID[2] != gl_bat_history_record_param.bat_ID[2])
    ||(bat_ID[3] != gl_bat_history_record_param.bat_ID[3])
    ||(bat_ID[4] != gl_bat_history_record_param.bat_ID[4])
    ||(bat_ID[5] != gl_bat_history_record_param.bat_ID[5]))
    {
        return FALSE;
    }
    return TRUE;
}

//��ȡ�Ѿ��������ʷ����
bool Battery_history_record_get_data(unsigned char bat_ID[6], unsigned char buf_out[128])
{
    if((!gl_bat_history_record_param.is_buf_valid)
        ||(bat_ID[0] != gl_bat_history_record_param.bat_ID[0])
        ||(bat_ID[1] != gl_bat_history_record_param.bat_ID[1])
        ||(bat_ID[2] != gl_bat_history_record_param.bat_ID[2])
        ||(bat_ID[3] != gl_bat_history_record_param.bat_ID[3])
        ||(bat_ID[4] != gl_bat_history_record_param.bat_ID[4])
        ||(bat_ID[5] != gl_bat_history_record_param.bat_ID[5]))
    {
        return FALSE;
    }

    memcpy(buf_out, gl_bat_history_record_param.history_record_buf, 128);
    return TRUE;
}

void Battery_history_record_set_data(unsigned char buf_out[128])
{
    unsigned char vl_bat_ID[6];
    unsigned int vl_int_tmp;

    vl_int_tmp = buf_out[0]|
                    (buf_out[1] << 8)|
                    (buf_out[2] << 16)|
                    (buf_out[3] << 24);
    if((gl_bat_history_record_param.history_record_earliest_index == vl_int_tmp)
        &&(0 == gl_bat_history_record_param.is_earliest_param_valid))
    {
        memcpy(gl_bat_history_record_param.history_record_earliest_index_datetime, &buf_out[4], 6);
        gl_bat_history_record_param.is_earliest_param_valid = 1;
        return;
    }

    if((gl_bat_history_record_param.history_record_lastest_index == vl_int_tmp)
        &&(0 == gl_bat_history_record_param.is_lastest_param_valid))
    {
        memcpy(gl_bat_history_record_param.history_record_lastest_index_datetime, &buf_out[4], 6);
        gl_bat_history_record_param.is_lastest_param_valid = 1;
        return;
    }
    
    Battery_get_bat_ID(0,vl_bat_ID);
    //�����ص�ID�ʹ洢��ID��һ����˵����������ˡ���Ҫ�������ݲ���
    if((vl_bat_ID[0] != gl_bat_history_record_param.bat_ID[0])
        ||(vl_bat_ID[1] != gl_bat_history_record_param.bat_ID[1])
        ||(vl_bat_ID[2] != gl_bat_history_record_param.bat_ID[2])
        ||(vl_bat_ID[3] != gl_bat_history_record_param.bat_ID[3])
        ||(vl_bat_ID[4] != gl_bat_history_record_param.bat_ID[4])
        ||(vl_bat_ID[5] != gl_bat_history_record_param.bat_ID[5])
        ||((gl_bat_history_record_param.is_buf_valid)&&(0 == gl_bat_history_record_param.is_buf_updated))
        ||(0 == gl_bat_history_record_param.is_index_valid)
        ||(gl_bat_history_record_param.history_record_read_cnt >= gl_bat_history_record_param.history_record_cnt)
        )
    {
        //������Ҫ���ID����ڵ�ص�ID��һ�����������ݡ�
        //������ݻ�û���ϴ�����������
        //�����ʼ��ź������������Ч�ģ���������
        //��ȡ�������Ѿ�������Ҫ�Ķ�������������
    }
    else
    {
        memcpy(gl_bat_history_record_param.history_record_buf, buf_out, 128);
        gl_bat_history_record_param.is_buf_valid = 1;
        Battery_history_record_clear_updated_flag();
        gl_bat_history_record_param.history_record_read_cnt++;
    }
     
}

//������Ҫ��ȡ����ʷ��¼��ʼ��ź���Ҫ��ȡ������
void Battery_history_record_set_index_param(unsigned char bat_ID[6], unsigned int log_head, unsigned char datetime[6], unsigned int log_cnt, unsigned int log_read_cnt)
{
    memcpy(gl_bat_history_record_param.bat_ID, bat_ID, 6);
    gl_bat_history_record_param.history_record_begin_index = log_head;
    memcpy(gl_bat_history_record_param.history_record_begin_index_datetime, datetime, 6);
    gl_bat_history_record_param.history_record_cnt = log_cnt;
    //��ǲ�����Ч
    gl_bat_history_record_param.is_index_valid = 1;
    gl_bat_history_record_param.history_record_read_cnt = log_read_cnt;
    gl_bat_history_record_param.is_buf_valid = 0;
}

bool Battery_history_record_is_data_updated(void)
{
    return gl_bat_history_record_param.is_buf_updated?TRUE:FALSE;
}

void Battery_history_record_clear_updated_flag(void)
{
    gl_bat_history_record_param.is_buf_updated = 0;
}

void Battery_history_record_set_updated_flag(void)
{
    gl_bat_history_record_param.is_buf_updated = 1;

    //�������ʷ���ݱ���λ����ȡ�����ϲ�ѯһ���Ƿ���Ҫ��ȡ��ʷ��¼����
    Batter_history_record_set_nfc_cmd_interval_cnt(0);
}

//��ѯĳֵ�Ƿ���һ����Χ�ڡ�
bool Battery_history_record_is_within_range(unsigned int range_begin, unsigned int range_end, unsigned int data)
{
    if(range_end >= range_begin)
    {
        if((data > range_end)||(data < range_begin))
        {
            //��Ų���ȷ
            return FALSE;
        }
    }
    else
    {
        if((range_begin < data )&&(data < range_end))
        {
            //��Ų���ȷ
            return FALSE;
        }
    }

    return TRUE;
}


//��ѯ����Ƿ��ڼ�¼��Χ��
bool Battery_history_record_is_index_within_range(unsigned int index)
{
    unsigned int vl_int_new_tmp;
    unsigned int vl_int_old_tmp;
    //��ȡ�������������¼���
    vl_int_new_tmp = Battery_get_newest_record_index_number();

    //��ȡ�������������¼���
    vl_int_old_tmp = Battery_get_oldest_record_index_number();

    return Battery_history_record_is_within_range(vl_int_old_tmp, vl_int_new_tmp, Battery_history_record_get_real_index(index));
}

//�Ƿ���Ҫͨ��NFC��ȡ��ʷ��¼
bool Battery_history_record_is_need_to_read_record(void)
{
    unsigned char vl_bat_ID[6];

    //����Ѿ���ȡ���������ݣ��˳�
//    if((gl_bat_history_record_param.history_record_read_cnt == gl_bat_history_record_param.history_record_cnt)
//    //||() //��ȡȫ�����ݵ����
//    )
//    {
//        return FALSE;
//    }
    
    Battery_get_bat_ID(0, vl_bat_ID);
    if(((0 == gl_bat_history_record_param.is_buf_valid)// ��ʷ���ݻ�����Ч
        ||(gl_bat_history_record_param.is_buf_updated)) //�Ѿ��ϱ�����λ��
        &&(gl_bat_history_record_param.is_index_valid) // ���õı������Ч��
        &&(TRUE == Battery_history_record_is_bat_ID_same(vl_bat_ID)) //Bat ID�Ͳ��ڵ��һ��
        //&&(TRUE == Battery_history_record_is_index_within_range(gl_bat_history_record_param.history_record_begin_index)) //��ʼ����ڼ�¼��Χ��
        &&(TRUE == Battery_history_record_is_index_within_range(
                    Battery_history_record_get_real_index(gl_bat_history_record_param.history_record_begin_index + 1 + gl_bat_history_record_param.history_record_read_cnt))
                                                                ) //׼����ȡ�ı���ڼ�¼��Χ��
        )
    {
        return TRUE;
    }
    if((((0xFFFFFFFF != gl_bat_history_record_param.history_record_cnt) && (gl_bat_history_record_param.history_record_read_cnt < gl_bat_history_record_param.history_record_cnt)) // ���޶�ȡʱ��
            ||((0xFFFFFFFF == gl_bat_history_record_param.history_record_cnt) && (gl_bat_history_record_param.history_record_lastest_index != Battery_history_record_get_real_index(gl_bat_history_record_param.history_record_begin_index + gl_bat_history_record_param.history_record_read_cnt)))) // ���޶�ȡʱ�򣬶������һ��Ϊֹ
        &&(FALSE == Battery_history_record_is_index_within_range(
                                    Battery_history_record_get_real_index(gl_bat_history_record_param.history_record_begin_index + 1 + gl_bat_history_record_param.history_record_read_cnt))))
    {
        gl_bat_history_record_param.history_record_read_cnt++;
    }
    return FALSE;
}

/*
��ȡ��ʷ��¼���̵�״̬��״̬

return:
00���ȴ�״̬
01������"��ȡ��ʼ�����ʷ��¼��źͼ�¼����"
10���Ѿ��ɹ���ȡ1�������ʷ��¼
*/
unsigned char Battery_history_record_state_of_process(void)
{
    unsigned char vl_bat_ID[6];
    Battery_get_bat_ID(0, vl_bat_ID);
    if((0 == gl_bat_history_record_param.is_index_valid) //���ò����Ƿ���Ч
        ||(FALSE == Battery_history_record_is_bat_ID_same(vl_bat_ID)) //���ڵ�ص�ID�����õ�ID�Ƿ�һ��
        
        ||((1 == gl_bat_history_record_param.is_index_valid)&&
            ((gl_bat_history_record_param.history_record_cnt == gl_bat_history_record_param.history_record_read_cnt) || 
                    ((0xFFFFFFFF == gl_bat_history_record_param.history_record_cnt) && (gl_bat_history_record_param.history_record_lastest_index == Battery_history_record_get_real_index(gl_bat_history_record_param.history_record_begin_index + gl_bat_history_record_param.history_record_read_cnt))))
            &&(gl_bat_history_record_param.is_buf_updated)) //��ȫ����һ�����õ�����
        )
    {
        return 0x01; // ����"��ȡ��ʼ�����ʷ��¼��źͼ�¼����"
    }

    if((0 == gl_bat_history_record_param.is_buf_updated)
        &&(gl_bat_history_record_param.is_buf_valid))
    {
        return 0x02; // �Ѿ��ɹ���ȡ1�������ʷ��¼
    }

    return 0x00; // �ȴ�״̬

}

void Batter_history_record_set_nfc_cmd_interval_cnt(unsigned int cnt)
{
    gl_bat_history_record_param.NFC_CMD_interval_cnt = cnt;
}


/*
3���Ӳ�ѯһ���Ƿ���Ҫ��ȡ��ʷ��¼����
���߸շ�������ʷʱ�����λ��ʱ���ٲ�ѯ�Ƿ���Ҫ��ȡ��ʷ��¼��
*/
void Battery_history_record_process(void)
{
    //
    if((0 == gl_bat_history_record_param.NFC_CMD_interval_cnt) //�ӳ���ɺ�
        &&(gl_cmd_buf[0][ENUM_NFC_CMD_INDEX_HISTORY_RECORD] == 0) // ����ִ�к�
        &&(MM_is_snd_done_by_index(gl_modbus_param.BMS_index) // nfc������ɺ�
        &&(gl_cmd_buf[0][ENUM_NFC_CMD_INDEX_R_CTRL] == 1) //�����Ĵ����Ĳ�ѯ�Ѿ���ʼ��
        )
        //�����ѯ�ڼ䣬ͬʱ��о��͵�ѹС��3V����ֹ��ȡBAT��ʷ����
        &&((0 != sl_bat_poll_param.is_poll_allways_en_flag) //����һֱ��ѯ״̬
        || (sl_bat_poll_param.poll_interval_MAX < BAT_POLL_BIG_INTERVAL_MAX))//��о��ѹ���ڵ���3V
    )
    {
        if(Battery_history_record_is_need_to_read_record())
        {
            //��ȡ�������������¼���
            gl_bat_history_record_param.history_record_lastest_index = Battery_get_newest_record_index_number();
                //��ȡ�������������¼���
            gl_bat_history_record_param.history_record_earliest_index = Battery_get_oldest_record_index_number();
                
            gl_cmd_buf[0][ENUM_NFC_CMD_INDEX_HISTORY_RECORD] = 1;
        }
    }
}

/*
�Ƚ�ʱ��
DT��ʽ: ��/��/��/ʱ/��/�룻BCD��
return :
0, ��ͬ
1, DT1��DT2��ʱ�������
-1, DT2��DT1��ʱ�������
*/
signed char Battery_history_record_datetime_compare(unsigned char *DT1, unsigned char *DT2)
{
    unsigned char i;
    for(i = 0; i < 6; i++)
    {
        if(DT1[i] > DT2[i])
        {
            return 1;
        }

        if(DT2[i] > DT1[i])
        {
            return -1;
        }
    }
    return 0;       
}

////////////////////////////////////////////////////////////////////////////////////////////////////

