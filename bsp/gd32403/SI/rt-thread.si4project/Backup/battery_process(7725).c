

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
            //Battery_discharge_process();
            break;
        case ENUM_CHARGE:
            Battery_charge_process();
            break;
        case ENUM_SLEEP:
            break;
        default:
            break;
    }
    #if 1


    // ������������Ч��־�иı䣬Ӧ����ѯһ�ε��
    if(master_rs485_is_charger_status_changed())
    {
        sl_bat_poll_param.poll_inerval_timer_cnt = BAT_POLL_ON_TIME_MAX-1;
    }

    if(sl_bat_poll_param.is_status_changed_on_door)
    {
        sl_bat_poll_param.poll_allways_en_delay_cnt = BAT_POLL_DELAY_MAX;
        sl_bat_poll_param.is_status_changed_on_door = 0;
    }
    
    if(sl_bat_poll_param.is_poll_allways_en_flag || 
        (sl_bat_poll_param.poll_inerval_timer_cnt < BAT_POLL_ON_TIME_MAX)||
        (sl_bat_poll_param.poll_interval_for_bms_update_cnt)) // ����յ�һ����ʼ����BMS������������ѯһ��ʱ��
    {
        if(sl_bat_poll_param.poll_inerval_timer_cnt == 0)
        {
            sl_bat_poll_param.poll_inerval_timer_cnt = sl_bat_poll_param.poll_interval_MAX;
        }

        //��ȡ�����Ϣ
        Battery_info_polling_Process(vl_BMS_index);
    }
    #endif
    Battery_chg_hw_ctrl_process();

    //��ȡ�����ʷ����
    Battery_history_record_process();

    
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
unsigned char is_battery_error(unsigned char bms_index, unsigned char option_ignore)
{
    unsigned short vl_value;
    vl_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_OPFT1);
    //�����¶ȹ���
    if(option_ignore == BATTERY_ERROR_OPTION_IGNORE_TEMPERATURE)
    {
        vl_value &= (~(0x7F<<6)); // bit6~bit12 �����¶ȵĹ��϶�����
    }
    if(vl_value&(~((1<<15)|(1<<14)|
        //(1<<0)|(1<<1)|(1<<2))))//���ʱ���ù�
        #ifdef BAT_WARMOR_BY_SELF_ENABLE
        (1<<8)| // Bit8   1--����¶ȹ���
        #endif
        (1<<1)|(1<<2))))//���ʱ���ù�//2018-8-21�޸ģ���ѹ���ϲ�������
    {
        return 1;
    }

    vl_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_OPFT2);
    if(vl_value)
    {
        return 1;
    }

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
    return 0;
}

#ifdef TEST_DETECT_BAT_ERROR_EN

//�õ���Ƿ��������������mos��
unsigned char is_battery_checked_enable_for_detecting_bat(unsigned char bms_index)
{
    unsigned short vl_value;
    vl_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_OPFT1);
    if(vl_value)
    {
        return 1;
    }

    vl_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_OPFT2);
    if(vl_value)
    {
        return 1;
    }

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
    return 0;
}
#endif

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

#ifdef TEST_BAT_V_VIA_CHARGER_EN

//��ʼ����������ʱ����Ҫ��ʼ��
void Battery_check_bat_v_via_charger_init(void)
{
    memset(&gl_check_bat_v, 0, sizeof(st_check_bat_v));
}

//�Ƿ���Ҫ���
bool Battery_check_bat_v_is_off(void)
{
    return (gl_check_bat_v.is_need_bat_v_checking == 0)?TRUE:FALSE;
}


//�Ƿ����ڼ����
bool Battery_check_bat_v_is_on(void)
{
    return (gl_check_bat_v.is_need_bat_v_checking == 1)?TRUE:FALSE;
}

//�Ƿ��Ѿ�������
bool Battery_check_bat_v_is_end(void)
{
    return (gl_check_bat_v.is_need_bat_v_checking == 2)?TRUE:FALSE;
}

//�Ƿ��⵽��ص�ѹ
bool Battery_check_bat_v_is_valid(void)
{
    return (gl_check_bat_v.is_bat_v_valid == 1)?TRUE:FALSE;
}

void Battery_check_bat_v_set_valid_flag(void)
{
    gl_check_bat_v.is_bat_v_valid = 1;
}

#define BATTERY_BAT_V_CHARGER_CHECK_TOTOL_TIME_CNT_MAX 5000
#define BATTERY_BAT_V_CHARGER_COMM_TIMEOUT_CNT_MAX 1000 // 1S
#define BATTERY_BAT_V_CHARGER_CMD_CNT_MAX 100
//�ز���ʱ����
bool Battery_check_bat_v_process(unsigned char bms_index)
{
    
    unsigned short vl_bms_state_reg, vl_bms_ctrl_reg;

    #ifdef CHECKING_BAT_IN_IO_FLAG_ENABLE
    //û��΢������ʱ�򣬳��������ص�ѹ�Ĺ���Ҳȥ��
    //if(FALSE == Battery_is_checking_mode_on_bat_in_by_HW())
    //{
    //    return FALSE;
    //}
    #endif
    
    vl_bms_state_reg = Battery_get_switch_state(bms_index);
    vl_bms_ctrl_reg = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_CTRL);

    if((gl_check_bat_v.is_need_bat_v_checking == 0)
        #if 0 //
        &&
        ()&& //û��Ƿѹ
        ()&& //û���¶��쳣
        (master_rs485_is_charger_OK()) // ���������
        #endif
        )
    {
        gl_check_bat_v.bat_v_charger_comm_timeout_cnt = BATTERY_BAT_V_CHARGER_COMM_TIMEOUT_CNT_MAX; // 1��
        gl_check_bat_v.bat_v_charger_cmd_cnt = BATTERY_BAT_V_CHARGER_CMD_CNT_MAX;
        gl_check_bat_v.bat_v_charger_check_totol_time_cnt = BATTERY_BAT_V_CHARGER_CHECK_TOTOL_TIME_CNT_MAX;
        gl_check_bat_v.is_need_bat_v_checking = 1;
    }

    if(gl_check_bat_v.is_need_bat_v_checking == 1)
    {
        //�Ƿ��д���������Ƿ��й��ϣ��Ƿ��г����
        if((!(0x08&Battery_get_switch_state(0)))||
            (is_battery_error(0, BATTERY_ERROR_OPTION_IGNORE_NONE)))
        {
            //�������
            gl_check_bat_v.is_need_bat_v_checking = 2;

            gl_check_bat_v.is_bat_v_valid = 0;
            gl_check_bat_v.bat_v_value = 0;

            return FALSE;
        }
        else
        {
            //��ʼ���

            //�򿪵��Ԥ��
            if((BMS_CTRL_BIT_PRESUPPLY_SWITCH != ((BMS_CTRL_BIT_PRESUPPLY_SWITCH) & vl_bms_state_reg))&&
            (BMS_CTRL_BIT_PRESUPPLY_SWITCH != ((BMS_CTRL_BIT_PRESUPPLY_SWITCH) & vl_bms_ctrl_reg)))
            {
                Battery_set_switch_state(bms_index, BMS_CTRL_BIT_PRESUPPLY_SWITCH);
                //������ر�
                master_rs485_close_charger_by_cmd();
                gl_check_bat_v.bat_v_charger_comm_timeout_cnt = BATTERY_BAT_V_CHARGER_COMM_TIMEOUT_CNT_MAX; // 1��
                gl_check_bat_v.bat_v_charger_cmd_cnt = BATTERY_BAT_V_CHARGER_CMD_CNT_MAX;

                if(!gl_check_bat_v.bat_v_charger_check_totol_time_cnt)
                {
                    //�������
                    gl_check_bat_v.is_need_bat_v_checking = 2;

                    gl_check_bat_v.is_bat_v_valid = 0;
                    gl_check_bat_v.bat_v_value = 0;

                    return FALSE;
                }
                
                return TRUE;
            }
            else
            {
                if(((gl_check_bat_v.bat_v_charger_comm_timeout_cnt == 0)&&(TRUE == master_rs485_is_cmd_list_empty()))|| //��ʱ��ʱ
                    (gl_check_bat_v.bat_v_charger_cmd_cnt == 0)) //��ѯ�����������
                {
                    //�������
                    gl_check_bat_v.is_need_bat_v_checking = 2;

                    //���ý��
                    if(400 < master_rs485_get_V_detected_by_charger()) //��صĵ�ѹ�������20V
                    {
                        gl_check_bat_v.is_bat_v_valid = 1;
                        gl_check_bat_v.bat_v_value = master_rs485_get_V_detected_by_charger();
                    }
                    else
                    {
                        gl_check_bat_v.is_bat_v_valid = 0;
                        gl_check_bat_v.bat_v_value = 0;
                    }

                    return FALSE;
                }
                else if(gl_check_bat_v.bat_v_charger_comm_timeout_cnt != 0)
                {
                    //������ر�
                    master_rs485_close_charger_by_cmd();
                    return TRUE;
                }
            }
        }
    }

    return FALSE;
}
#endif

#ifdef TEST_DETECT_BAT_ERROR_EN

static unsigned short gl_detect_bat_error_reg_state[18];
void Battery_save_bat_reg_state_when_detect_bat_error(unsigned char bms_index)
{
    gl_detect_bat_error_reg_state[0] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_SN34);
    gl_detect_bat_error_reg_state[1] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_SN56);
    gl_detect_bat_error_reg_state[2] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_SN78);
    gl_detect_bat_error_reg_state[3] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_STATE);
    gl_detect_bat_error_reg_state[4] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_CTRL);
    gl_detect_bat_error_reg_state[5] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_DEVFT1);
    gl_detect_bat_error_reg_state[6] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_DEVFT2);    
    gl_detect_bat_error_reg_state[7] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_OPFT1);
    gl_detect_bat_error_reg_state[8] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_OPFT2);
    gl_detect_bat_error_reg_state[9] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_SOC);
    gl_detect_bat_error_reg_state[10] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_LVOLT);
    gl_detect_bat_error_reg_state[11] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_HTEMP);
    gl_detect_bat_error_reg_state[12] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_TVOLT);
    gl_detect_bat_error_reg_state[13] = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_TCURR);

    gl_detect_bat_error_reg_state[17] = 0;
    memcpy((unsigned char*)&gl_detect_bat_error_reg_state[14], 
            (unsigned char*)&gl_poll_cmd_cnf_param, 
            7);//sizeof(st_poll_cmd_cnf_param));
    
}

unsigned int Battery_get_bat_reg_state_when_detect_bat_error(unsigned char* p)
{
    //���ش洢��ָ��
    memcpy(p, (unsigned char*)gl_detect_bat_error_reg_state, 18*2);
    return (18*2);
}

bool Battery_check_bat_is_on(void)
{
    return (gl_check_bat_err.is_need_bat_checking == 1)?TRUE:FALSE;
}

//����Ƿ��Ѿ���������
bool Battery_check_bat_is_end(void)
{
    return (gl_check_bat_err.is_need_bat_checking == 2)?TRUE:FALSE;
}


void Battery_check_bat_clear_on_and_error(void)
{
    memset(&gl_check_bat_err, 0, sizeof(st_check_bat_err));
}

unsigned char Battery_check_bat_is_bat_error(void)
{
    return gl_check_bat_err.is_bat_error;
}

unsigned char Battery_check_bat_get_bat_error(void)
{
    return gl_check_bat_err.is_bat_error;
}


//���ÿ�ʼ����صĲ���
void Bttery_check_bat_init(void)
{
    if(is_battery_voltage_OK(0))
    {
        gl_check_bat_err.checking_step = 0;
        gl_check_bat_err.is_need_bat_checking = 1;
        gl_check_bat_err.is_bat_error = 0;
        gl_check_bat_err.checking_step_timeout_cnt = 0;
        gl_check_bat_err.bat_step_timeout_err_cnt = 0;
    }
}

//���ź��ӳ�30���ٽ��е�ع��ϼ��
#define CHECK_BAT_ERROR_DOOR_CLOSE_DELAY_MAX 30000
//��ؿ��ػ��߳�������ض����󣬵ȴ����������ѹʱ��
#define CHECK_BAT_ERROR_WAIT_FOR_CHARGER_VOLTAGE_DELAY_MAX 40000
//���ʱ��ÿ��������(step)�������ʱ��
#define CHECK_BAT_ERROR_STEP_TIMEOUT_CNT_MAX 300000 // 5����

bool Battery_check_bat_process(void)
{
    //bool sl_elock_status;
    unsigned short vl_bat_voltage;
    //unsigned short vl_soc;
    unsigned char vl_cell_numbers;
    /*��������ɿ���أ����ҵ�ص�ѹ����ĳ��ѹ��Χ�ڣ�
    ��ʼ����ʱ����������ع���*/
    if((gl_check_bat_err.bat_process_cnt == 0)&&
        (gl_check_bat_err.is_need_bat_checking == 0)
    #ifdef BAT_ERROR_CHECK_DELAY_AFTER_BAT_UPDATE
    &&(0 == sl_bat_error_check_delay_after_bat_update_cnt)
    #endif
    )
    {
        gl_check_bat_err.bat_process_cnt = 5000;// ÿ5�����һ������

        
        //sl_elock_status = ELock_get_status((ENUM_ELOCK_INDEX)0);
        vl_bat_voltage = Battery_get_voltage(0);
        //vl_soc = Battery_get_SOC(0);
        vl_cell_numbers = Battery_get_cell_numbers(0);
        //if(sl_elock_status_old != sl_elock_status)
        if((Battery_chg_is_charge_enable(0))
            &&(ADC_is_CHG_enable())
            &&(master_rs485_is_charger_OK())
            &&(ELOCK_LOCK == ELock_get_status((ENUM_ELOCK_INDEX)0))
            &&(0 == is_battery_checked_enable_for_detecting_bat(0))
            #ifdef EVENT_TO_STOP_CHARGING_ENABLE
            &&(!Battery_get_event_to_stop_chg_flag())
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
            #ifdef CABINET_WARMOR_SUPPLY_BY_CHARGER
            &&(0 == master_rs485_get_charger_warmer_fsm_state()) //���ģʽ
            #endif
        )
        {
            //sl_elock_status_old = sl_elock_status;
            
            //vl_soc = Battery_get_reg_value(0, ENUM_REG_ADDR_SOC);
            #if 0
            //���SOCΪ0������ص�ѹ�ߣ����жϵ��������
            if((((vl_soc != 0xFFFF))
                &&(vl_soc == 0)
                &&((vl_cell_numbers*380) <= vl_bat_voltage))//���赥�ڵ����3.2VΪ����
                ||(!(0x08&Battery_get_switch_state(0))))//��ؼ�ⲻ��������
            {
                gl_check_bat_err.is_bat_error = 1;
                gl_check_bat_err.is_need_bat_checking = 2;
                gl_check_bat_err.checking_step_timeout_cnt = 0;
                gl_check_bat_err.door_close_delay_cnt = 0;
            }
            //������Źرգ�������Ǻõģ���ص�ѹ�ں��ʷ�Χ�ڣ����������������ж�
            else 
            #endif
                if(
                    (TRUE == master_rs485_is_charger_OK())&&
                (((vl_cell_numbers*360) <= vl_bat_voltage)&&((vl_cell_numbers*400) >= vl_bat_voltage)))
            {
                if(!gl_check_bat_err.door_close_delay_cnt)
                    gl_check_bat_err.door_close_delay_cnt = CHECK_BAT_ERROR_DOOR_CLOSE_DELAY_MAX;
            }
            
        }
    }

    if(gl_check_bat_err.is_need_bat_checking == 1)
    {

        if(
            (0 == ADC_is_CHG_enable())
            ||(FALSE == master_rs485_is_charger_OK())
            ||
            (is_battery_checked_enable_for_detecting_bat(0))
            #ifdef EVENT_TO_STOP_CHARGING_ENABLE
            ||(Battery_get_event_to_stop_chg_flag())
            #endif
            #ifdef CABINET_WARMOR_SUPPLY_BY_CHARGER
            ||(0 != master_rs485_get_charger_warmer_fsm_state()) //���ǳ��ģʽ
            #endif
            )
        {
            // ������򿪵�ؿ��ص����ʱ��
            Battery_check_bat_clear_on_and_error();
            return FALSE;
        }
        else
        {
            Battery_check_bat_error(0);
            return TRUE;
        }
    }

    return FALSE;
}

void Battery_check_bat_error(unsigned char bms_index)
{
    unsigned short vl_reg_value;
    unsigned short vl_charger_voltage;
    unsigned short vl_bms_state_reg, vl_bms_ctrl_reg;
    vl_bms_state_reg = Battery_get_switch_state(bms_index);
    vl_bms_ctrl_reg = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_CTRL);
    switch(gl_check_bat_err.checking_step)
    {
        case 0:
            
        gl_check_bat_err.checking_step_timeout_cnt = CHECK_BAT_ERROR_STEP_TIMEOUT_CNT_MAX;
        
        /*��ȡ���STATE�Ĵ���Bit3��־λ��Ϊ0��ʾ���û�м�⵽���������ж�Ϊ��ػ���ʧЧ��*/
        if(!(0x08 & vl_bms_state_reg))// 1--G2��Ч��0--G2��Ч
        {
            #if 0
            //��ؼ�ⲻ�����������ܴ򿪳�ŵ�ܺʹ���
            gl_check_bat_err.is_bat_error |= 0x01; 
            gl_check_bat_err.is_need_bat_checking = 2;
            gl_check_bat_err.checking_step_timeout_cnt = 0;
            #else
            //�Ѿ��ŵ���ĵط�����ػ���
            gl_check_bat_err.is_need_bat_checking = 2;
            gl_check_bat_err.checking_step_timeout_cnt = 0;
            #endif
        }
        gl_check_bat_err.checking_step++;

        //Battery_clear_switch_state(bms_index, BMS_CTRL_BIT_CHG_SWITCH| BMS_CTRL_BIT_SUPPLY_SWITCH|BMS_CTRL_BIT_PRESUPPLY_SWITCH);
        //������ر�
        master_rs485_close_charger_by_cmd();
        gl_check_bat_err.comm_counter_cnt = CHECK_BAT_ERROR_WAIT_FOR_CHARGER_VOLTAGE_DELAY_MAX;//�ȴ�20��

        break;
        case 1:
        /*
                            �رյ�س�ŵ�MOS��Ԥ�ŵ�MOS����������Ը��أ�����4S��
                            a. �ڼ��⵽" Ԥ������·����"��־��λ���ж�Ϊ���Ԥ�ŵ��·���ϣ�MOS��·����/�������ϣ���
                            b. �ڼ��⵽�зŵ��������"�ŵ�MOSʧЧ"��־��λ���ж�Ϊ��طŵ�MOSʧЧ����·���ϣ���
                            */
        if(((BMS_CTRL_BIT_CHG_SWITCH| BMS_CTRL_BIT_SUPPLY_SWITCH|BMS_CTRL_BIT_PRESUPPLY_SWITCH) & vl_bms_state_reg)&&
            ((BMS_CTRL_BIT_CHG_SWITCH| BMS_CTRL_BIT_SUPPLY_SWITCH|BMS_CTRL_BIT_PRESUPPLY_SWITCH) & vl_bms_ctrl_reg))
        {
            //������ر�
            master_rs485_close_charger_by_cmd(); 
            if(gl_check_bat_err.checking_step_timeout_cnt < (CHECK_BAT_ERROR_STEP_TIMEOUT_CNT_MAX - 6000))
            {
                Battery_clear_switch_state(bms_index, BMS_CTRL_BIT_CHG_SWITCH| BMS_CTRL_BIT_SUPPLY_SWITCH|BMS_CTRL_BIT_PRESUPPLY_SWITCH);            
            }
            gl_check_bat_err.comm_counter_cnt = CHECK_BAT_ERROR_WAIT_FOR_CHARGER_VOLTAGE_DELAY_MAX;//�ȴ�20��
        }
        else if((main_is_charger_id_enable())||(main_is_charger_id_enable()))
        {
            //������ر�
            master_rs485_close_charger_by_cmd(); 
        }
        else if(gl_check_bat_err.comm_counter_cnt == 0)
        {
            if(TRUE == master_rs485_is_cmd_list_empty()) //������������û��������Ҫ����
            {
                gl_check_bat_err.checking_step_timeout_cnt = CHECK_BAT_ERROR_STEP_TIMEOUT_CNT_MAX;
                //��ȡ
                vl_charger_voltage = master_rs485_get_V_detected_by_charger();
                vl_reg_value = Battery_get_cell_numbers(bms_index);
                vl_reg_value *= 18;
                if(vl_charger_voltage >= vl_reg_value)// ������ڵ�о��ѹ��1.8V���ʹ���
                {
                    gl_check_bat_err.is_bat_error |= 0x02; 
                    gl_check_bat_err.is_need_bat_checking = 2;
                    gl_check_bat_err.checking_step_timeout_cnt = 0;
                    Battery_save_bat_reg_state_when_detect_bat_error(bms_index);
                }
                gl_check_bat_err.checking_step++;
            }
        }
        break;
        case 2:
        /* ��Ԥ�ŵ�MOS�����ȴ�������ɡ�
                            û�м�⵽�ŵ��������жϵ��Ԥ�ŵ��·���ϣ�MOS�޷���������
                            */
        if(((BMS_CTRL_BIT_PRESUPPLY_SWITCH) & vl_bms_state_reg) != 
                    (BMS_CTRL_BIT_PRESUPPLY_SWITCH))
        {
            Battery_set_switch_state(bms_index, BMS_CTRL_BIT_PRESUPPLY_SWITCH);
            //������ر�
            master_rs485_close_charger_by_cmd();
            gl_check_bat_err.comm_counter_cnt = CHECK_BAT_ERROR_WAIT_FOR_CHARGER_VOLTAGE_DELAY_MAX;//�ȴ�20��
        }
        else if((main_is_charger_id_enable())||(main_is_charger_id_enable()))
        {
            //������ر�
            master_rs485_close_charger_by_cmd(); 
        }
        else if(gl_check_bat_err.comm_counter_cnt == 0)
        {
            if(TRUE == master_rs485_is_cmd_list_empty()) //������������û��������Ҫ����
            {
                gl_check_bat_err.checking_step_timeout_cnt = CHECK_BAT_ERROR_STEP_TIMEOUT_CNT_MAX;
                //��ȡ
                vl_charger_voltage = master_rs485_get_V_detected_by_charger();
                vl_reg_value = Battery_get_voltage(bms_index);
                vl_reg_value /= 10;
                //����������ѹС�ڵ�ص�ѹ��5V����Ϊ�ǹ���
                if((vl_charger_voltage + 50) < (vl_reg_value))
                {
                    gl_check_bat_err.is_bat_error |= 0x04; 
                    gl_check_bat_err.is_need_bat_checking = 2;
                    gl_check_bat_err.checking_step_timeout_cnt = 0;
                    Battery_save_bat_reg_state_when_detect_bat_error(bms_index);

                    gl_check_bat_err.bat_connected_faild_error_cnt = BAT_CONNECTED_FAILD_ERROR_CNT_MAX;
                }
                gl_check_bat_err.checking_step++;
            }
        }
        break;
        case 3:
        /*�ر�Ԥ�ŵ�MOS���򿪷ŵ�MOS�����ȴ�������ɡ�
                            û�м�⵽�ŵ��������жϵ�طŵ�MOSʧЧ���޷���������
                            */
        if(( BMS_CTRL_BIT_PRESUPPLY_SWITCH)&vl_bms_state_reg)
        {
            Battery_clear_switch_state(bms_index, BMS_CTRL_BIT_PRESUPPLY_SWITCH);
        }
        else if(((BMS_CTRL_BIT_SUPPLY_SWITCH) & vl_bms_state_reg) != 
                    (BMS_CTRL_BIT_SUPPLY_SWITCH))
        {
            Battery_set_switch_state(bms_index, BMS_CTRL_BIT_SUPPLY_SWITCH);
            //������ر�
            master_rs485_close_charger_by_cmd();
            gl_check_bat_err.comm_counter_cnt = CHECK_BAT_ERROR_WAIT_FOR_CHARGER_VOLTAGE_DELAY_MAX;//�ȴ�20��
        }
        else if((main_is_charger_id_enable())||(main_is_charger_id_enable()))
        {
            //������ر�
            master_rs485_close_charger_by_cmd(); 
        }
        else if(gl_check_bat_err.comm_counter_cnt == 0)
        {
            if(TRUE == master_rs485_is_cmd_list_empty()) //������������û��������Ҫ����
            {
                gl_check_bat_err.checking_step_timeout_cnt = CHECK_BAT_ERROR_STEP_TIMEOUT_CNT_MAX;
                //��ȡ
                vl_charger_voltage = master_rs485_get_V_detected_by_charger();
                vl_reg_value = Battery_get_voltage(bms_index);
                vl_reg_value /= 10;
                //����������ѹС�ڵ�ص�ѹ��5V����Ϊ�ǹ���
                vl_charger_voltage += 50;
                if((vl_charger_voltage) < (vl_reg_value))
                {
                    gl_check_bat_err.is_bat_error |= 0x08; 
                    gl_check_bat_err.is_need_bat_checking = 2;
                    gl_check_bat_err.checking_step_timeout_cnt = 0;
                    Battery_save_bat_reg_state_when_detect_bat_error(bms_index);
                }
                gl_check_bat_err.checking_step++;
            }
        }
        break;
        case 4:
        /*�رշŵ�MOS���Ͽ����Ը��أ����ȴ�������ɡ�
                �ٽ������������ָ����ѹ������
                ���ȴ�������������������4s��
                �ڼ��⵽�г���������"���MOSʧЧ"��־��λ���ж�Ϊ��س��MOSʧЧ����·���ϣ���
                            */
        if(( BMS_CTRL_BIT_SUPPLY_SWITCH)&vl_bms_state_reg)
        {
            Battery_clear_switch_state(bms_index, BMS_CTRL_BIT_SUPPLY_SWITCH);
            //�������
            //master_rs485_open_charger_by_cmd();
            gl_check_bat_err.comm_counter_cnt = CHECK_BAT_ERROR_WAIT_FOR_CHARGER_VOLTAGE_DELAY_MAX;//�ȴ�20��
        }
        else if((!master_rs485_is_charge_en())||(!main_is_charger_id_enable()))
        {
            //�������
            master_rs485_open_charger_by_cmd();
        }
        else if(gl_check_bat_err.comm_counter_cnt == 0)
        {
            if(TRUE == master_rs485_is_cmd_list_empty()) //������������û��������Ҫ����
            {                   
                gl_check_bat_err.checking_step_timeout_cnt = CHECK_BAT_ERROR_STEP_TIMEOUT_CNT_MAX;
                vl_reg_value = Battery_get_TCurrent(bms_index);
                //if((vl_reg_value==0)||(vl_reg_value > 30000))
                if(vl_reg_value > 30000)
                {
                    gl_check_bat_err.is_bat_error |= 0x10;
                    gl_check_bat_err.is_need_bat_checking = 2;
                    gl_check_bat_err.checking_step_timeout_cnt = 0;
                    Battery_save_bat_reg_state_when_detect_bat_error(bms_index);
                }
                gl_check_bat_err.checking_step++;

                //������ر�
                master_rs485_close_charger_by_cmd();
            }
        }
        break;
        case 5:
        /*�򿪳��MOS�����ȴ�������ɡ�
                            û�м�⵽����������жϵ�س��MOSʧЧ���޷���������
                            */

        //�ȴ�6�룬�ȳ�����ر����
        if(gl_check_bat_err.checking_step_timeout_cnt > (CHECK_BAT_ERROR_STEP_TIMEOUT_CNT_MAX - 6000))
        {
            gl_check_bat_err.comm_counter_cnt = CHECK_BAT_ERROR_WAIT_FOR_CHARGER_VOLTAGE_DELAY_MAX + 20000;//�ȴ�
            break;
        }
        
        if(((BMS_CTRL_BIT_CHG_SWITCH) & vl_bms_state_reg) != 
                    (BMS_CTRL_BIT_CHG_SWITCH))
        {
            Battery_set_switch_state(bms_index, BMS_CTRL_BIT_CHG_SWITCH);

            //�������
            //master_rs485_open_charger_by_cmd();
            gl_check_bat_err.comm_counter_cnt = CHECK_BAT_ERROR_WAIT_FOR_CHARGER_VOLTAGE_DELAY_MAX + 20000;//�ȴ�
        }
        else if((!master_rs485_is_charge_en())||(!main_is_charger_id_enable()))
        {
            //�������
            master_rs485_open_charger_by_cmd();
            gl_check_bat_err.comm_counter_cnt = CHECK_BAT_ERROR_WAIT_FOR_CHARGER_VOLTAGE_DELAY_MAX + 20000;//�ȴ�
        }
        else if(gl_check_bat_err.comm_counter_cnt == 0)
        {
            if(TRUE == master_rs485_is_cmd_list_empty()) //������������û��������Ҫ����
            {                                        
                vl_reg_value = Battery_get_TCurrent(bms_index);
                //if((vl_reg_value==0)||(vl_reg_value < 30000))
                if(vl_reg_value <= 30000)
                {
                    gl_check_bat_err.is_bat_error |= 0x20;
                    gl_check_bat_err.is_need_bat_checking = 2;
                    gl_check_bat_err.checking_step_timeout_cnt = 0;
                    Battery_save_bat_reg_state_when_detect_bat_error(bms_index);
                }
                gl_check_bat_err.checking_step++;

                gl_check_bat_err.is_need_bat_checking = 2; //ֹͣ�����
                gl_check_bat_err.checking_step_timeout_cnt = 0;
            }
        }
        break; 
        default:
            //gl_check_bat_err.is_need_bat_checking = 0;
            break;
    }
}
#endif

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
                    &&(0 == is_battery_error(vl_bms_index, BATTERY_ERROR_OPTION_IGNORE_NONE))
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
                    &&(0 == is_battery_error(vl_bms_index, BATTERY_ERROR_OPTION_IGNORE_NONE))
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

unsigned short Battery_get_switch_state(unsigned char bms_index)
{
    //��ŵ綼��ʱ������
    unsigned short vl_ctrl_value;

    #if 0
    unsigned short vl_pos;
    
    vl_pos = MM_get_reg_addr_position_in_table(ENUM_REG_ADDR_STATE);
    vl_ctrl_value = ((gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_H<<8) + 
        gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_L);
    #else
    
    vl_ctrl_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_STATE);
    #endif
    return vl_ctrl_value;
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


void Battery_set_switch_state(unsigned char bms_index, unsigned short on)
{
    unsigned short vl_pos, vl_ctrl_value;
    unsigned char i;
    //������������ڲ�ѯ���ƼĴ��������˳�
    for(i = 0; i < NFC_READER_COUNT_MAX; i++)
    {
        if(gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_R_CTRL])
            return;
    }

    if((sl_bat_poll_param.poll_allways_en_delay_cnt == 0)&&
        (FALSE == Battery_is_charge_enable_by_BatState(bms_index)))
    {        
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

    vl_ctrl_value |= on;

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

    //��Ϊ����ǳ��ĳ��������κ�ʱ��֪ͨ������
    vl_ctrl_value |= BMS_STATE_BIT_CHG_IN;

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
void Battery_set_reg_need_flashed(unsigned char bms_index, unsigned char flag)
{
    gl_bms_info_p[bms_index]->reg_value_is_need_flashed |= flag;
}

bool Battery_is_reg_need_flashed(unsigned char bms_index, unsigned char flag)
{
    return (gl_bms_info_p[bms_index]->reg_value_is_need_flashed & flag?TRUE:FALSE);
}

void Battery_set_reg_valid_flag(unsigned char bms_index, unsigned char flag)
{
    gl_bms_info_p[bms_index]->reg_value_ready |= flag;
    gl_bms_info_p[bms_index]->reg_value_is_need_flashed &= (~flag);

    #ifdef EVENT_TO_STOP_CHARGING_ENABLE
    //3.       ���ʱ���κε�ش������¶ȴ���Ԥ��ֵ��80�ȣ�
    if((0 == bms_index)&&(BMS_READONLY_DATA1_FLAG_BIT == flag))
    {
        //��ȡ�������¶�
        if(80 <= Battery_get_bat_temp(bms_index))
        {
            //Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_HIGH_TEMP);
            if((FALSE == Battery_is_event_to_stop_chg_flag_be_set(ENUM_SCR_OF_STOP_FLAG_HIGH_TEMP))&&
                (sl_src_of_stop_flag_high_temp_delay_cnt == 0))
            {
                sl_src_of_stop_flag_high_temp_delay_cnt = EVENT_TO_STOP_CHG_FLAG_DELAY_MAX;
            }
        }
        else
        {
            sl_src_of_stop_flag_high_temp_delay_cnt = 0;
        }
    }

    //4.       ���Ԥ����ʧ��
    if((0 == bms_index)&&(BMS_READONLY_DATA1_FLAG_BIT == flag))
    {
        unsigned short vl_reg1_value, vl_reg2_value, vl_reg3_value;
        vl_reg1_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_DEVFT1);
        vl_reg2_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_OPFT1);
        vl_reg3_value = Battery_get_switch_state(bms_index);
        if(((1<<3)&vl_reg3_value)&& // Bit3    1--G2��Ч��0--G2��Ч
            (((1<<6)&vl_reg1_value)// Bit6   1--Ԥ������·����
            ||((1<<15)&vl_reg2_value))) // Bit15   1--Ԥ����ʧ���ⲿ����/��·
        {
            //Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_PRE_CURRENT);
            if((FALSE == Battery_is_event_to_stop_chg_flag_be_set(ENUM_SCR_OF_STOP_FLAG_PRE_CURRENT))&&
                (sl_src_of_stop_flag_pre_current_delay_cnt == 0))
            {
                sl_src_of_stop_flag_pre_current_delay_cnt = EVENT_TO_STOP_CHG_FLAG_DELAY_MAX;
            }
        }
        else
        {
            sl_src_of_stop_flag_pre_current_delay_cnt = 0;
        }
    }
    
    if((0 == bms_index)
        &&(BMS_READONLY_DATA1_FLAG_BIT & gl_bms_info_p[bms_index]->reg_value_ready)
        &&(BMS_READONLY_DATA2_FLAG_BIT & gl_bms_info_p[bms_index]->reg_value_ready)
        &&(0 != Battery_get_voltage(bms_index))
        )
    {
        unsigned short vl_bat_voltage;
        unsigned short vl_soc;
        unsigned char vl_cell_numbers;
        unsigned short vl_reg_value;
        vl_cell_numbers = Battery_get_cell_numbers(bms_index);
        vl_bat_voltage = Battery_get_voltage(bms_index);
        vl_soc = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_SOC);
        //6.    ������������ʱ������⵽��ص�ѹ����26V��13S����32V��16S)��
        //�����������
        if(16 != vl_cell_numbers)
        {
            if(2600 >= vl_bat_voltage)
            {
                //Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_LOW_VOLTAGE);
                if((FALSE == Battery_is_event_to_stop_chg_flag_be_set(ENUM_SCR_OF_STOP_FLAG_LOW_VOLTAGE))&&
                    (sl_src_of_stop_flag_low_voltage_delay_cnt == 0))
                {
                    sl_src_of_stop_flag_low_voltage_delay_cnt = EVENT_TO_STOP_CHG_FLAG_DELAY_MAX;
                }
            }
            else
            {
                sl_src_of_stop_flag_low_voltage_delay_cnt = 0;
            }
        }
        else
        {
            if(3200 >= vl_bat_voltage)
            {
                //Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_LOW_VOLTAGE);
                if((FALSE == Battery_is_event_to_stop_chg_flag_be_set(ENUM_SCR_OF_STOP_FLAG_LOW_VOLTAGE))&&
                    (sl_src_of_stop_flag_low_voltage_delay_cnt == 0))
                {
                    sl_src_of_stop_flag_low_voltage_delay_cnt = EVENT_TO_STOP_CHG_FLAG_DELAY_MAX;
                }
            }
            else
            {
                sl_src_of_stop_flag_low_voltage_delay_cnt = 0;
            }
        }
#if 0
        //���SOCΪ0������ص�ѹ��(3.8*��о����)V���ϣ����жϵ��������
        if((((vl_soc != 0xFFFF))
            &&(vl_soc == 0)
            &&((vl_cell_numbers*380) <= vl_bat_voltage))//���赥�ڵ����3.8VΪ����
            )//��ؼ�ⲻ��������
        {
            //Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_SOC_ERROR);
            if((FALSE == Battery_is_event_to_stop_chg_flag_be_set(ENUM_SCR_OF_STOP_FLAG_SOC_ERROR))&&
                (sl_src_of_stop_flag_soc_error_delay_cnt == 0))
            {
                sl_src_of_stop_flag_soc_error_delay_cnt = EVENT_TO_STOP_CHG_FLAG_DELAY_MAX;
            }
        }
        else
        {
            sl_src_of_stop_flag_soc_error_delay_cnt = 0;
        }

        
        //��UVP���ϣ����ǵ��SOC��30%���ϣ����ߵ�ص�ѹ��(3.8*��о����)V����
        vl_reg_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_OPFT1);
        vl_cell_numbers = Battery_get_cell_numbers(bms_index);
        if((vl_reg_value & ((1<<1)|(1<<2)))
            &&(((vl_cell_numbers*380) <= vl_bat_voltage)
            ||((vl_soc != 0xFFFF)&&(vl_soc >= 300))))
        {
            //Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_UVP_ERROR);
            if((FALSE == Battery_is_event_to_stop_chg_flag_be_set(ENUM_SCR_OF_STOP_FLAG_UVP_ERROR))&&
                (sl_src_of_stop_flag_uvp_error_delay_cnt == 0))
            {
                sl_src_of_stop_flag_uvp_error_delay_cnt = EVENT_TO_STOP_CHG_FLAG_DELAY_MAX;
            }
        }
        else
        {
            sl_src_of_stop_flag_uvp_error_delay_cnt = 0;
        }
#else
        vl_reg_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_OPFT1);
        vl_cell_numbers = Battery_get_cell_numbers(bms_index);
#endif
        //���ͬʱ��UVP��OVP����
        if((vl_reg_value & (1<<0)) && (vl_reg_value & ((1<<1)|(1<<2))))
        {
            //Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_UVP_OVP_ERROR);
            if((FALSE == Battery_is_event_to_stop_chg_flag_be_set(ENUM_SCR_OF_STOP_FLAG_UVP_OVP_ERROR))&&
                (sl_src_of_stop_flag_uvp_ovp_error_delay_cnt == 0))
            {
                sl_src_of_stop_flag_uvp_ovp_error_delay_cnt = EVENT_TO_STOP_CHG_FLAG_DELAY_MAX;
            }
        }
        else
        {
            sl_src_of_stop_flag_uvp_ovp_error_delay_cnt= 0;
        }

        //���ͬʱ�����º͵��¹���
        if((vl_reg_value &((1<<6)|(1<<7)|(1<<10)|(1<<11)|(1<<12)))&&
            (vl_reg_value &((1<<8)|(1<<9))))
        {
            //Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_UTP_OTP_ERROR);
            if((FALSE == Battery_is_event_to_stop_chg_flag_be_set(ENUM_SCR_OF_STOP_FLAG_UTP_OTP_ERROR))&&
                (sl_src_of_stop_flag_utp_otp_error_delay_cnt == 0))
            {
                sl_src_of_stop_flag_utp_otp_error_delay_cnt = EVENT_TO_STOP_CHG_FLAG_DELAY_MAX;
            }
        }
        else
        {
            sl_src_of_stop_flag_utp_otp_error_delay_cnt = 0;
        }

        //��ص�о����С��2V
        vl_reg_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_LVOLT);
        if((vl_reg_value != 0xFFFF) && (vl_reg_value < 2000))
        {
            //Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_CELL_VOLTAGE_LOW);
            if((FALSE == Battery_is_event_to_stop_chg_flag_be_set(ENUM_SCR_OF_STOP_FLAG_CELL_VOLTAGE_LOW))&&
                (sl_src_of_stop_flag_cell_voltage_low_delay_cnt == 0))
            {
                sl_src_of_stop_flag_cell_voltage_low_delay_cnt = EVENT_TO_STOP_CHG_FLAG_DELAY_MAX;
            }
        }
        else
        {
            sl_src_of_stop_flag_cell_voltage_low_delay_cnt = 0;
        }

        /*
            30�����жϵ�ز���磬�жϷ����ǣ�
            SOCС�ڽ��ֵ����90%�����Ƿ��г�����(����50mA)��
            �ù��Ϲ��ൽDamage bit2�ı�־�����ǲ���رյ�غͳ������һ���ָ���磬����������
             */
        Battery_bat_chg_current_false_process(bms_index);
        
    }
    
    if((0 == bms_index)
        &&(BMS_READONLY_DATA1_FLAG_BIT & gl_bms_info_p[bms_index]->reg_value_ready)
        )
    {
        unsigned short vl_bat_current;
        vl_bat_current = Battery_get_TCurrent(bms_index);
        //14. ��������500mA���ϣ�����ʱ�䳬��10�롣�رճ�硣
        if((0 != vl_bat_current) && (vl_bat_current <= (30000 - CHARGER_CURR_BAT_CURR_DIFF_MAX)))
        {
            if(0 == gl_event_to_stop_chg_param.bat_current_out_counter)
            {
                gl_event_to_stop_chg_param.bat_current_out_counter = gl_delay_cnt_test;
            }
            
            //�����Ƚ�
            if((gl_delay_cnt_test - gl_event_to_stop_chg_param.bat_current_out_counter) >= 10000)//10��
            {
                Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_CURRENT_OUT);
            }
        }

        //��ػ�����ⲻ������
        if(!(0x08&Battery_get_switch_state(bms_index)))
        {
            //gl_event_to_stop_chg_param.hall_miss_status_clear_delay_cnt = 600000;
            //Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_HALL_OFF);
            if((FALSE == Battery_is_event_to_stop_chg_flag_be_set(ENUM_SCR_OF_STOP_FLAG_HALL_OFF))&&
                (sl_src_of_stop_flag_hall_off_delay_cnt == 0))
            {
                sl_src_of_stop_flag_hall_off_delay_cnt = EVENT_TO_STOP_CHG_FLAG_DELAY_MAX;
            }
        }
        else if(((1<<ENUM_SCR_OF_STOP_FLAG_HALL_OFF) == Battery_get_src_of_stop_flag())
            &&(gl_event_to_stop_chg_param.hall_miss_status_clear_delay_cnt == 0))
        {
            Battery_clear_event_to_stop_chg_flag();
            sl_src_of_stop_flag_hall_off_delay_cnt = 0; 
        }
        else
        {
            sl_src_of_stop_flag_hall_off_delay_cnt = 0;
        }
    }    
    
    #ifndef BAT_WARMOR_BY_SELF_ENABLE
    
    if((0 == bms_index)
        &&(BMS_READONLY_DATA1_FLAG_BIT & gl_bms_info_p[bms_index]->reg_value_ready)
        &&(BMS_READONLY_DATA2_FLAG_BIT & gl_bms_info_p[bms_index]->reg_value_ready))
    {
        unsigned short vl_reg_value;
        unsigned short vl_max_chg_voltage;

        /*
            7.    PMS��ص���ڲ���ѹ������⵽��ص�ѹ�ﵽ������ѹ�󣬳�������δ���ͣ�
                    ����ʵ�����������һ�£�ͬʱ��ص�ѹ��Ȼ���ӣ���ʱ��Ͽ�������Ĺ���

                    ͨ���Ƚ�(�������ĳ���ѹ+ 500mV) �͵��ʵ�ʵĵ�ѹ���ж�����Ĺ��ϡ�
            */
        vl_reg_value = Battery_get_voltage(bms_index);
        vl_max_chg_voltage = Battery_get_max_chg_voltage(bms_index);
        if((0 != vl_reg_value)
            &&(0 != vl_max_chg_voltage)
            &&(vl_reg_value >= ((BAT_VOLTAGE_FULL_VALUE_OFFSET_MAX/10) + vl_max_chg_voltage*10)))
        {
            //Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_MAX_VOLTAGE_EXCEEDED);
            if((FALSE == Battery_is_event_to_stop_chg_flag_be_set(ENUM_SCR_OF_STOP_FLAG_MAX_VOLTAGE_EXCEEDED))&&
                (sl_src_of_stop_flag_max_voltage_exceeded_delay_cnt == 0))
            {
                sl_src_of_stop_flag_max_voltage_exceeded_delay_cnt = EVENT_TO_STOP_CHG_FLAG_DELAY_MAX;
            }
        }
        else
        {
            sl_src_of_stop_flag_max_voltage_exceeded_delay_cnt = 0;
        }
        
    }
    #endif
    #endif

    if((0 == bms_index)
        &&(BMS_READONLY_DATA1_FLAG_BIT & gl_bms_info_p[bms_index]->reg_value_ready)
        &&(BMS_READONLY_DATA2_FLAG_BIT & gl_bms_info_p[bms_index]->reg_value_ready)
        )
    {
        Battery_bat_fault_debounce_process();

        
        #ifdef BAT_TEMP_CHARGER_ENABLE
        if(Battery_get_bat_temp_value_reflash_flag())
        {
            Battery_set_bat_temp_value_when_plus_in();
            Battery_low_temp_param_reset();
        }
        #endif

        #ifdef PMS_AUTO_INCREASE_SETTING_VOLTAGE_ENABLE
        {
            unsigned short vl_tmp;
            vl_tmp = Battery_get_TCurrent(bms_index);
            if(vl_tmp > 30000)
            {
                vl_tmp -= 30000;
            }
            else
            {
                vl_tmp = 0;
            }
            
            Battery_charger_setting_voltage_offset_process(Battery_get_voltage(bms_index),
                                                            vl_tmp);
        }
        #endif
    }

    #ifdef EVENT_TO_STOP_CHARGING_ENABLE
    if((0 == bms_index)
        &&(BMS_READONLY_DATA1_FLAG_BIT & gl_bms_info_p[bms_index]->reg_value_ready)
        &&(BMS_READONLY_DATA2_FLAG_BIT & gl_bms_info_p[bms_index]->reg_value_ready)
        &&(FALSE == Battery_is_event_to_stop_chg_flag_be_set(ENUM_SCR_OF_STOP_FLAG_CELL_V_DIFF_VALUE_EXCEEDED))
        )
    {
        unsigned short vl_HVOLT;
        unsigned short vl_LVOLT;
        
        vl_HVOLT = Battery_get_reg_value(bms_index,ENUM_REG_ADDR_HVOLT);
        vl_LVOLT = Battery_get_reg_value(bms_index,ENUM_REG_ADDR_LVOLT);
        if((vl_HVOLT > 3800)&& // �����о��ߵ�ѹ����3.8V
            (vl_HVOLT != 0xFFFF)&&
            (vl_LVOLT != 0xFFFF))
        {
            if((vl_HVOLT > vl_LVOLT)&&
                ((vl_HVOLT - vl_LVOLT) > 100)) // 100mV
            {
                if(sl_src_of_stop_flag_cell_v_diff_value_exceeded_delay_cnt == 0)
                {
                    //������������Ϊ10��
                    sl_src_of_stop_flag_cell_v_diff_value_exceeded_delay_cnt = 10000;//EVENT_TO_STOP_CHG_FLAG_DELAY_MAX;
                }
            }
            else
            {
                sl_src_of_stop_flag_cell_v_diff_value_exceeded_delay_cnt = 0;
            }
        }
    }
    #endif
}

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
/*
���³�����
1�������񣬵���¶ȣ�t��0�����ֱ�ӳ�磻

2�������񣬵�����¶ȣ�-2��t��0����ػص�2�����ϳ�磨����2�ȣ���
���ǻص�2��ά��0Сʱ����ʼ��硣����0.1C�ĵ������20���ӣ�
�ٸ��ݵ�ص���������硣

3�������񣬵�����¶ȣ�-4��t��-2����ػص�4�����ϳ�磨����4�ȣ���
���ǻص�2��ά��1Сʱ����ʼ��硣
����0.1C�ĵ������20���ӣ��ٸ��ݵ�ص���������硣

4�������񣬵�����¶ȣ�-6��t��-4����ػص�6�����ϳ�磨����6�ȣ���
���ǻص�4��ά��1Сʱ����ʼ��硣
����0.1C�ĵ������20���ӣ��ٸ��ݵ�ص���������硣

5�������񣬵�����¶ȣ�-8��t��-6����ػص�8�����ϳ�磨����8�ȣ���
���ǻص�4��ά��2Сʱ����ʼ��硣
����0.1C�ĵ������20���ӣ��ٸ��ݵ�ص���������硣

6�������񣬵�����¶ȣ�-10��t��-8����ػص�8�������ҵȴ�0.5Сʱ��ʼ��磨����8�ȣ���
���ǻص�4��ά��3Сʱ����ʼ��硣
����0.1C�ĵ������20���ӣ��ٸ��ݵ�ص���������硣

7�������񣬵�����¶ȣ�t��-10����ػص�8�������ҵȴ�1Сʱ��ʼ��磨����8�ȣ���
���ǻص�4��ά��4Сʱ����ʼ��硣
����0.1C�ĵ������20���ӣ��ٸ��ݵ�ص���������硣

*/
#ifdef BAT_TEMP_CHARGER_ENABLE

/*
{
    short low_temp; // 0.1��Ϊ��λ
    unsigned int low_temp_delay_cnt; // 1mSΪ��λ
    short high_temp;// 0.1��Ϊ��λ
    unsigned int high_temp_delay_cnt;// 1mSΪ��λ
    unsigned int low_current_delay_count;// 1mSΪ��λ
}

*/
static st_bat_low_temp_ctrl_param gl_bat_low_temp_ctrl_param[6] = 
#if 1
{
{20, 10, 20, 20, 10},
{20, (1000*60*60), 40, 10, (1000*60*20)},
{40, (1000*60*60), 60, 10, (1000*60*20)},
{40, (1000*60*60*2), 80, 10, (1000*60*20)},
{40, (1000*60*60*3), 80, (1000*60*30), (1000*60*20)},
{40, (1000*60*60*4), 80, (1000*60*60), (1000*60*20)}
}
#else
{
{20, 10, 20, 20, (1000*60*2)},
{20, (1000*60*5), 40, 10, (1000*60*2)},
{40, (1000*60*6), 60, 10, (1000*60*2)},
{40, (1000*60*7), 80, 10, (1000*60*2)},
{40, (1000*60*8), 80, (1000*60*3), (1000*60*2)},
{40, (1000*60*9), 80, (1000*60*6), (1000*60*2)}
}

#endif
;


unsigned char Battery_is_bat_chg_enable_by_temp(void)
{
    //�Ƿ����������ù���"���ݵ���¶Ⱦ����Ƿ���ͳ�����"
    if(main_get_temp_charge_enable_flag())
    {
        //����
        return gl_bat_low_temp_param.is_bat_chg_enable;
    }
    else
    {
        //������ù��ܣ������������������
        return 2; // 0, �������磬1������ǰ��С������磬2�������������
    }
}

void Battery_set_bat_temp_value_reflash_flag(void)
{
    gl_bat_low_temp_param.is_need_to_reflash_temp = 1;
}

unsigned char Battery_get_bat_temp_value_reflash_flag(void)
{
    return gl_bat_low_temp_param.is_need_to_reflash_temp;
}


//��¼�����ص�����¶ȣ��ز���ʱ�����
void Battery_set_bat_temp_value_when_plus_in(void)
{
    gl_bat_low_temp_param.temp_value = Battery_get_bat_lowest_temp(0);
    gl_bat_low_temp_param.is_need_to_reflash_temp = 0;
}

short Battery_get_bat_temp_value_when_plus_in(void)
{
    return gl_bat_low_temp_param.temp_value;
}


//��ȡ��ǰ��ص�����¶�
short Battery_get_bat_current_lower_temp(void)
{
    return Battery_get_bat_lowest_temp(0);
}

void Battery_clear_counter_of_temp(void)
{
    gl_bat_low_temp_param.delay_counter_1 = 0;
    gl_bat_low_temp_param.delay_counter_2 = 0;
    gl_bat_low_temp_param.delay_counter_charge = 0;
}

void Battery_low_temp_param_reset(void)
{
    gl_bat_low_temp_param.is_bat_chg_enable = 0;
    Battery_clear_counter_of_temp();
}

void Battery_bat_low_temp_process(void)
{
    //�����񣬵���¶ȣ�t��0�����ֱ�ӳ�磻
    if(0 <= gl_bat_low_temp_param.temp_value)
    {
        gl_bat_low_temp_param.is_bat_chg_enable = 2;
    }
    else if((-20 <= gl_bat_low_temp_param.temp_value) && (gl_bat_low_temp_param.temp_value < 0))
    {
        /*                    
                2�������񣬵�����¶ȣ�-2��t��0����ػص�2�����ϳ�磨����2�ȣ���
                ���ǻص�2��ά��0Сʱ����ʼ��硣����0.1C�ĵ������20���ӣ�
                �ٸ��ݵ�ص���������硣
                */
        #if 0
        if((gl_bat_low_temp_param.is_bat_chg_enable == 0)&&(2 <= Battery_get_bat_current_lower_temp()))
        {
            gl_bat_low_temp_param.is_bat_chg_enable = 1;
            gl_bat_low_temp_param.delay_counter_charge = (1000*60*20);
        }

        //if(2 <= Battery_get_bat_current_lower_temp())
        //{
            
        //}

        if((gl_bat_low_temp_param.is_bat_chg_enable == 1)&&( 0 == gl_bat_low_temp_param.delay_counter_charge))
        {
            gl_bat_low_temp_param.is_bat_chg_enable = 2;
        }
        #else
        Battery_bat_low_temp_ctrl_calc(&gl_bat_low_temp_ctrl_param[0]);
        #endif
    }
    else if((-40 <= gl_bat_low_temp_param.temp_value) && (gl_bat_low_temp_param.temp_value < -20))
    {
        /*
                3�������񣬵�����¶ȣ�-4��t��-2����ػص�4�����ϳ�磨����4�ȣ���
                ���ǻص�2��ά��1Сʱ����ʼ��硣
                ����0.1C�ĵ������20���ӣ��ٸ��ݵ�ص���������硣
                */
        #if 0
        if((gl_bat_low_temp_param.is_bat_chg_enable == 0) && (4 <= Battery_get_bat_current_lower_temp()))
        {
            gl_bat_low_temp_param.is_bat_chg_enable = 1;
            if(0 == gl_bat_low_temp_param.delay_counter_charge)
            {
                gl_bat_low_temp_param.delay_counter_charge = 1;
            }

            if(0 == gl_bat_low_temp_param.delay_counter_1)
            {
                gl_bat_low_temp_param.delay_counter_1= 1;
            }
        }

        if((gl_bat_low_temp_param.is_bat_chg_enable == 0)&&(2 <= Battery_get_bat_current_lower_temp()))
        {
            if(0 == gl_bat_low_temp_param.delay_counter_2)
            {
                gl_bat_low_temp_param.delay_counter_2 = (1000*60*60);
            }
        }

        
        if((gl_bat_low_temp_param.is_bat_chg_enable == 1)&&((1000*60*20) <= gl_bat_low_temp_param.delay_counter_charge))
        {
            gl_bat_low_temp_param.is_bat_chg_enable = 2;
        }
        #else
        Battery_bat_low_temp_ctrl_calc(&gl_bat_low_temp_ctrl_param[1]);
        #endif
    }
    else if((-60 <= gl_bat_low_temp_param.temp_value) && (gl_bat_low_temp_param.temp_value < -40))
    {
        /*
            4�������񣬵�����¶ȣ�-6��t��-4����ػص�6�����ϳ�磨����6�ȣ���
            ���ǻص�4��ά��1Сʱ����ʼ��硣
            ����0.1C�ĵ������20���ӣ��ٸ��ݵ�ص���������硣
        */
        #if 0
        if((gl_bat_low_temp_param.is_bat_chg_enable == 0) && (6 <= Battery_get_bat_current_lower_temp()))
        {
            gl_bat_low_temp_param.is_bat_chg_enable = 1;
            gl_bat_low_temp_param.delay_counter_charge = 0;
        }

        if((gl_bat_low_temp_param.is_bat_chg_enable == 0)&&(4 <= Battery_get_bat_current_lower_temp()))
        {
            if((1000*60*60) <= gl_bat_low_temp_param.delay_counter_2)
            {
                gl_bat_low_temp_param.is_bat_chg_enable = 1;
                gl_bat_low_temp_param.delay_counter_charge = 0;
            }
        }

        if((gl_bat_low_temp_param.is_bat_chg_enable == 1)&&((1000*60*20) <= gl_bat_low_temp_param.delay_counter_charge))
        {
            gl_bat_low_temp_param.is_bat_chg_enable = 2;
        }
        #else
        Battery_bat_low_temp_ctrl_calc(&gl_bat_low_temp_ctrl_param[2]);
        #endif
    }
    else if((-80 <= gl_bat_low_temp_param.temp_value) && (gl_bat_low_temp_param.temp_value < -60))
    {
        /*
        
            5�������񣬵�����¶ȣ�-8��t��-6����ػص�8�����ϳ�磨����8�ȣ���
            ���ǻص�4��ά��2Сʱ����ʼ��硣
            ����0.1C�ĵ������20���ӣ��ٸ��ݵ�ص���������硣
            */
        #if 0
        if((gl_bat_low_temp_param.is_bat_chg_enable == 0) && (8 <= Battery_get_bat_current_lower_temp()))
        {
            gl_bat_low_temp_param.is_bat_chg_enable = 1;
            gl_bat_low_temp_param.delay_counter_charge = 0;
        }

        if((gl_bat_low_temp_param.is_bat_chg_enable == 0)&&(4 <= Battery_get_bat_current_lower_temp()))
        {
            if((1000*60*60*2) <= gl_bat_low_temp_param.delay_counter_2)
            {
                gl_bat_low_temp_param.is_bat_chg_enable = 1;
                gl_bat_low_temp_param.delay_counter_charge = 0;
            }
        }

        if((gl_bat_low_temp_param.is_bat_chg_enable == 1)&&((1000*60*20) <= gl_bat_low_temp_param.delay_counter_charge))
        {
            gl_bat_low_temp_param.is_bat_chg_enable = 2;
        }
        #else
        Battery_bat_low_temp_ctrl_calc(&gl_bat_low_temp_ctrl_param[3]);
        #endif
    }
    else if((-100 <= gl_bat_low_temp_param.temp_value) && (gl_bat_low_temp_param.temp_value < -80))
    {
        /*
            6�������񣬵�����¶ȣ�-10��t��-8����ػص�8�������ҵȴ�0.5Сʱ��ʼ��磨����8�ȣ���
            ���ǻص�4��ά��3Сʱ����ʼ��硣
            ����0.1C�ĵ������20���ӣ��ٸ��ݵ�ص���������硣
        */
        #if 0
        if((gl_bat_low_temp_param.is_bat_chg_enable == 0) && (8 <= Battery_get_bat_current_lower_temp()))
        {
            if((1000*60*30) <= gl_bat_low_temp_param.delay_counter_1)
            {
                gl_bat_low_temp_param.is_bat_chg_enable = 1;
                gl_bat_low_temp_param.delay_counter_charge = 0;
            }
        }

        if((gl_bat_low_temp_param.is_bat_chg_enable == 0)&&(4 <= Battery_get_bat_current_lower_temp()))
        {
            if((1000*60*60*3) <= gl_bat_low_temp_param.delay_counter_2)
            {
                gl_bat_low_temp_param.is_bat_chg_enable = 1;
                gl_bat_low_temp_param.delay_counter_charge = 0;
            }
        }

        if((gl_bat_low_temp_param.is_bat_chg_enable == 1)&&((1000*60*20) <= gl_bat_low_temp_param.delay_counter_charge))
        {
            gl_bat_low_temp_param.is_bat_chg_enable = 2;
        }
        #else
        Battery_bat_low_temp_ctrl_calc(&gl_bat_low_temp_ctrl_param[4]);
        #endif
    }
    else if(gl_bat_low_temp_param.temp_value < -100)
    {
        /*
                
            7�������񣬵�����¶ȣ�t��-10����ػص�8�������ҵȴ�1Сʱ��ʼ��磨����8�ȣ���
            ���ǻص�4��ά��4Сʱ����ʼ��硣
            ����0.1C�ĵ������20���ӣ��ٸ��ݵ�ص���������硣
            */
        #if 0
        if((gl_bat_low_temp_param.is_bat_chg_enable == 0) && (8 <= Battery_get_bat_current_lower_temp()))
        {
            if((1000*60*60) <= gl_bat_low_temp_param.delay_counter_1)
            {
                gl_bat_low_temp_param.is_bat_chg_enable = 1;
                gl_bat_low_temp_param.delay_counter_charge = 0;
            }
        }

        if((gl_bat_low_temp_param.is_bat_chg_enable == 0)&&(4 <= Battery_get_bat_current_lower_temp()))
        {
            if((1000*60*60*4) <= gl_bat_low_temp_param.delay_counter_2)
            {
                gl_bat_low_temp_param.is_bat_chg_enable = 1;
                gl_bat_low_temp_param.delay_counter_charge = 0;
            }
        }

        if((gl_bat_low_temp_param.is_bat_chg_enable == 1)&&((1000*60*20) <= gl_bat_low_temp_param.delay_counter_charge))
        {
            gl_bat_low_temp_param.is_bat_chg_enable = 2;
        }
        #else
        Battery_bat_low_temp_ctrl_calc(&gl_bat_low_temp_ctrl_param[5]);
        #endif
    }
        
}


//����ʱ��������ж�
void Battery_bat_low_temp_ctrl_calc(st_bat_low_temp_ctrl_param *bat_low_temp_ctrl_param)
{
    
    gl_bat_low_temp_param.low_current_delay_count = bat_low_temp_ctrl_param->low_current_delay_count;
    if((gl_bat_low_temp_param.is_bat_chg_enable == 0) && (bat_low_temp_ctrl_param->high_temp <= Battery_get_bat_current_lower_temp()))
    {
        if(0 == gl_bat_low_temp_param.delay_counter_1)
        {
            gl_bat_low_temp_param.delay_counter_1 = bat_low_temp_ctrl_param->high_temp_delay_cnt;//(1000*60*30);
        }
    }

    if((gl_bat_low_temp_param.is_bat_chg_enable == 0)&&(bat_low_temp_ctrl_param->low_temp <= Battery_get_bat_current_lower_temp()))
    {
        if(0 == gl_bat_low_temp_param.delay_counter_2)
        {
            gl_bat_low_temp_param.delay_counter_2 = bat_low_temp_ctrl_param->low_temp_delay_cnt;//(1000*60*60*3);
        }
    }

    if((gl_bat_low_temp_param.is_bat_chg_enable == 1)&&(0 == gl_bat_low_temp_param.delay_counter_charge))
    {
        gl_bat_low_temp_param.is_bat_chg_enable = 2;
    }
}
#endif
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef PMS_AUTO_INCREASE_SETTING_VOLTAGE_ENABLE
//������������ѹ���������⵼�������ѹƫ��ʱ���ʵ����߳���������ѹ��

//��ȡ���ó���������ѹ��ƫ��ֵ
unsigned char Battery_get_charger_setting_voltage_offset(void)
{
    return gl_charger_setting_V_offset_param.voltage_offset_value;
}

GLOBAL_BPROC void Battery_clear_charger_setting_voltage_offset(void)
{
    Battery_charger_setting_voltage_offset_init();
}

void Battery_set_charger_setting_voltage(unsigned short v_value)
{
    gl_charger_setting_V_offset_param.expect_setting_voltage = v_value;
}

void Battery_set_charger_setting_current(unsigned short a_value)
{
    gl_charger_setting_V_offset_param.expect_setting_current = a_value;
}

void Battery_charger_setting_voltage_offset_init(void)
{
    memset((char*)&gl_charger_setting_V_offset_param, 0, sizeof(st_charger_setting_V_offset_param));
}

//bat_voltage: ��λ��0.01V
//bat_current: ��λ��0.01A
void Battery_charger_setting_voltage_offset_process(unsigned short bat_voltage,
                                                    unsigned short bat_current)
{
    unsigned char vl_diff_current_value;
    if(12 >= master_rs485_get_expect_current())// С�ڵ���1.2A
    {
        vl_diff_current_value = 30; //���ﵥλ��0.01A
    }
    else
    {
        vl_diff_current_value = 60; //���ﵥλ��0.01A
    }
    
    if((TRUE == main_is_charger_id_enable())
        &&(TRUE == master_rs485_is_charge_en())
        &&(30000 != Battery_get_TCurrent(0)) //������Ϊ0
        &&((gl_charger_setting_V_offset_param.expect_setting_voltage - bat_voltage) >= 0) //0mV
    &&((gl_charger_setting_V_offset_param.expect_setting_current - bat_current) >= vl_diff_current_value) // 600mA
    )
    {        
        //���ü�ʱʱ��
        if(!gl_charger_setting_V_offset_param.debount_time_cnt)
        {
            gl_charger_setting_V_offset_param.debount_time_cnt = 60000;
        }
    }
    else
    {
        //��ʱ����
        gl_charger_setting_V_offset_param.debount_time_cnt = 0;
    }
}

#endif


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

