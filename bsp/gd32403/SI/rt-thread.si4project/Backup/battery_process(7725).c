

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

extern unsigned char vl_BMS_index; //从0开始
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
            //30分钟后重启
            gl_check_bat_err.bat_step_timeout_err_cnt = 1800000; //30分钟
        }
    }

    if(gl_check_bat_err.bat_step_timeout_err_cnt)
    {
        gl_check_bat_err.bat_step_timeout_err_cnt--;
        if(0 == gl_check_bat_err.bat_step_timeout_err_cnt)
        {
            if(gl_check_bat_err.is_bat_error & 0x40)
            {
                //重启PMS
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
                //再次启动主动检测
                Bttery_check_bat_init();
            }
            else
            {
                gl_check_bat_err.bat_connected_faild_error_cnt_reload_max--;
                if(gl_check_bat_err.bat_connected_faild_error_cnt_reload_max)
                {
                    //gl_check_bat_err.bat_connected_faild_error_cnt = (1000*60*5);
                    //再次启动主动检测
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
            (gl_bms_app_bl_status[0].is_bl_mode))//处于bootloader模式
    {
        gl_pms_param.is_bat_detected = 1;
    }
    else
    {
        gl_pms_param.is_bat_detected = 0;
    }
    
}

/*获取目前pms运行的模式: 
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
            //应该不能存在这种选择
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

    /*检查PMS升级文件是否有效*/

    gl_pms_param.OD_pms_update_file_param[0].OD_update_file_max_subindex = 3;


    /*检查充电器升级文件是否有效*/

    gl_pms_param.OD_charger_update_file_param[0].OD_update_file_max_subindex = 3;
    gl_pms_param.OD_charger_update_file_param[1].OD_update_file_max_subindex = 3;
    gl_pms_param.OD_charger_update_file_param[2].OD_update_file_max_subindex = 3;


    /*检查电池升级文件是否有效*/

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


    // 如果充电器的有效标志有改变，应该轮询一次电池
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
        (sl_bat_poll_param.poll_interval_for_bms_update_cnt)) // 如果收到一个开始升级BMS的命令，则允许查询一段时间
    {
        if(sl_bat_poll_param.poll_inerval_timer_cnt == 0)
        {
            sl_bat_poll_param.poll_inerval_timer_cnt = sl_bat_poll_param.poll_interval_MAX;
        }

        //读取电池信息
        Battery_info_polling_Process(vl_BMS_index);
    }
    #endif
    Battery_chg_hw_ctrl_process();

    //读取电池历史数据
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
            //与电池通讯
            if(MM_set_BMS_index(i))
            {
                if((vl_data_len <= 7)|| // 这时候表示没有要透传的数据
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
            //与电池通讯
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
            
            //设置重启值
            gl_bms_info_p[i]->reg_unit[vl_k].bits16_H = 0x7A;
            gl_bms_info_p[i]->reg_unit[vl_k].bits16_L= 0xB9;
            
            vl_write_reg_10.byte_data = (unsigned char*)&gl_bms_info_p[i]->reg_unit[vl_k];
            //与电池通讯
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
                //存储的控制寄存器数据无效
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
            //与电池通讯
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

            //与电池1通讯
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

            //与电池1通讯
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
            //判断电池版本号是否大于等于35版本
            if(((BMS_DEV_INFO_FLAG_BIT)&Battery_get_reg_valid_flag(bms_index))&&
                (35 <= Battery_get_reg_value(bms_index, ENUM_REG_ADDR_FWREV)))
            {
                //支持寄存器地址322
                vl_read_reg_03.reg_cnt[0] = ((READ_ONLY_REG_TOTAL_SIZE-(READ_ONLY_REG_TOTAL_SIZE/2))>>8)&0xFF;
                vl_read_reg_03.reg_cnt[1] = ((READ_ONLY_REG_TOTAL_SIZE-(READ_ONLY_REG_TOTAL_SIZE/2)))&0xFF;
            }
            else
            {
                vl_read_reg_03.reg_cnt[0] = ((READ_ONLY_REG_TOTAL_SIZE-(READ_ONLY_REG_TOTAL_SIZE/2) - 7)>>8)&0xFF;
                vl_read_reg_03.reg_cnt[1] = ((READ_ONLY_REG_TOTAL_SIZE-(READ_ONLY_REG_TOTAL_SIZE/2) - 7))&0xFF;
            }

            //与电池1通讯
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

            //与电池1通讯
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

            //与电池1通讯
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

            //与电池1通讯
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

            //与电池1通讯
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

            //与电池1通讯
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
            //与电池1通讯
            if(MM_set_BMS_index(i))
            {
                if(0 == gl_bat_history_record_param.is_earliest_param_valid)
                {
                    //读取电池里面的最近记录编号
                    gl_bat_history_record_param.history_record_lastest_index = Battery_get_newest_record_index_number();
                    //读取电池里面的最早记录编号
                    gl_bat_history_record_param.history_record_earliest_index = Battery_get_oldest_record_index_number();
                
                    if(MM_snd_read_history_record_cmd(MM_ADDR_VALUE, 
                        gl_bat_history_record_param.history_record_earliest_index)
                        )
                    {
                        gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_HISTORY_RECORD] = 0;
                        //设置间隔时间3S
                        Batter_history_record_set_nfc_cmd_interval_cnt(1000);
                        return 1;
                    }
                }
                else if(0 == gl_bat_history_record_param.is_lastest_param_valid)
                {
                    //读取电池里面的最近记录编号
                    gl_bat_history_record_param.history_record_lastest_index = Battery_get_newest_record_index_number();
                    //读取电池里面的最早记录编号
                    gl_bat_history_record_param.history_record_earliest_index = Battery_get_oldest_record_index_number();
                    
                    if(MM_snd_read_history_record_cmd(MM_ADDR_VALUE, 
                        gl_bat_history_record_param.history_record_lastest_index)
                        )
                    {
                        gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_HISTORY_RECORD] = 0;
                        //设置间隔时间3S
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
                        //设置间隔时间3S
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

//设置获取电池版本信息的命令
void Battery_cmd_buf_get_version(unsigned char bms_index)
{
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_VERSION] = 1;
}

//单个NFC读卡器是否所有命令已经发送完成
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
//是否所以命令已经发送完成
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
    //开机查询一次电池版本信息
    //if(vl_bat_info_flag[0]==0)
    {
        gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_VERSION] = 1;
    }
    
    //查询BMS 只读信息，从寄存器256到308
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_INFO1] = 1;
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_INFO2] = 1;
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_CTRL] = 1;

    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_PARAM1] = 1;
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_PARAM2] = 1;
 

    //开机查询一次
    {
        gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_USERDATA] = 1;
    }

//    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_BLV] = 1;

    #ifdef BAT_PMS_AUTHORITY_FUNCTION_EN
    //电池双向认证流程
    Authority_process();
    #endif

    //读取电池历史记录数据
//    if((0 == gl_bat_history_record_param.is_earliest_param_valid)
//        ||(0 == gl_bat_history_record_param.is_lastest_param_valid))
//    {
//        gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_HISTORY_RECORD] = 1;
//    }
    
    return ;

}


/*

battery_index: 1，表示电池1； 2，表示电池2
return: 0，表示电压信息不可用；1，表示电池信息有效
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

//就是不能再打开充放电管
unsigned char is_battery_error(unsigned char bms_index, unsigned char option_ignore)
{
    unsigned short vl_value;
    vl_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_OPFT1);
    //屏蔽温度故障
    if(option_ignore == BATTERY_ERROR_OPTION_IGNORE_TEMPERATURE)
    {
        vl_value &= (~(0x7F<<6)); // bit6~bit12 关于温度的故障都忽略
    }
    if(vl_value&(~((1<<15)|(1<<14)|
        //(1<<0)|(1<<1)|(1<<2))))//充电时候不用管
        #ifdef BAT_WARMOR_BY_SELF_ENABLE
        (1<<8)| // Bit8   1--充电温度过低
        #endif
        (1<<1)|(1<<2))))//充电时候不用管//2018-8-21修改，过压故障不能屏蔽
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

//该电池是否允许主动检测电池mos管
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
    //返回存储的指针
    memcpy(p, (unsigned char*)gl_bat_critical_error_reg_state, 12*2);
    return (12*2);
}
#endif

// 设备故障，同时出现过压和欠压，同时出现过温和低温，
//电芯电压低于2V，电池最高温度超过80摄氏度
//有电压，但是SOC是0%
//其他PMS主动检测到的电池硬件故障(暂时没有加进来)
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
            Bit0  过压
            Bit1  初级过放
            Bit2  二级过放
            */
        //同时出现过压和欠压
        return 1;
    }
    if((vl_value&(3<<6))&&(vl_value&(3<<8)))
    {
        /*
                Bit6    充电温度过高
                Bit7    放电温度过高
                Bit8    充电温度过低
                Bit9    放电温度过低
                */
        //同时出现过温和低温
        return 1;
    }

    vl_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_LVOLT);
    //出现小于2V
    if((vl_value != 0xFFFF) && (vl_value < 2000))
        return 1;

    vl_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_HTEMP);
    //温度超过80摄氏度
    if((vl_value != 0xFFFF) && (vl_value > (800+400)))
        return 1;

    //soc是0,但是总电压还是挺高的(电芯电压超过3.8V)
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
比较两电池电压
return : 0，表示两电池电压比较小于0.5v
            1，表示电池1的电压比较高
            2，表示电池2的电压比较高
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
//字节的bit7表示是否允许充电，0,不允许；1,允许
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


//修改NFC读卡器个数时候，需要手动修改分组配置
void Battery_chg_param_init(void)
{
    //暂时按顺序分
    unsigned char i, k;
    for(i = 0; i < CHG_GROUP_INDEX_MAX; i++)
    {
        for(k = 0; k < CHG_PORT_INDEX_MAX; k++)
        {
            gl_chg_group_param.chg_group_index[i].chg_port_index[k] = i+k;
        }
    }
    #ifdef TEST_BAT_V_VIA_CHARGER_EN
    //初始化
    Battery_check_bat_v_via_charger_init();
    //重启默认结束检测
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
    //改为只有一个控制脚，因为只有一个电池仓
    GPIO_PinModeSet(BATTERY_CHG_ENABLE_PORT, BATTERY_CHG_ENABLE_PIN, gpioModePushPull, 0);
    #endif
#endif
    #if (NFC_READER_COUNT_MAX > 6)
    Have to add control pins for Battery Charge.
    #endif
}

//预充电打开
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
    //这里的预充电流由充电器接收到的命令控制

    #endif
}

//充电打开
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
            //打开充电电路，这里可能要加上充电器控制逻辑
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

//充电关闭
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
            //关闭充电电路，这里可能需要加上关充电器的逻辑
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

//找出正在充电的电池，并判断电压，然后选择充电模式
static void Battery_chg_hw_ctrl_process(void)
{
    unsigned char i;
    unsigned short vl_bms_tmp_V = 0;
    unsigned char vl_group_index;

    for(i = 0; i < NFC_READER_COUNT_MAX; i++)
    {
        //找出在线的电池
        //if(is_battery_voltage_OK(i))
        if(
            (ADC_is_CHG_enable())&&
            (master_rs485_is_charger_OK())&&
            ((BMS_READONLY_DATA1_FLAG_BIT|BMS_READONLY_DATA2_FLAG_BIT)&
            Battery_get_reg_valid_flag(i))&&
            (Battery_chg_is_charge_enable(i))
            &&
            (ELOCK_LOCK == ELock_get_status((ENUM_ELOCK_INDEX)i))//如果锁是打开的，则不充电
            )
        {
            //vl_bms_tmp_V = Battery_get_voltage(i);

            vl_group_index = Battery_group_get_group_index_by_bms_index(i);
            if(vl_group_index == 0xFF)
                continue;
            //计算该组中最高电压值，因为充电时候先充最高电压的电池
            vl_bms_tmp_V = Battery_group_get_highest_voltage(vl_group_index);

            if(vl_bms_tmp_V == 0)
            {
                //无效的电压
                //充电关闭
                //Battery_chg_hw_ctrl_off(i);
                continue;
            }
            #if 0
            if(vl_bms_tmp_V < 4800)
            {
                //小于48v, 先开预充
                Battery_prechg_hw_ctrl_on(i);
            }
            else
            {
                //正式充电打开
                Battery_chg_hw_ctrl_on(i);
            }
            #else
            //正式充电打开；因为没有预充电硬件开关(放在充电器中管理)，直接充电
            Battery_chg_hw_ctrl_on(i);
            #endif
        }
        else
        {
            //充电关闭
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
            //充电关闭，等待上位机重新发送允许充电的命令
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
                //再判断是否在线
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

//是否允许充电
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

//搜索到底属于哪个组的
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
    //没有找到
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

        //是否无效号
        if(CHG_IDLE_PORT_INDEX == vl_bms_index)
            continue;

        //找出在线的电池
        if(is_battery_voltage_OK(vl_bms_index))
        {
            vl_bms_tmp_V = Battery_get_voltage(vl_bms_index);
            //找出最高的电压
            if(vl_bms_tmp_V > vl_bms_V)
            {
                vl_bms_V = vl_bms_tmp_V;
            }
        }
    }

    return vl_bms_V;
}

#ifdef TEST_BAT_V_VIA_CHARGER_EN

//初始化，开仓门时候需要初始化
void Battery_check_bat_v_via_charger_init(void)
{
    memset(&gl_check_bat_v, 0, sizeof(st_check_bat_v));
}

//是否不需要检测
bool Battery_check_bat_v_is_off(void)
{
    return (gl_check_bat_v.is_need_bat_v_checking == 0)?TRUE:FALSE;
}


//是否正在检测电池
bool Battery_check_bat_v_is_on(void)
{
    return (gl_check_bat_v.is_need_bat_v_checking == 1)?TRUE:FALSE;
}

//是否已经检测过了
bool Battery_check_bat_v_is_end(void)
{
    return (gl_check_bat_v.is_need_bat_v_checking == 2)?TRUE:FALSE;
}

//是否检测到电池电压
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
//关仓门时候检测
bool Battery_check_bat_v_process(unsigned char bms_index)
{
    
    unsigned short vl_bms_state_reg, vl_bms_ctrl_reg;

    #ifdef CHECKING_BAT_IN_IO_FLAG_ENABLE
    //没有微动开关时候，充电器检测电池电压的功能也去掉
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
        ()&& //没有欠压
        ()&& //没有温度异常
        (master_rs485_is_charger_OK()) // 充电器正常
        #endif
        )
    {
        gl_check_bat_v.bat_v_charger_comm_timeout_cnt = BATTERY_BAT_V_CHARGER_COMM_TIMEOUT_CNT_MAX; // 1秒
        gl_check_bat_v.bat_v_charger_cmd_cnt = BATTERY_BAT_V_CHARGER_CMD_CNT_MAX;
        gl_check_bat_v.bat_v_charger_check_totol_time_cnt = BATTERY_BAT_V_CHARGER_CHECK_TOTOL_TIME_CNT_MAX;
        gl_check_bat_v.is_need_bat_v_checking = 1;
    }

    if(gl_check_bat_v.is_need_bat_v_checking == 1)
    {
        //是否有磁铁，电池是否有故障，是否有充电器
        if((!(0x08&Battery_get_switch_state(0)))||
            (is_battery_error(0, BATTERY_ERROR_OPTION_IGNORE_NONE)))
        {
            //结束检测
            gl_check_bat_v.is_need_bat_v_checking = 2;

            gl_check_bat_v.is_bat_v_valid = 0;
            gl_check_bat_v.bat_v_value = 0;

            return FALSE;
        }
        else
        {
            //开始检测

            //打开电池预放
            if((BMS_CTRL_BIT_PRESUPPLY_SWITCH != ((BMS_CTRL_BIT_PRESUPPLY_SWITCH) & vl_bms_state_reg))&&
            (BMS_CTRL_BIT_PRESUPPLY_SWITCH != ((BMS_CTRL_BIT_PRESUPPLY_SWITCH) & vl_bms_ctrl_reg)))
            {
                Battery_set_switch_state(bms_index, BMS_CTRL_BIT_PRESUPPLY_SWITCH);
                //充电器关闭
                master_rs485_close_charger_by_cmd();
                gl_check_bat_v.bat_v_charger_comm_timeout_cnt = BATTERY_BAT_V_CHARGER_COMM_TIMEOUT_CNT_MAX; // 1秒
                gl_check_bat_v.bat_v_charger_cmd_cnt = BATTERY_BAT_V_CHARGER_CMD_CNT_MAX;

                if(!gl_check_bat_v.bat_v_charger_check_totol_time_cnt)
                {
                    //结束检测
                    gl_check_bat_v.is_need_bat_v_checking = 2;

                    gl_check_bat_v.is_bat_v_valid = 0;
                    gl_check_bat_v.bat_v_value = 0;

                    return FALSE;
                }
                
                return TRUE;
            }
            else
            {
                if(((gl_check_bat_v.bat_v_charger_comm_timeout_cnt == 0)&&(TRUE == master_rs485_is_cmd_list_empty()))|| //计时超时
                    (gl_check_bat_v.bat_v_charger_cmd_cnt == 0)) //轮询充电器次数超
                {
                    //结束检测
                    gl_check_bat_v.is_need_bat_v_checking = 2;

                    //设置结果
                    if(400 < master_rs485_get_V_detected_by_charger()) //电池的电压输出超过20V
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
                    //充电器关闭
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
    //返回存储的指针
    memcpy(p, (unsigned char*)gl_detect_bat_error_reg_state, 18*2);
    return (18*2);
}

bool Battery_check_bat_is_on(void)
{
    return (gl_check_bat_err.is_need_bat_checking == 1)?TRUE:FALSE;
}

//电池是否已经检测结束了
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


//设置开始检测电池的参数
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

//关门后延迟30秒再进行电池故障检测
#define CHECK_BAT_ERROR_DOOR_CLOSE_DELAY_MAX 30000
//电池开关或者充电器开关动作后，等待充电器检测电压时间
#define CHECK_BAT_ERROR_WAIT_FOR_CHARGER_VOLTAGE_DELAY_MAX 40000
//检测时候，每个检测进度(step)的最大检测时间
#define CHECK_BAT_ERROR_STEP_TIMEOUT_CNT_MAX 300000 // 5分钟

bool Battery_check_bat_process(void)
{
    //bool sl_elock_status;
    unsigned short vl_bat_voltage;
    //unsigned short vl_soc;
    unsigned char vl_cell_numbers;
    /*如果仓门由开变关，并且电池电压处于某电压范围内，
    则开始倒计时，启动检测电池故障*/
    if((gl_check_bat_err.bat_process_cnt == 0)&&
        (gl_check_bat_err.is_need_bat_checking == 0)
    #ifdef BAT_ERROR_CHECK_DELAY_AFTER_BAT_UPDATE
    &&(0 == sl_bat_error_check_delay_after_bat_update_cnt)
    #endif
    )
    {
        gl_check_bat_err.bat_process_cnt = 5000;// 每5秒进入一次这里

        
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
            &&(0 == master_rs485_get_charger_warmer_fsm_state()) //充电模式
            #endif
        )
        {
            //sl_elock_status_old = sl_elock_status;
            
            //vl_soc = Battery_get_reg_value(0, ENUM_REG_ADDR_SOC);
            #if 0
            //如果SOC为0，而电池电压高，则判断电池有问题
            if((((vl_soc != 0xFFFF))
                &&(vl_soc == 0)
                &&((vl_cell_numbers*380) <= vl_bat_voltage))//假设单节电池是3.2V为界线
                ||(!(0x08&Battery_get_switch_state(0))))//电池检测不到磁铁，
            {
                gl_check_bat_err.is_bat_error = 1;
                gl_check_bat_err.is_need_bat_checking = 2;
                gl_check_bat_err.checking_step_timeout_cnt = 0;
                gl_check_bat_err.door_close_delay_cnt = 0;
            }
            //如果仓门关闭，充电器是好的，电池电压在合适范围内，则再做其他故障判断
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
            ||(0 != master_rs485_get_charger_warmer_fsm_state()) //不是充电模式
            #endif
            )
        {
            // 不允许打开电池开关的情况时候
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
        
        /*读取电池STATE寄存器Bit3标志位，为0表示电池没有检测到磁铁，可判断为电池霍尔失效。*/
        if(!(0x08 & vl_bms_state_reg))// 1--G2有效；0--G2无效
        {
            #if 0
            //电池检测不到磁铁，不能打开充放电管和磁铁
            gl_check_bat_err.is_bat_error |= 0x01; 
            gl_check_bat_err.is_need_bat_checking = 2;
            gl_check_bat_err.checking_step_timeout_cnt = 0;
            #else
            //已经放到别的地方检测电池霍尔
            gl_check_bat_err.is_need_bat_checking = 2;
            gl_check_bat_err.checking_step_timeout_cnt = 0;
            #endif
        }
        gl_check_bat_err.checking_step++;

        //Battery_clear_switch_state(bms_index, BMS_CTRL_BIT_CHG_SWITCH| BMS_CTRL_BIT_SUPPLY_SWITCH|BMS_CTRL_BIT_PRESUPPLY_SWITCH);
        //充电器关闭
        master_rs485_close_charger_by_cmd();
        gl_check_bat_err.comm_counter_cnt = CHECK_BAT_ERROR_WAIT_FOR_CHARGER_VOLTAGE_DELAY_MAX;//等待20秒

        break;
        case 1:
        /*
                            关闭电池充放电MOS及预放电MOS，并接入测试负载，持续4S。
                            a. 期间检测到" 预启动电路故障"标志置位可判断为电池预放电电路故障（MOS短路故障/采样故障）。
                            b. 期间检测到有放电电流或者"放电MOS失效"标志置位可判断为电池放电MOS失效（短路故障）。
                            */
        if(((BMS_CTRL_BIT_CHG_SWITCH| BMS_CTRL_BIT_SUPPLY_SWITCH|BMS_CTRL_BIT_PRESUPPLY_SWITCH) & vl_bms_state_reg)&&
            ((BMS_CTRL_BIT_CHG_SWITCH| BMS_CTRL_BIT_SUPPLY_SWITCH|BMS_CTRL_BIT_PRESUPPLY_SWITCH) & vl_bms_ctrl_reg))
        {
            //充电器关闭
            master_rs485_close_charger_by_cmd(); 
            if(gl_check_bat_err.checking_step_timeout_cnt < (CHECK_BAT_ERROR_STEP_TIMEOUT_CNT_MAX - 6000))
            {
                Battery_clear_switch_state(bms_index, BMS_CTRL_BIT_CHG_SWITCH| BMS_CTRL_BIT_SUPPLY_SWITCH|BMS_CTRL_BIT_PRESUPPLY_SWITCH);            
            }
            gl_check_bat_err.comm_counter_cnt = CHECK_BAT_ERROR_WAIT_FOR_CHARGER_VOLTAGE_DELAY_MAX;//等待20秒
        }
        else if((main_is_charger_id_enable())||(main_is_charger_id_enable()))
        {
            //充电器关闭
            master_rs485_close_charger_by_cmd(); 
        }
        else if(gl_check_bat_err.comm_counter_cnt == 0)
        {
            if(TRUE == master_rs485_is_cmd_list_empty()) //充电器命令缓存中没有命令需要发送
            {
                gl_check_bat_err.checking_step_timeout_cnt = CHECK_BAT_ERROR_STEP_TIMEOUT_CNT_MAX;
                //读取
                vl_charger_voltage = master_rs485_get_V_detected_by_charger();
                vl_reg_value = Battery_get_cell_numbers(bms_index);
                vl_reg_value *= 18;
                if(vl_charger_voltage >= vl_reg_value)// 假设大于电芯电压是1.8V，就错误
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
        /* 打开预放电MOS，并等待操作完成。
                            没有检测到放电电流则可判断电池预放电电路故障（MOS无法开启）。
                            */
        if(((BMS_CTRL_BIT_PRESUPPLY_SWITCH) & vl_bms_state_reg) != 
                    (BMS_CTRL_BIT_PRESUPPLY_SWITCH))
        {
            Battery_set_switch_state(bms_index, BMS_CTRL_BIT_PRESUPPLY_SWITCH);
            //充电器关闭
            master_rs485_close_charger_by_cmd();
            gl_check_bat_err.comm_counter_cnt = CHECK_BAT_ERROR_WAIT_FOR_CHARGER_VOLTAGE_DELAY_MAX;//等待20秒
        }
        else if((main_is_charger_id_enable())||(main_is_charger_id_enable()))
        {
            //充电器关闭
            master_rs485_close_charger_by_cmd(); 
        }
        else if(gl_check_bat_err.comm_counter_cnt == 0)
        {
            if(TRUE == master_rs485_is_cmd_list_empty()) //充电器命令缓存中没有命令需要发送
            {
                gl_check_bat_err.checking_step_timeout_cnt = CHECK_BAT_ERROR_STEP_TIMEOUT_CNT_MAX;
                //读取
                vl_charger_voltage = master_rs485_get_V_detected_by_charger();
                vl_reg_value = Battery_get_voltage(bms_index);
                vl_reg_value /= 10;
                //如果充电器电压小于电池电压减5V，认为是故障
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
        /*关闭预放电MOS，打开放电MOS，并等待操作完成。
                            没有检测到放电电流则可判断电池放电MOS失效（无法开启）。
                            */
        if(( BMS_CTRL_BIT_PRESUPPLY_SWITCH)&vl_bms_state_reg)
        {
            Battery_clear_switch_state(bms_index, BMS_CTRL_BIT_PRESUPPLY_SWITCH);
        }
        else if(((BMS_CTRL_BIT_SUPPLY_SWITCH) & vl_bms_state_reg) != 
                    (BMS_CTRL_BIT_SUPPLY_SWITCH))
        {
            Battery_set_switch_state(bms_index, BMS_CTRL_BIT_SUPPLY_SWITCH);
            //充电器关闭
            master_rs485_close_charger_by_cmd();
            gl_check_bat_err.comm_counter_cnt = CHECK_BAT_ERROR_WAIT_FOR_CHARGER_VOLTAGE_DELAY_MAX;//等待20秒
        }
        else if((main_is_charger_id_enable())||(main_is_charger_id_enable()))
        {
            //充电器关闭
            master_rs485_close_charger_by_cmd(); 
        }
        else if(gl_check_bat_err.comm_counter_cnt == 0)
        {
            if(TRUE == master_rs485_is_cmd_list_empty()) //充电器命令缓存中没有命令需要发送
            {
                gl_check_bat_err.checking_step_timeout_cnt = CHECK_BAT_ERROR_STEP_TIMEOUT_CNT_MAX;
                //读取
                vl_charger_voltage = master_rs485_get_V_detected_by_charger();
                vl_reg_value = Battery_get_voltage(bms_index);
                vl_reg_value /= 10;
                //如果充电器电压小于电池电压减5V，认为是故障
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
        /*关闭放电MOS，断开测试负载，并等待操作完成。
                再接入充电器并输出指定电压电流，
                并等待充电器输出建立。持续4s。
                期间检测到有充电电流或者"充电MOS失效"标志置位可判断为电池充电MOS失效（短路故障）。
                            */
        if(( BMS_CTRL_BIT_SUPPLY_SWITCH)&vl_bms_state_reg)
        {
            Battery_clear_switch_state(bms_index, BMS_CTRL_BIT_SUPPLY_SWITCH);
            //充电器打开
            //master_rs485_open_charger_by_cmd();
            gl_check_bat_err.comm_counter_cnt = CHECK_BAT_ERROR_WAIT_FOR_CHARGER_VOLTAGE_DELAY_MAX;//等待20秒
        }
        else if((!master_rs485_is_charge_en())||(!main_is_charger_id_enable()))
        {
            //充电器打开
            master_rs485_open_charger_by_cmd();
        }
        else if(gl_check_bat_err.comm_counter_cnt == 0)
        {
            if(TRUE == master_rs485_is_cmd_list_empty()) //充电器命令缓存中没有命令需要发送
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

                //充电器关闭
                master_rs485_close_charger_by_cmd();
            }
        }
        break;
        case 5:
        /*打开充电MOS，并等待操作完成。
                            没有检测到充电电流则可判断电池充电MOS失效（无法开启）。
                            */

        //等待6秒，等充电器关闭完成
        if(gl_check_bat_err.checking_step_timeout_cnt > (CHECK_BAT_ERROR_STEP_TIMEOUT_CNT_MAX - 6000))
        {
            gl_check_bat_err.comm_counter_cnt = CHECK_BAT_ERROR_WAIT_FOR_CHARGER_VOLTAGE_DELAY_MAX + 20000;//等待
            break;
        }
        
        if(((BMS_CTRL_BIT_CHG_SWITCH) & vl_bms_state_reg) != 
                    (BMS_CTRL_BIT_CHG_SWITCH))
        {
            Battery_set_switch_state(bms_index, BMS_CTRL_BIT_CHG_SWITCH);

            //充电器打开
            //master_rs485_open_charger_by_cmd();
            gl_check_bat_err.comm_counter_cnt = CHECK_BAT_ERROR_WAIT_FOR_CHARGER_VOLTAGE_DELAY_MAX + 20000;//等待
        }
        else if((!master_rs485_is_charge_en())||(!main_is_charger_id_enable()))
        {
            //充电器打开
            master_rs485_open_charger_by_cmd();
            gl_check_bat_err.comm_counter_cnt = CHECK_BAT_ERROR_WAIT_FOR_CHARGER_VOLTAGE_DELAY_MAX + 20000;//等待
        }
        else if(gl_check_bat_err.comm_counter_cnt == 0)
        {
            if(TRUE == master_rs485_is_cmd_list_empty()) //充电器命令缓存中没有命令需要发送
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

                gl_check_bat_err.is_need_bat_checking = 2; //停止检测电池
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
    //用来保存全局计时器计数。如果需要打开BAT开关就reload一次。
    static unsigned int gl_bat_switch_close_from_open_state_cnt_old;
    unsigned int vl_tmp_int_value;
    static unsigned char sl_elock_status_old = 0; //0，表示打开；1表示关闭
    
    //开机上电延迟处理下面流程
    if(gl_Battery_delay_process_cnt>2500)
        return;

    //如果NFC正在发送命令，则不处理下面流程
    for(i = 0; i < CHG_PORT_INDEX_MAX; i++)
    {
        vl_bms_index = CHG_PORT_INDEX_MASK & gl_chg_group_param.chg_group_index[group_index].chg_port_index[i];

        //是否无效号
        if(CHG_IDLE_PORT_INDEX == vl_bms_index)
            continue;
        
        if(gl_bms_info_p[vl_bms_index]->waiting_cmd_ack)
            return ;
    }


    vl_bms_V = Battery_group_get_highest_voltage(group_index);
    for(i = 0; i < CHG_PORT_INDEX_MAX; i++)
    {
        vl_bms_index = CHG_PORT_INDEX_MASK & gl_chg_group_param.chg_group_index[group_index].chg_port_index[i];


        //是否无效号
        if(CHG_IDLE_PORT_INDEX == vl_bms_index)
            continue;

        //仓门由开变关时候，处理防盗的策略
        {
            static unsigned char sl_elock_status_old_for_checking = 1; //0，表示打开；1表示关闭
            if((sl_elock_status_old_for_checking != 0)&&
                (ELOCK_OPEN== ELock_get_status((ENUM_ELOCK_INDEX)vl_bms_index)))
            {
                sl_elock_status_old_for_checking = 0;
                memset(&gl_bat_theft_prevention_param, 0, sizeof(st_bat_theft_prevention_param));
            }
            else if(!sl_elock_status_old_for_checking)//当仓门刚关闭时候
            {
                sl_elock_status_old_for_checking = 1;

                #if 0 // 不好判断真没有电池的情况，屏蔽该功能。2019-7-23
                //检测电池是否在线，如果不在线，不再检测电池连接器连接状态，
                //默认上传连接器没有连接好的状态。
                if((0 == gl_bat_theft_prevention_param.connector_default_state)&&
                    (0 == is_battery_voltage_OK(vl_bms_index)))
                {
                    gl_bat_theft_prevention_param.connector_default_state = 1;
                }
                #endif
                //检测微动开关是否到位，如果已经到位，不再检测微动开关是否到位，
                //默认上传微动开关没有到位的状态。
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

        //仓门打开的时候，清除step timeout定时器，不再自动重启PMS
        if(gl_check_bat_err.bat_step_timeout_err_cnt && 
            (ELOCK_OPEN== ELock_get_status((ENUM_ELOCK_INDEX)vl_bms_index)))
        {
            gl_check_bat_err.bat_step_timeout_err_cnt = 0;
        }

        //找出在线的电池
        if(is_battery_voltage_OK(vl_bms_index))
        {
            unsigned short vl_bms_reg_value;

            //仓门关锁后，在检测微动开关期间电池霍尔检测不到磁铁。
            //如果中间出现掉霍尔信号或者偶尔掉霍尔信号，
            //不再检测电池连接器连接状态，默认上传连接器没有连接好的状态。
            if((0 == gl_bat_theft_prevention_param.connector_default_state) &&
                (ELOCK_LOCK == ELock_get_status((ENUM_ELOCK_INDEX)vl_bms_index))
                #ifdef UNMASK_CODE
                &&(SM_is_closing())
                #endif
                &&(!((1<<3) & (Battery_get_switch_state(vl_bms_index)))))
            {
                //默认没有连接好连接器
                gl_bat_theft_prevention_param.connector_default_state = 1;
            }

            #ifdef TEST_BAT_V_VIA_CHARGER_EN
            if(
                (ELOCK_LOCK== ELock_get_status((ENUM_ELOCK_INDEX)vl_bms_index)) 
                #ifdef CHECKING_BAT_IN_IO_FLAG_ENABLE
                // 如果有微动开关，需要检测到微动开关到位后才启动电池连接器是否插好的检测
                &&(((TRUE == Battery_is_checking_mode_on_bat_in_by_HW()) && (TRUE == Battery_is_bat_in_via_IO_switch()))
                    ||(FALSE == Battery_is_checking_mode_on_bat_in_by_HW()))
                #endif
                && (Battery_check_bat_v_process(vl_bms_index))
                )
            {
                //正在进行关舱门检测电池过程
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
                    //正在处理电池故障
                    continue;
                }
            }
            else if(FALSE == slave_rs485_is_detect_bat_error_enable())
            {
                //不允许主动检测。则清空检测的状态
                Battery_check_bat_clear_on_and_error();
            }
            #endif
            
            //函数里面有判断是否需要间隔轮询
            #if 0
            if((sl_bat_poll_param.poll_allways_en_delay_cnt == 0)&&
                (is_battery_error(vl_bms_index, BATTERY_ERROR_OPTION_IGNORE_NONE)|| // 电池有不能再打开充放电管的错误
                (FALSE == Battery_is_charge_enable_by_BatState(vl_bms_index))|| // 电池不让充电
                (FALSE == master_rs485_is_charger_OK())|| // 充电器错误
                (FALSE == Battery_chg_is_charge_enable(vl_bms_index))//上位机命令禁止充电时候
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
                    &&(0 != Battery_get_max_chg_current(vl_bms_index)) // 2019-3-30 当电池需求电流为0时候，关闭电池充放电管。
                    //&&(0 == (BMS_STATE_BIT_FULLY_CHARGE & vl_bms_ctrl_status))//电池的充满标志没有被置位，2018-8-30  // Bit14   1--充电满标志(充电状态有效)
                    &&(1000 > Battery_get_reg_value(vl_bms_index, ENUM_REG_ADDR_SOC)) // SOC小于100%才能充电
                    #ifdef CABINET_WARMOR_SUPPLY_BY_CHARGER
                    &&(0 == master_rs485_get_charger_warmer_fsm_state()) //处于充电模式
                    #endif
                )))
            #endif
            {
                sl_bat_poll_param.poll_interval_MAX = BAT_POLL_INTERVAL_MAX;
                //获取单体电池最低电压
                vl_bms_reg_value = Battery_get_reg_value(vl_bms_index, ENUM_REG_ADDR_LVOLT);

                // 间歇性轮询
                sl_bat_poll_param.is_poll_allways_en_flag = 0;
                //是否有单体电池电压过低，小于2.7V
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
                else if(vl_bms_reg_value > 3900) // 电芯电压大于3.9V后，保存一直轮询
                {
                    //总是轮询
                    sl_bat_poll_param.is_poll_allways_en_flag = 1;
                    sl_bat_poll_param.poll_interval_MAX = 0;
                }
            }
            else
            {
                //总是轮询
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

            //fault故障延迟上报功能，清零参数
            Battery_bat_fault_debounce_param_init();
        }

        #ifdef BAT_WARMOR_BY_SELF_ENABLE
        //电池自加热测试不打开间隔轮询。
        //总是轮询
        sl_bat_poll_param.is_poll_allways_en_flag = 1;
        sl_bat_poll_param.poll_interval_MAX = 0;
        #endif

        if(ELOCK_OPEN== ELock_get_status((ENUM_ELOCK_INDEX)vl_bms_index))
        {
            sl_elock_status_old = 0;
            //总是轮询
            sl_bat_poll_param.is_poll_allways_en_flag = 1;
            sl_bat_poll_param.poll_interval_MAX = 0;
            
            #ifdef TEST_DETECT_BAT_ERROR_EN
            Battery_check_bat_clear_on_and_error();
            #endif

            #ifdef TEST_BAT_V_VIA_CHARGER_EN
            Battery_check_bat_v_via_charger_init();
            #endif

            #ifdef EVENT_TO_STOP_CHARGING_ENABLE
            //清除因事件而停止充电的标志。
            Battery_clear_event_to_stop_chg_flag();
            #endif

            #ifdef PMS_AUTO_INCREASE_SETTING_VOLTAGE_ENABLE
            //开仓门，清除电压偏移量
            Battery_clear_charger_setting_voltage_offset();
            #endif

            //清除不充电故障的标志
            Battery_bat_chg_current_false_flag_clear();

            //fault故障延迟上报功能，清零参数
            Battery_bat_fault_debounce_param_init();

            //充电器过流或者短路故障报警的锁定功能，清零参数
            master_rs485_charger_error_cycle_lock_init();

            master_rs485_set_chg_voltage_max_value_default();
        }
        else if(!sl_elock_status_old)//当仓门刚关闭时候
        {
            sl_elock_status_old = 1;

            #ifdef BAT_TEMP_CHARGER_ENABLE
            //设标记，在读取电池温度那里设置"插入时电池温度值"
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
            //不充电故障的计时，清零。
            Battery_bat_chg_current_false_timer_reset();
        }
        
        //找出在线的电池
        if((is_battery_voltage_OK(vl_bms_index))
            //&&(0 == is_battery_error(vl_bms_index, BATTERY_ERROR_OPTION_IGNORE_NONE))
            )
        {

            vl_bms_ctrl_status = Battery_get_switch_state(vl_bms_index);
            //找出需要充电的电池和不需要充电的电池(0.5V电压比较)
            vl_bms_tmp_V = Battery_get_voltage(vl_bms_index);

            if((vl_bms_V < vl_bms_tmp_V))
                return ;//异常情况，退出
            
            if((vl_bms_V - vl_bms_tmp_V) >= 50)
            {
                //关闭不需要充电的电池
                if((BMS_CTRL_BIT_CHG_SWITCH| BMS_CTRL_BIT_SUPPLY_SWITCH)&vl_bms_ctrl_status)
                {
                    Battery_clear_switch_state(vl_bms_index, BMS_CTRL_BIT_CHG_SWITCH| BMS_CTRL_BIT_SUPPLY_SWITCH);
                }
            }
            else
            {
                //打开需要充电的电池
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
                    &&(0 != Battery_get_max_chg_current(vl_bms_index)) // 2019-3-30 当电池需求电流为0时候，关闭电池充放电管。
                    //&&(0 == (BMS_STATE_BIT_FULLY_CHARGE & vl_bms_ctrl_status))//电池的充满标志没有被置位，2018-8-30  // Bit14   1--充电满标志(充电状态有效)
                    &&(1000 > Battery_get_reg_value(vl_bms_index, ENUM_REG_ADDR_SOC)) // SOC小于100%才能充电
                    #ifdef CABINET_WARMOR_SUPPLY_BY_CHARGER
                    &&(0 == master_rs485_get_charger_warmer_fsm_state()) //处于充电模式
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
                    //不允许充电，充放电关闭
                    if(((BMS_CTRL_BIT_PRESUPPLY_SWITCH | BMS_CTRL_BIT_CHG_SWITCH | BMS_CTRL_BIT_SUPPLY_SWITCH) 
                        & vl_bms_ctrl_status)
                        #ifdef TEST_BAT_V_VIA_CHARGER_EN
                        &&(!Battery_check_bat_v_is_on())
                        #endif
                        )
                    {
                        
                        //先关闭充电器ID才能关闭电池开关
                        if(FALSE == main_is_charger_id_enable())
                        {
                            Battery_clear_switch_state(vl_bms_index, BMS_CTRL_BIT_CHG_SWITCH| BMS_CTRL_BIT_SUPPLY_SWITCH|BMS_CTRL_BIT_PRESUPPLY_SWITCH);
                        }
                        else 
                        {
                            vl_tmp_int_value = gl_delay_cnt_test - gl_bat_switch_close_from_open_state_cnt_old;
                            if(vl_tmp_int_value >= 4000) // 4秒
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
        //处于bootloader模式
        memset((unsigned char*)version_desc, 0, sizeof(st_version_desc));
        version_desc->HW_main_ver = gl_bms_app_bl_status[bms_index].hw_main_version;//硬件主版本
        version_desc->HW_sub_ver = gl_bms_app_bl_status[bms_index].hw_sub_version;//硬件次版本
    }
    else
    {
        vl_tmp = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_HWVER);

        version_desc->HW_main_ver = (vl_tmp>>8)&0xFF;//硬件主版本
        version_desc->HW_sub_ver = vl_tmp&0xFF;//硬件次版本

        vl_tmp = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_FWMSV);
        version_desc->FW_main_ver = (vl_tmp>>8)&0xFF; //固件主版本
        version_desc->FW_sub_ver = vl_tmp&0xFF; //固件次版本

        vl_tmp = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_FWREV);
        version_desc->FW_revised_ver = vl_tmp&0xFF; //固件修正版本

        vl_tmp = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_FWBNH);
        version_desc->FW_build_ver[3] = (vl_tmp>>8)&0xFF;
        version_desc->FW_build_ver[2] = vl_tmp&0xFF;
        
        vl_tmp = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_FWBNL);
        version_desc->FW_build_ver[1] = (vl_tmp>>8)&0xFF;
        version_desc->FW_build_ver[0] = vl_tmp&0xFF;
    }
    return 1;
}

//获取历史记录的最早记录号
unsigned int Battery_get_oldest_record_index_number(void)
{
    unsigned short vl_H;
    unsigned short vl_L;
    vl_H = Battery_get_reg_value(0, ENUM_REG_ADDR_ERECH);
    vl_L = Battery_get_reg_value(0, ENUM_REG_ADDR_ERECL);
    return ((vl_H << 16) + vl_L);
}

//获取历史记录的最近记录号
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
//返回值的精度是0.1V
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

//返回值的精度是0.1A
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

//返回值没有偏移，但是小于0摄氏度的话统一返回0; 单位1 度C
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

//返回值0.1C/bit为单位
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


//返回值是1mA/h为单位
unsigned short Battery_get_bat_rated_cap(unsigned char bms_index)
{
    unsigned short vl_pos;
    vl_pos = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_BCAP);
    if(vl_pos == 0xFFFF)
        vl_pos = 0;
    
    return vl_pos;
}

//返回值是0.1V为单位
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

//return : 0.1%为单位
unsigned short Battery_get_SOC(unsigned char bms_index)
{
    unsigned short vl_pos;
    vl_pos = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_SOC);

    if(vl_pos == 0xFFFF)
        vl_pos = 0;
    
    return vl_pos;
}

//获取电池有多少个电芯串联
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
    //充放电都打开时候才算打开
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

//根据电池内部状态判断是否能充电。该函数只在设置CTRL寄存器前使用
static bool Battery_is_charge_enable_by_BatState(unsigned char bms_index)
{
    unsigned short vl_ctrl_value;
    unsigned short vl_state_value;

    vl_ctrl_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_CTRL);
    vl_state_value = Battery_get_switch_state(bms_index);

    //设置控制寄存器中充电bit的值与状态寄存器中充电bit的值相等，就允许充电
    if(((vl_ctrl_value&BMS_CTRL_BIT_CHG_SWITCH) && (vl_state_value&BMS_STATE_BIT_CHG_SWITCH))||
        ((!(vl_ctrl_value&BMS_CTRL_BIT_CHG_SWITCH)) && (!(vl_state_value&BMS_STATE_BIT_CHG_SWITCH))))
    {
        //总是轮询
        //sl_bat_poll_param.is_poll_allways_en_flag = 1;
        return TRUE;
    }
    // 间歇性轮询
    //sl_bat_poll_param.is_poll_allways_en_flag = 0;
    return FALSE;
}


/*

1--过压
1--初级过放
1--二级过放
1--充电过流
1--初级放电过流
1--二级放电过流
1--充电温度过高
1--放电温度过高
1--充电温度过低
1--放电温度过低
1--充电MOS温度过高
1--放电MOS温度过高
1--连接器温度过高
保留
1--预启动失败超时
1--预启动失败外部重载/短路

*/

//判断电池是否可以放电
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
        //不能放电
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
        //不能放电
        return FALSE;
    }

    vl_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_DEVFT1);
    if(vl_value&(
        (0x7F<<0)
        |(7<<9)
        |(1<<15)
    ))
    {
        //不能放电
        return FALSE;
    }

    vl_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_DEVFT2);
    if(vl_value&(
        (0x3F<<0)
    ))
    {
        //不能放电
        return FALSE;
    }
    return TRUE;
}


void Battery_set_switch_state(unsigned char bms_index, unsigned short on)
{
    unsigned short vl_pos, vl_ctrl_value;
    unsigned char i;
    //如果有命令正在查询控制寄存器，则退出
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
    //如果没有认证电池或者电池认证，则退出
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
        //通知充电插入
        vl_ctrl_value |= BMS_STATE_BIT_CHG_IN;
    }
    else
    {
        //通知充电拔出
        vl_ctrl_value &= (~BMS_STATE_BIT_CHG_IN);
    }

    //因为这个是充电的程序，所以任何时候都通知充电插入
    vl_ctrl_value |= BMS_STATE_BIT_CHG_IN;

    #ifdef BAT_WARMOR_BY_SELF_ENABLE
    vl_ctrl_value |= (1<<14); // 测试电池自动加热功能
    #endif
    
    gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_H = (vl_ctrl_value>>8)&0xff;
    gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_L = (vl_ctrl_value>>0)&0xff;

    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_W_CTRL] = 1;

    //再查询一下
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_INFO1] = 1;
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_INFO2] = 1;
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_CTRL] = 1;
    //用于检测是否在bootloader状态
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_BLV] = 1;

    //读取电池版本号
    #ifndef BAT_PMS_AUTHORITY_FUNCTION_EN
    if(!(BMS_DEV_INFO_FLAG_BIT & Battery_get_reg_valid_flag(bms_index)))
    #endif
    {
        gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_VERSION] = 1;
    }
    #ifdef BAT_PMS_AUTHORITY_FUNCTION_EN
    //电池双向认证流程
    Authority_process();
    #endif
}


void Battery_clear_switch_state(unsigned char bms_index, unsigned short on)
{
    unsigned short vl_pos, vl_ctrl_value;
    unsigned char i;
    //如果有命令正在查询控制寄存器，则退出
    for(i = 0; i < NFC_READER_COUNT_MAX; i++)
    {
        if(gl_cmd_buf[i][ENUM_NFC_CMD_INDEX_R_CTRL])
            return;
    }

    //如果还有命令在自己命令缓存，则退出
    for(i = 0; i < ENUM_NFC_CMD_INDEX_MAX; i++)
    {
        if(gl_cmd_buf[bms_index][i])
            return;
    }

    #ifdef BAT_PMS_AUTHORITY_FUNCTION_EN
    //如果没有认证电池或者电池认证，则退出
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
        //通知充电插入
        vl_ctrl_value |= BMS_STATE_BIT_CHG_IN;
    }
    else
    {
        //通知充电拔出
        vl_ctrl_value &= (~BMS_STATE_BIT_CHG_IN);
    }

    
    #ifdef BAT_WARMOR_BY_SELF_ENABLE
    vl_ctrl_value |= (1<<14); // 测试电池自动加热功能
    #endif
    
    gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_H = (vl_ctrl_value>>8)&0xff;
    gl_bms_info_p[bms_index]->reg_unit[vl_pos].bits16_L = (vl_ctrl_value>>0)&0xff;
    
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_W_CTRL] = 1;
    //再查询一下
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_INFO1] = 1;
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_INFO2] = 1;
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_CTRL] = 1;
    //用于检测是否在bootloader状态
    gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_BLV] = 1;

    //读取电池版本号
    #ifndef BAT_PMS_AUTHORITY_FUNCTION_EN
    if(!(BMS_DEV_INFO_FLAG_BIT & Battery_get_reg_valid_flag(bms_index)))
    #endif
    {
        gl_cmd_buf[bms_index][ENUM_NFC_CMD_INDEX_R_VERSION] = 1;
    }

    #ifdef BAT_PMS_AUTHORITY_FUNCTION_EN
    //电池双向认证流程
    Authority_process();
    #endif
}

void Battery_clear_reg_valid_flag(unsigned char bms_index)
{
    gl_bms_info_p[bms_index]->reg_value_ready = 0;
    //if(gl_elock_clear_bms_flag_cnt[bms_index])
    //    Elock_clear_bms_flag(bms_index);
    #ifdef BAT_PMS_AUTHORITY_FUNCTION_EN
    //没有电池，复位电池认证参数
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
    //3.       充电时，任何电池传感器温度大于预期值（80度）
    if((0 == bms_index)&&(BMS_READONLY_DATA1_FLAG_BIT == flag))
    {
        //获取电池最高温度
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

    //4.       电池预启动失败
    if((0 == bms_index)&&(BMS_READONLY_DATA1_FLAG_BIT == flag))
    {
        unsigned short vl_reg1_value, vl_reg2_value, vl_reg3_value;
        vl_reg1_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_DEVFT1);
        vl_reg2_value = Battery_get_reg_value(bms_index, ENUM_REG_ADDR_OPFT1);
        vl_reg3_value = Battery_get_switch_state(bms_index);
        if(((1<<3)&vl_reg3_value)&& // Bit3    1--G2有效；0--G2无效
            (((1<<6)&vl_reg1_value)// Bit6   1--预启动电路故障
            ||((1<<15)&vl_reg2_value))) // Bit15   1--预启动失败外部重载/短路
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
        //6.    充电器启动充电时，当检测到电池电压低于26V（13S）和32V（16S)，
        //不能启动充电
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
        //如果SOC为0，而电池电压在(3.8*电芯数量)V以上，则判断电池有问题
        if((((vl_soc != 0xFFFF))
            &&(vl_soc == 0)
            &&((vl_cell_numbers*380) <= vl_bat_voltage))//假设单节电池是3.8V为界线
            )//电池检测不到磁铁，
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

        
        //报UVP故障，但是电池SOC在30%以上，或者电池电压在(3.8*电芯数量)V以上
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
        //电池同时报UVP和OVP故障
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

        //电池同时报高温和低温故障
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

        //电池电芯出现小于2V
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
            30分钟判断电池不充电，判断方法是，
            SOC小于借出值或者90%，看是否有充电电流(大于50mA)。
            该故障归类到Damage bit2的标志。但是不会关闭电池和充电器，一旦恢复充电，故障消除。
             */
        Battery_bat_chg_current_false_process(bms_index);
        
    }
    
    if((0 == bms_index)
        &&(BMS_READONLY_DATA1_FLAG_BIT & gl_bms_info_p[bms_index]->reg_value_ready)
        )
    {
        unsigned short vl_bat_current;
        vl_bat_current = Battery_get_TCurrent(bms_index);
        //14. 电池有输出500mA以上，持续时间超过10秒。关闭充电。
        if((0 != vl_bat_current) && (vl_bat_current <= (30000 - CHARGER_CURR_BAT_CURR_DIFF_MAX)))
        {
            if(0 == gl_event_to_stop_chg_param.bat_current_out_counter)
            {
                gl_event_to_stop_chg_param.bat_current_out_counter = gl_delay_cnt_test;
            }
            
            //计数比较
            if((gl_delay_cnt_test - gl_event_to_stop_chg_param.bat_current_out_counter) >= 10000)//10秒
            {
                Battery_set_event_to_stop_chg_flag(ENUM_SCR_OF_STOP_FLAG_CURRENT_OUT);
            }
        }

        //电池霍尔检测不到磁铁
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
            7.    PMS监控电池内部电压，当检测到电池电压达到充满电压后，充电电流仍未降低，
                    与电池实际需求电流不一致，同时电池电压仍然增加，此时需断开充电器的供电

                    通过比较(电池需求的充电电压+ 500mV) 和电池实际的电压来判断上面的故障。
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
        if((vl_HVOLT > 3800)&& // 单体电芯最高电压大于3.8V
            (vl_HVOLT != 0xFFFF)&&
            (vl_LVOLT != 0xFFFF))
        {
            if((vl_HVOLT > vl_LVOLT)&&
                ((vl_HVOLT - vl_LVOLT) > 100)) // 100mV
            {
                if(sl_src_of_stop_flag_cell_v_diff_value_exceeded_delay_cnt == 0)
                {
                    //这里特殊设置为10秒
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
//去抖
static unsigned int sl_bat_chg_current_disable_debounce_cnt_old = 0;

/*
    30分钟判断电池不充电，判断方法是，
    SOC小于借出值或者90%，看是否有充电电流(大于50mA)。
    该故障归类到Damage bit2的标志。但是不会关闭电池和充电器，一旦恢复充电，故障消除。
     */
static void Battery_bat_chg_current_false_process(unsigned char bms_index)
{
    unsigned short vl_reg_value;

    vl_reg_value = Battery_get_switch_state(bms_index);
    if((vl_reg_value & BMS_STATE_BIT_CHG_SWITCH)&& //电池充电管打开
        (900 > Battery_get_SOC(bms_index)) //SOC小于90%
//        &&
//        ((main_led_get_SOC_full_threshold(0) * 10) > Battery_get_SOC(bms_index)) //SOC小于借出值
        )
    {
        if(//(0 == Battery_get_TCurrent(bms_index))||
            (30005 > Battery_get_TCurrent(bms_index))) // 充电电流小于50mA
        {
            //不能充电了。
            if((gl_delay_cnt_test - sl_bat_chg_current_false_cnt_old) > (30*60*1000))
            {
                sl_bat_chg_current_false_flag = 1;
            }
        }
        else
        {
            //正常充电
            sl_bat_chg_current_false_cnt_old = gl_delay_cnt_test;
            sl_bat_chg_current_false_flag = 0;
        }

        //去抖计时复位
        sl_bat_chg_current_disable_debounce_cnt_old = gl_delay_cnt_test;
    }
    else
    {
        //不需要判断不充电时间

        //去抖，3秒
        if((gl_delay_cnt_test - sl_bat_chg_current_disable_debounce_cnt_old) > 3000)
        {
            sl_bat_chg_current_false_cnt_old = gl_delay_cnt_test;
        }
    }
    
}

//是否长时间充电没有电流
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
//TRUE表示使用硬件开关辅助检测电池插入
static bool sl_bat_checking_mode_flag = FALSE;
void Battery_checking_mode_on_bat_in_by_HW_init(void)
{
    //初始化
    if(0 == GPIO_PinInGet(MICRO_SWITCH_OPEN_TEST_PORT, MICRO_SWITCH_OPEN_TEST_PIN))
    {
        //低电平表示使用硬件开关检测电池插入
        sl_bat_checking_mode_flag = TRUE;
    }
    else
    {
        sl_bat_checking_mode_flag = FALSE;
    }
}

extern bool sl_bat_checking_IO_status;
//通过后盖上的微动开关判断是否电池已经插入
bool Battery_is_bat_in_via_IO_switch(void)
{    
    return sl_bat_checking_IO_status;
}

// 识别是否加了硬件开关来检测电池插入拔出的状态
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
    //返回存储的指针
    memcpy(p, (unsigned char*)gl_bat_event_to_stop_chg_reg_state, 18*2);
    return (18*2);
}
#endif

void Battery_event_to_stop_chg_param_init(void)
{
    memset(&gl_event_to_stop_chg_param, 0, sizeof(st_event_to_stop_chg_param));

    /*
        设置默认关闭检测的故障，如下:
        ENUM_SCR_OF_STOP_FLAG_CELL_V_DIFF_VALUE_EXCEEDED, // 电芯电压差异大，最高电芯电压与最低电芯电压差值大于等于0.1V
    */
    gl_event_to_stop_chg_param.src_of_stop_flag_disable_flag |= (1 << ENUM_SCR_OF_STOP_FLAG_CELL_V_DIFF_VALUE_EXCEEDED);
}

void Battery_set_event_to_stop_chg_flag(enum_src_of_stop_flag src )
{
    #ifdef BAT_ERROR_CHECK_DELAY_AFTER_BAT_UPDATE
    if(sl_bat_error_check_delay_after_bat_update_cnt)
        return;
    #endif

    //被禁止检测的故障，不再设置故障标志
    if(gl_event_to_stop_chg_param.src_of_stop_flag_disable_flag & (1<<src))
    {
        return ;
    }
    
    
    if(ELOCK_OPEN== ELock_get_status((ENUM_ELOCK_INDEX)0))
    {
        //如果仓门开的时候，不设置该故障标志
        return;
    }

    if(ENUM_SCR_OF_STOP_FLAG_CHARGER_OTP_LOCK != src) // 反复过温故障还是允许充电的
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

//设置禁止进行故障事件被动检测的标志
void Battery_set_event_to_stop_chg_flag_disable_flag(unsigned int flag)
{
    gl_event_to_stop_chg_param.src_of_stop_flag_disable_flag |= flag;
}
//清除禁止进行故障事件被动检测的标志
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

//比较电池ID是否有变化
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
//2.  H,M,L级故障，过滤10s再上报给上位机

//按次数和按时间
//计算total_counts, error_counts, total_counts必须大于10, 同时error_counts/total_counts  > 0.8时候
//总时间total_time, total_time必须大于10S

//total_time总是在增加，直到10S

//开门时候清零total_time, total_counts, error_counts;


//如果连续10秒都是故障，则上报该故障。中间如果该故障消除，则重新计时。
//如果仓门在开的状态或者检测不到电池，重新计时。

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

//是否这个故障是可以上报的
//nb : 第几个位的故障
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
        //不过压
        gl_bat_fault_debounce_param[0].total_time_cnt = 0;
    }

    if(!(vl_tmp_value&((1<<1)|(1<<2))))
    {
        //不欠压

        gl_bat_fault_debounce_param[1].total_time_cnt = 0;
    }
    
    if(!(vl_tmp_value&((1<<3)|((1<<4)|(1<<5)))))
    {
        //不过流
        gl_bat_fault_debounce_param[2].total_time_cnt = 0;
    }

    if(!(vl_tmp_value &((1<<6)|(1<<7)|(1<<10)|(1<<11)|(1<<12))))
    {
        //不过温
        gl_bat_fault_debounce_param[3].total_time_cnt = 0;
    }

    if(!(vl_tmp_value &((1<<8)|(1<<9))))
    {
        //不低温
        gl_bat_fault_debounce_param[4].total_time_cnt = 0;
    }

    vl_value_opft2 = Battery_get_reg_value(0, ENUM_REG_ADDR_OPFT2);

    vl_value_devft1 = Battery_get_reg_value(0, ENUM_REG_ADDR_DEVFT1);
    vl_value_devft1 &= (~((1<<7)|(1<<8)));
    
    vl_value_devft2 = Battery_get_reg_value(0, ENUM_REG_ADDR_DEVFT2);
    
    if((vl_tmp_value &((1<<14)|(1<<15)))||
        vl_value_opft2||
        vl_value_devft1||
        vl_value_devft2// 考虑到电池没有完全插进去的时候有NFC而没有霍尔，所以不加电池的霍尔判断
        )
    {
        //其他

        gl_bat_fault_debounce_param[5].total_time_cnt = 0;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
低温充电策略
1、进机柜，电池温度，t≥0，电池直接充电；

2、进机柜，电池在温度，-2≤t＜0，电池回到2度以上充电（包含2度）；
或是回到2度维持0小时，开始充电。先以0.1C的电流充电20分钟，
再根据电池的需求来充电。

3、进机柜，电池在温度，-4≤t＜-2，电池回到4度以上充电（包含4度）；
或是回到2度维持1小时，开始充电。
先以0.1C的电流充电20分钟，再根据电池的需求来充电。

4、进机柜，电池在温度，-6≤t＜-4，电池回到6度以上充电（包含6度）；
或是回到4度维持1小时，开始充电。
先以0.1C的电流充电20分钟，再根据电池的需求来充电。

5、进机柜，电池在温度，-8≤t＜-6，电池回到8度以上充电（包含8度）；
或是回到4度维持2小时，开始充电。
先以0.1C的电流充电20分钟，再根据电池的需求来充电。

6、进机柜，电池在温度，-10≤t＜-8，电池回到8度以上且等待0.5小时开始充电（包含8度）；
或是回到4度维持3小时，开始充电。
先以0.1C的电流充电20分钟，再根据电池的需求来充电。

7、进机柜，电池在温度，t＜-10，电池回到8度以上且等待1小时开始充电（包含8度）；
或是回到4度维持4小时，开始充电。
先以0.1C的电流充电20分钟，再根据电池的需求来充电。

*/
#ifdef BAT_TEMP_CHARGER_ENABLE

/*
{
    short low_temp; // 0.1℃为单位
    unsigned int low_temp_delay_cnt; // 1mS为单位
    short high_temp;// 0.1℃为单位
    unsigned int high_temp_delay_cnt;// 1mS为单位
    unsigned int low_current_delay_count;// 1mS为单位
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
    //是否允许启动该功能"根据电池温度决定是否充电和充电电流"
    if(main_get_temp_charge_enable_flag())
    {
        //允许
        return gl_bat_low_temp_param.is_bat_chg_enable;
    }
    else
    {
        //不允许该功能，则总是允许正常充电
        return 2; // 0, 不允许充电，1，允许前期小电流充电，2，允许正常充电
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


//记录插入电池的最低温度，关舱门时候调用
void Battery_set_bat_temp_value_when_plus_in(void)
{
    gl_bat_low_temp_param.temp_value = Battery_get_bat_lowest_temp(0);
    gl_bat_low_temp_param.is_need_to_reflash_temp = 0;
}

short Battery_get_bat_temp_value_when_plus_in(void)
{
    return gl_bat_low_temp_param.temp_value;
}


//获取当前电池的最低温度
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
    //进机柜，电池温度，t≥0，电池直接充电；
    if(0 <= gl_bat_low_temp_param.temp_value)
    {
        gl_bat_low_temp_param.is_bat_chg_enable = 2;
    }
    else if((-20 <= gl_bat_low_temp_param.temp_value) && (gl_bat_low_temp_param.temp_value < 0))
    {
        /*                    
                2、进机柜，电池在温度，-2≤t＜0，电池回到2度以上充电（包含2度）；
                或是回到2度维持0小时，开始充电。先以0.1C的电流充电20分钟，
                再根据电池的需求来充电。
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
                3、进机柜，电池在温度，-4≤t＜-2，电池回到4度以上充电（包含4度）；
                或是回到2度维持1小时，开始充电。
                先以0.1C的电流充电20分钟，再根据电池的需求来充电。
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
            4、进机柜，电池在温度，-6≤t＜-4，电池回到6度以上充电（包含6度）；
            或是回到4度维持1小时，开始充电。
            先以0.1C的电流充电20分钟，再根据电池的需求来充电。
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
        
            5、进机柜，电池在温度，-8≤t＜-6，电池回到8度以上充电（包含8度）；
            或是回到4度维持2小时，开始充电。
            先以0.1C的电流充电20分钟，再根据电池的需求来充电。
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
            6、进机柜，电池在温度，-10≤t＜-8，电池回到8度以上且等待0.5小时开始充电（包含8度）；
            或是回到4度维持3小时，开始充电。
            先以0.1C的电流充电20分钟，再根据电池的需求来充电。
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
                
            7、进机柜，电池在温度，t＜-10，电池回到8度以上且等待1小时开始充电（包含8度）；
            或是回到4度维持4小时，开始充电。
            先以0.1C的电流充电20分钟，再根据电池的需求来充电。
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


//计算时间和条件判断
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
//当充电器输出电压精度有问题导致输出电压偏低时候，适当调高充电器输出电压。

//获取设置充电器输出电压的偏移值
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

//bat_voltage: 单位是0.01V
//bat_current: 单位是0.01A
void Battery_charger_setting_voltage_offset_process(unsigned short bat_voltage,
                                                    unsigned short bat_current)
{
    unsigned char vl_diff_current_value;
    if(12 >= master_rs485_get_expect_current())// 小于等于1.2A
    {
        vl_diff_current_value = 30; //这里单位是0.01A
    }
    else
    {
        vl_diff_current_value = 60; //这里单位是0.01A
    }
    
    if((TRUE == main_is_charger_id_enable())
        &&(TRUE == master_rs485_is_charge_en())
        &&(30000 != Battery_get_TCurrent(0)) //电流不为0
        &&((gl_charger_setting_V_offset_param.expect_setting_voltage - bat_voltage) >= 0) //0mV
    &&((gl_charger_setting_V_offset_param.expect_setting_current - bat_current) >= vl_diff_current_value) // 600mA
    )
    {        
        //设置计时时间
        if(!gl_charger_setting_V_offset_param.debount_time_cnt)
        {
            gl_charger_setting_V_offset_param.debount_time_cnt = 60000;
        }
    }
    else
    {
        //计时清零
        gl_charger_setting_V_offset_param.debount_time_cnt = 0;
    }
}

#endif


////////////////////////////////////////////////////////////////////////////////////////////////////
/*
读取电池历史记录
*/


//计算真正记录编号。因为记录的范围不是0~0xFFFFFFFF
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

//计算编号之间的差值(end-begin)。因为记录的范围不是0~0xFFFFFFFF
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

//重新初始化
void Battery_history_record_param_reset(void)
{
    Battery_history_record_param_init();
}

//仓内电池ID与配置记录中的电池ID是否一致
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

//获取已经缓存的历史数据
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
    //如果电池的ID和存储的ID不一样，说明换过电池了。需要清零数据参数
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
        //服务器要求的ID与仓内电池的ID不一样，不存数据。
        //如果数据还没有上传，不存数据
        //如果开始编号和需求读数是无效的，不存数据
        //读取的数量已经超过需要的读数，不存数据
    }
    else
    {
        memcpy(gl_bat_history_record_param.history_record_buf, buf_out, 128);
        gl_bat_history_record_param.is_buf_valid = 1;
        Battery_history_record_clear_updated_flag();
        gl_bat_history_record_param.history_record_read_cnt++;
    }
     
}

//设置需要读取的历史记录开始编号和需要读取的数量
void Battery_history_record_set_index_param(unsigned char bat_ID[6], unsigned int log_head, unsigned char datetime[6], unsigned int log_cnt, unsigned int log_read_cnt)
{
    memcpy(gl_bat_history_record_param.bat_ID, bat_ID, 6);
    gl_bat_history_record_param.history_record_begin_index = log_head;
    memcpy(gl_bat_history_record_param.history_record_begin_index_datetime, datetime, 6);
    gl_bat_history_record_param.history_record_cnt = log_cnt;
    //标记参数有效
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

    //缓存的历史数据被上位机读取后，马上查询一次是否需要读取历史记录数据
    Batter_history_record_set_nfc_cmd_interval_cnt(0);
}

//查询某值是否在一个范围内。
bool Battery_history_record_is_within_range(unsigned int range_begin, unsigned int range_end, unsigned int data)
{
    if(range_end >= range_begin)
    {
        if((data > range_end)||(data < range_begin))
        {
            //编号不正确
            return FALSE;
        }
    }
    else
    {
        if((range_begin < data )&&(data < range_end))
        {
            //编号不正确
            return FALSE;
        }
    }

    return TRUE;
}


//查询编号是否在记录范围内
bool Battery_history_record_is_index_within_range(unsigned int index)
{
    unsigned int vl_int_new_tmp;
    unsigned int vl_int_old_tmp;
    //读取电池里面的最近记录编号
    vl_int_new_tmp = Battery_get_newest_record_index_number();

    //读取电池里面的最早记录编号
    vl_int_old_tmp = Battery_get_oldest_record_index_number();

    return Battery_history_record_is_within_range(vl_int_old_tmp, vl_int_new_tmp, Battery_history_record_get_real_index(index));
}

//是否需要通过NFC读取历史记录
bool Battery_history_record_is_need_to_read_record(void)
{
    unsigned char vl_bat_ID[6];

    //如果已经读取完需求数据，退出
//    if((gl_bat_history_record_param.history_record_read_cnt == gl_bat_history_record_param.history_record_cnt)
//    //||() //读取全部数据的情况
//    )
//    {
//        return FALSE;
//    }
    
    Battery_get_bat_ID(0, vl_bat_ID);
    if(((0 == gl_bat_history_record_param.is_buf_valid)// 历史数据缓存无效
        ||(gl_bat_history_record_param.is_buf_updated)) //已经上报给上位机
        &&(gl_bat_history_record_param.is_index_valid) // 设置的编号是有效的
        &&(TRUE == Battery_history_record_is_bat_ID_same(vl_bat_ID)) //Bat ID和仓内电池一致
        //&&(TRUE == Battery_history_record_is_index_within_range(gl_bat_history_record_param.history_record_begin_index)) //起始编号在记录范围内
        &&(TRUE == Battery_history_record_is_index_within_range(
                    Battery_history_record_get_real_index(gl_bat_history_record_param.history_record_begin_index + 1 + gl_bat_history_record_param.history_record_read_cnt))
                                                                ) //准备读取的编号在记录范围内
        )
    {
        return TRUE;
    }
    if((((0xFFFFFFFF != gl_bat_history_record_param.history_record_cnt) && (gl_bat_history_record_param.history_record_read_cnt < gl_bat_history_record_param.history_record_cnt)) // 有限读取时候
            ||((0xFFFFFFFF == gl_bat_history_record_param.history_record_cnt) && (gl_bat_history_record_param.history_record_lastest_index != Battery_history_record_get_real_index(gl_bat_history_record_param.history_record_begin_index + gl_bat_history_record_param.history_record_read_cnt)))) // 无限读取时候，读到最后一条为止
        &&(FALSE == Battery_history_record_is_index_within_range(
                                    Battery_history_record_get_real_index(gl_bat_history_record_param.history_record_begin_index + 1 + gl_bat_history_record_param.history_record_read_cnt))))
    {
        gl_bat_history_record_param.history_record_read_cnt++;
    }
    return FALSE;
}

/*
读取历史记录流程的状态机状态

return:
00：等待状态
01：请求"读取起始电池历史记录编号和记录条数"
10：已经成功读取1条电池历史记录
*/
unsigned char Battery_history_record_state_of_process(void)
{
    unsigned char vl_bat_ID[6];
    Battery_get_bat_ID(0, vl_bat_ID);
    if((0 == gl_bat_history_record_param.is_index_valid) //配置参数是否有效
        ||(FALSE == Battery_history_record_is_bat_ID_same(vl_bat_ID)) //仓内电池的ID与配置的ID是否一致
        
        ||((1 == gl_bat_history_record_param.is_index_valid)&&
            ((gl_bat_history_record_param.history_record_cnt == gl_bat_history_record_param.history_record_read_cnt) || 
                    ((0xFFFFFFFF == gl_bat_history_record_param.history_record_cnt) && (gl_bat_history_record_param.history_record_lastest_index == Battery_history_record_get_real_index(gl_bat_history_record_param.history_record_begin_index + gl_bat_history_record_param.history_record_read_cnt))))
            &&(gl_bat_history_record_param.is_buf_updated)) //完全读完一个配置的数量
        )
    {
        return 0x01; // 请求"读取起始电池历史记录编号和记录条数"
    }

    if((0 == gl_bat_history_record_param.is_buf_updated)
        &&(gl_bat_history_record_param.is_buf_valid))
    {
        return 0x02; // 已经成功读取1条电池历史记录
    }

    return 0x00; // 等待状态

}

void Batter_history_record_set_nfc_cmd_interval_cnt(unsigned int cnt)
{
    gl_bat_history_record_param.NFC_CMD_interval_cnt = cnt;
}


/*
3秒钟查询一次是否需要读取历史记录数据
或者刚发送完历史时间给上位机时候再查询是否需要读取历史记录。
*/
void Battery_history_record_process(void)
{
    //
    if((0 == gl_bat_history_record_param.NFC_CMD_interval_cnt) //延迟完成后
        &&(gl_cmd_buf[0][ENUM_NFC_CMD_INDEX_HISTORY_RECORD] == 0) // 命令执行后
        &&(MM_is_snd_done_by_index(gl_modbus_param.BMS_index) // nfc操作完成后
        &&(gl_cmd_buf[0][ENUM_NFC_CMD_INDEX_R_CTRL] == 1) //其他寄存器的查询已经开始后
        )
        //间隔轮询期间，同时电芯最低电压小于3V，禁止读取BAT历史数据
        &&((0 != sl_bat_poll_param.is_poll_allways_en_flag) //处于一直轮询状态
        || (sl_bat_poll_param.poll_interval_MAX < BAT_POLL_BIG_INTERVAL_MAX))//电芯电压大于等于3V
    )
    {
        if(Battery_history_record_is_need_to_read_record())
        {
            //读取电池里面的最近记录编号
            gl_bat_history_record_param.history_record_lastest_index = Battery_get_newest_record_index_number();
                //读取电池里面的最早记录编号
            gl_bat_history_record_param.history_record_earliest_index = Battery_get_oldest_record_index_number();
                
            gl_cmd_buf[0][ENUM_NFC_CMD_INDEX_HISTORY_RECORD] = 1;
        }
    }
}

/*
比较时间
DT格式: 年/月/日/时/分/秒；BCD码
return :
0, 相同
1, DT1比DT2的时间更加新
-1, DT2比DT1的时间更加新
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

