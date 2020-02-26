

/**@file
 *
 * @defgroup app_fifo FIFO implementation
 * @{
 * @ingroup app_common
 *
 * @brief FIFO implementation.
 */

#ifndef BATTERY_PROCESS_H__
#define BATTERY_PROCESS_H__

#include "fm175xx.h"
#include "comm_board_protocol.h"


#ifndef BATTERY_PROCESS_C
#define GLOBAL_BPROC extern
#else
#define GLOBAL_BPROC
#endif



typedef enum{
    ENUM_DISCHARGE,
    ENUM_CHARGE,
    ENUM_SLEEP
}ENUM_BMS_STATE;


typedef struct{
    ENUM_BMS_STATE bms_state;

}st_BP_info;

GLOBAL_BPROC st_BP_info gl_BP_info;

GLOBAL_BPROC unsigned char is_battery_voltage_OK(unsigned char bms_index);
#define BATTERY_ERROR_OPTION_IGNORE_NONE 0 //������
#define BATTERY_ERROR_OPTION_IGNORE_TEMPERATURE 1 // �����¶�
GLOBAL_BPROC unsigned char is_battery_error(unsigned char bms_index, unsigned char option_ignore);
GLOBAL_BPROC unsigned char is_battery_checked_enable_for_detecting_bat(unsigned char bms_index);
GLOBAL_BPROC unsigned char is_battery_critical_error(unsigned char bms_index);
GLOBAL_BPROC void Battery_save_bat_reg_state_when_critical_error(unsigned char bms_index);
GLOBAL_BPROC unsigned int  Battery_get_bat_reg_state_when_critical_error(unsigned char* p);

GLOBAL_BPROC void Battery_charge_process(void);
GLOBAL_BPROC void Battery_cmd_buf_clear(unsigned char bms_index);
GLOBAL_BPROC void Battery_cmd_buf_get_version(unsigned char bms_index);
GLOBAL_BPROC bool Battery_is_one_cmd_buf_empty(unsigned char bms_index);
#if 0
void Battery_info_polling_Process(void);
void Battery_send_cmd(void);
#else
GLOBAL_BPROC void Battery_info_polling_Process(unsigned char bms_index);
GLOBAL_BPROC unsigned char Battery_send_cmd(unsigned char bms_index);
#endif



enum{
    ENUM_NFC_CMD_INDEX_R_VERSION,
    ENUM_NFC_CMD_INDEX_R_INFO1,
    ENUM_NFC_CMD_INDEX_R_INFO2,
    ENUM_NFC_CMD_INDEX_R_CTRL,
    ENUM_NFC_CMD_INDEX_W_CTRL,
    ENUM_NFC_CMD_INDEX_R_PARAM1,
    ENUM_NFC_CMD_INDEX_R_PARAM2,
    ENUM_NFC_CMD_INDEX_W_PARAM,
    ENUM_NFC_CMD_INDEX_R_USERDATA,
    ENUM_NFC_CMD_INDEX_W_USERDATA,
    ENUM_NFC_CMD_INDEX_W_RESET,
    ENUM_NFC_CMD_INDEX_R_BLV,//��ѯbootloader�汾��
    ENUM_NFC_CMD_INDEX_BYPASS,// ͸��
    ENUM_NFC_CMD_INDEX_AUTHORITY, //��Ȩ����
    ENUM_NFC_CMD_INDEX_HISTORY_RECORD,
    ENUM_NFC_CMD_INDEX_MAX
};

//������־��Щ�����Ѿ���BMS�ж�ȡ��ɣ����Թ��ο�ʹ��
#define BMS_DEV_INFO_FLAG_BIT (1<<0)
#define BMS_READONLY_DATA1_FLAG_BIT (1<<1)
#define BMS_READONLY_DATA2_FLAG_BIT (1<<2)
#define BMS_CTRL_FLAG_BIT (1<<3)
#define BMS_PARAM1_FLAG_BIT (1<<4)
#define BMS_PARAM2_FLAG_BIT (1<<5)
#define BMS_USER_FLAG_BIT (1<<6)





// �����ʾ������Ҫ����
GLOBAL_BPROC unsigned char gl_cmd_buf[NFC_READER_COUNT_MAX][ENUM_NFC_CMD_INDEX_MAX];



#define BMS_CTRL_BIT_CHG_SWITCH (1<<0)
#define BMS_CTRL_BIT_SUPPLY_SWITCH (1<<1)
#define BMS_CTRL_BIT_PRESUPPLY_SWITCH (1<<2)



#define BMS_STATE_BIT_CHG_SWITCH (1<<0)
#define BMS_STATE_BIT_SUPPLY_SWITCH (1<<1)
#define BMS_STATE_BIT_PRESUPPLY_SWITCH (1<<2)
//���������λ
#define BMS_STATE_BIT_CHG_IN (1<<7)
//��س������־λ
#define BMS_STATE_BIT_FULLY_CHARGE (1<<14)




enum{
    ENUM_REG_ADDR_HWVER = 1, //Ӳ���汾
    ENUM_REG_ADDR_FWMSV = 3, //�̼��汾
    ENUM_REG_ADDR_FWREV = 4, //�̼��汾- �����汾
    ENUM_REG_ADDR_FWBNH = 5, //����汾����
    ENUM_REG_ADDR_FWBNL = 6, //����汾����
    
ENUM_REG_ADDR_MCUN12 = 7,
ENUM_REG_ADDR_MCUN34 = 8,
ENUM_REG_ADDR_MCUN56 = 9,
ENUM_REG_ADDR_MCUN78 = 10,

ENUM_REG_ADDR_BVOLT = 12, //���ѹ
ENUM_REG_ADDR_BCAP = 13,//�����
ENUM_REG_ADDR_SN12 = 14,
ENUM_REG_ADDR_SN34 = 15,
ENUM_REG_ADDR_SN56 = 16,
ENUM_REG_ADDR_SN78 = 17,
ENUM_REG_ADDR_ERECH = 18, // �����¼�Ÿ���
ENUM_REG_ADDR_ERECL = 19, // �����¼�ŵ���
ENUM_REG_ADDR_LRECH = 20, // �����¼�Ÿ���
ENUM_REG_ADDR_LRECL = 21, // �����¼�ŵ���


ENUM_REG_ADDR_LTSTA = 22, //��ʷ״̬

//USER ID 
ENUM_REG_ADDR_RO_USERID12 = 256,
ENUM_REG_ADDR_RO_USERID34 = 257,
ENUM_REG_ADDR_RO_USERID56 = 258,
ENUM_REG_ADDR_RO_USERID78 = 259,

ENUM_REG_ADDR_STATE = 260,
ENUM_REG_ADDR_SOC = 261,
ENUM_REG_ADDR_TVOLT = 262, //Ŀǰ�����ѹ
ENUM_REG_ADDR_TCURR = 263, //Ŀǰ�������
ENUM_REG_ADDR_HTEMP = 264, //��ߵ���¶�
ENUM_REG_ADDR_LTEMP = 266,//��͵���¶�
ENUM_REG_ADDR_HVOLT = 268,//��ߵ����ѹ
ENUM_REG_ADDR_LVOLT = 270,// ��͵����ѹ
ENUM_REG_ADDR_CSOP = 273,//10s������������
ENUM_REG_ADDR_SOH = 274,//         ����״̬
ENUM_REG_ADDR_CYCLE = 275,//ѭ������

ENUM_REG_ADDR_DEVFT1 = 280, //���й�����1
ENUM_REG_ADDR_DEVFT2 = 281, //���й�����1
ENUM_REG_ADDR_OPFT1 = 282, //���й�����1
ENUM_REG_ADDR_OPFT2 = 283, //���й�����2
ENUM_REG_ADDR_OPWARN1 = 284, //���и澯��1
ENUM_REG_ADDR_OPWARN2 = 285, //���и澯��2
ENUM_REG_ADDR_CMOST = 286, //���MOS�¶�
ENUM_REG_ADDR_DMOST = 287, //�ŵ�MOS�¶�
ENUM_REG_ADDR_FUELT = 288 , //�������¶�,
ENUM_REG_ADDR_CONT = 289, //�������¶�
ENUM_REG_ADDR_BTEMP1 = 290, //����¶�1
ENUM_REG_ADDR_BTEMP2 = 291, //����¶�2
ENUM_REG_ADDR_BVOLT1 = 292, // ������1��ѹ
ENUM_REG_ADDR_BVOLT2 = 293, // ������2��ѹ
ENUM_REG_ADDR_BVOLT3 = 294, // ������3��ѹ
ENUM_REG_ADDR_BVOLT4 = 295, // ������4��ѹ
ENUM_REG_ADDR_BVOLT5 = 296, // ������5��ѹ
ENUM_REG_ADDR_BVOLT6 = 297, // ������6��ѹ
ENUM_REG_ADDR_BVOLT7 = 298, // ������7��ѹ
ENUM_REG_ADDR_BVOLT8 = 299, // ������8��ѹ
ENUM_REG_ADDR_BVOLT9 = 300, // ������9��ѹ
ENUM_REG_ADDR_BVOLT10 = 301, // ������10��ѹ
ENUM_REG_ADDR_BVOLT11 = 302, // ������11��ѹ
ENUM_REG_ADDR_BVOLT12 = 303, // ������12��ѹ
ENUM_REG_ADDR_BVOLT13 = 304, // ������13��ѹ
ENUM_REG_ADDR_BVOLT14 = 305, // ������14��ѹ
ENUM_REG_ADDR_BVOLT15 = 306, // ������15��ѹ
ENUM_REG_ADDR_BVOLT16 = 307, // ������16��ѹ


ENUM_REG_ADDR_MAXCHGV = 315,//������ѹ
ENUM_REG_ADDR_TVST = 317, // TVS�¶�
ENUM_REG_ADDR_BHR1 = 318, //���1�����ٶ�
ENUM_REG_ADDR_BHR2 = 319, //���2�����ٶ�
ENUM_REG_ADDR_FGHR = 320, // �����������ٶ�
ENUM_REG_ADDR_DROCV = 321, // ��о��ѹ����½��ٶ�
ENUM_REG_ADDR_IDROCV = 322, // ��о��ѹ����½��ٶȱ��


ENUM_REG_ADDR_CTRL = 512, 
ENUM_REG_ADDR_RESET = 534,


//USER ID 
ENUM_REG_ADDR_USERID12 = 4097,
ENUM_REG_ADDR_USERID34 = 4098,
ENUM_REG_ADDR_USERID56 = 4099,
ENUM_REG_ADDR_USERID78 = 4100,




};


//ÿ����ѯ��ʱ�䳤��
#define BAT_POLL_ON_TIME_MAX 10000
// ��ѯ��ؼ��ʱ������
#define BAT_POLL_INTERVAL_MAX 60000
#define BAT_POLL_BIG_INTERVAL_MAX 86400000 // 24Сʱ
#define BAT_POLL_BIG_BIG_INTERVAL_MAX 0xFFFFFFFE//259200000 // 72Сʱ
// �����б仯ʱ��һֱ����ֱ��ѯģʽ1���ӣ������ٸ���ʵ���������ģʽ
#define BAT_POLL_DELAY_MAX 60000//1800000//360000         //��Ϊ60��
//���յ�����BMS��start �������������ѯ��ص�ʱ�䣬10����
#define BAT_POLL_BMS_UPDATE_DELAY_MAX 600000
//����͸����������ʱ��ֹͣ�����ѯ180��
#define BAT_POLL_DELAY_MAX_BY_BYPASS_CMD 180000
typedef struct{
unsigned char is_status_changed_on_door;
unsigned char is_poll_allways_en_flag; // 0, ��ʾ��ʱ��������ѯ��1����ʾһֱ��ѯ���޼��ʱ��
unsigned int poll_inerval_timer_cnt;
unsigned int poll_interval_MAX;
unsigned int poll_allways_en_delay_cnt; // �ڲ����б仯��ʱ����reset��������֤�ղ��ȥ�ĵ������ѯ������
unsigned int poll_interval_for_bms_update_cnt;
}st_bat_poll_param;
GLOBAL_BPROC st_bat_poll_param sl_bat_poll_param;

/*
����ع���
*/
typedef struct{
//�õ���Ƿ��й��ϡ�0��û�й���
unsigned char is_bat_error;
//�Ƿ���Ҫ����ع��ϣ�����⵽�е�أ����SOC�����Źر�30����⡣
unsigned char is_need_bat_checking; // 0�� ����Ҫ��⣻1�����ڼ�⣻2��������
//������
unsigned char checking_step;
//ÿ�����ȵĳ�ʱʱ�䣬��ʱ���˳���
unsigned int checking_step_timeout_cnt;
//���Źرպ�ļ�����
unsigned int door_close_delay_cnt;
//ͨ�ü�����
unsigned int comm_counter_cnt;
//ÿ���Ӳ�ѯһ�εļ�����
unsigned int bat_process_cnt;

///////////////////////////////////
//���û�м�⵽��Ԥ�ţ�ÿ5�����ټ��һ�Σ������10��
unsigned int bat_connected_faild_error_cnt; //�����ã�5���Ӽ���
unsigned char bat_connected_faild_error_cnt_reload_max; // �������������10��
///////////////////////////////////
//�����Step��5���ӳ�ʱ���ϣ���30���Ӻ��Զ�����PMS��
unsigned int bat_step_timeout_err_cnt;
///////////////////////////////////

}st_check_bat_err;

GLOBAL_BPROC st_check_bat_err gl_check_bat_err;

//��ز���ʱ��ʹ�ó������������Ƿ���λ
typedef struct{
unsigned char is_bat_v_valid; //������Ƿ��⵽��������ѹ
//�Ƿ���Ҫ����ص�ѹ������⵽�е�أ����SOC�����ŹرվͿ�ʼ���
unsigned char is_need_bat_v_checking; // 0�� ����Ҫ��⣻1�����ڼ�⣻2��������
unsigned short bat_v_charger_cmd_cnt; //��ʼ�򿪵��Ԥ�ź�������ͨѶ�Ĵ���
unsigned int bat_v_charger_comm_timeout_cnt; //��ʼ�򿪵��Ԥ�ź�������ͨѶ��ʱ�����
unsigned short bat_v_value; //�������⵽�ĵ�������ѹֵ����λ0.1V
unsigned short bat_v_charger_check_totol_time_cnt; //�������̵����ʱ�䡣
unsigned char bat_v_charger_charge_valid_cnt; // ������е���������ҵ���е�������Ĵ����������Ӧ��ʱ�����
}st_check_bat_v;
GLOBAL_BPROC st_check_bat_v gl_check_bat_v;


typedef enum{
    ENUM_SCR_OF_STOP_FLAG_CURRENT_DIFF, //���������-��ص���>Ԥ��ֵ�����ٺ�����
    ENUM_SCR_OF_STOP_FLAG_HIGH_TEMP, //���ʱ���κε�ش������¶ȴ���Ԥ��ֵ��80�ȣ�
    ENUM_SCR_OF_STOP_FLAG_PRE_CURRENT, //���Ԥ����ʧ��(��Ҫ������������Ч��������ж�)
    ENUM_SCR_OF_STOP_FLAG_CHG_SOC_UNWORK, //������һ��ʱ���20���ӣ�����ص�ѹ��SOCû������Ԥ����
    ENUM_SCR_OF_STOP_FLAG_LOW_VOLTAGE, //������������ʱ������⵽��ص�ѹ����26V��13S����32V��16S)�������������
    /*
        PMS��ص���ڲ���ѹ������⵽��ص�ѹ�ﵽ������ѹ�󣬳�������δ���ͣ�
            ����ʵ�����������һ�£�ͬʱ��ص�ѹ��Ȼ���ӣ���ʱ��Ͽ�������Ĺ���
    */
    ENUM_SCR_OF_STOP_FLAG_MAX_VOLTAGE_EXCEEDED, 
    ENUM_SCR_OF_STOP_FLAG_CURRENT_OUT, //����г������ٺ����ĵ������(�ڳ����ǲ������)
    ENUM_SCR_OF_STOP_FLAG_SOC_ERROR, //SOCΪ0�����ǵ�ѹ�ߡ�
    ENUM_SCR_OF_STOP_FLAG_UVP_ERROR,//������UVP���ϵ��ǵ�ѹ��SOC��
    ENUM_SCR_OF_STOP_FLAG_UVP_OVP_ERROR, //ͬʱ���ֵ�ѹ�͸�ѹ�澯
    ENUM_SCR_OF_STOP_FLAG_UTP_OTP_ERROR, //ͬʱ���ֵ��º͸��¸澯
    ENUM_SCR_OF_STOP_FLAG_HALL_OFF, //��ػ�������û�м�⵽����
    ENUM_SCR_OF_STOP_FLAG_CELL_VOLTAGE_LOW, // ��о��ѹ����2V
    
    ENUM_SCR_OF_STOP_FLAG_CHARGER_OCP_LOCK, //�ز��ź󣬳���������ϱ��������ϣ��������ϣ�ֹͣ���
    ENUM_SCR_OF_STOP_FLAG_CHARGER_SCP_LOCK, //�ز��ź󣬳���������ϱ���·���ϣ��������ϣ�ֹͣ���

    ENUM_SCR_OF_STOP_FLAG_CELL_V_DIFF_VALUE_EXCEEDED, // ��о��ѹ�������ߵ�о��ѹ����͵�о��ѹ��ֵ���ڵ���0.1V
    
    ENUM_SCR_OF_STOP_FLAG_CHARGER_OVP_LOCK, //�ز��ź󣬳���������ϱ���ѹ���ϣ��������ϣ�ֹͣ���
    ENUM_SCR_OF_STOP_FLAG_CHARGER_OTP_LOCK, //�ز��ź󣬳���������ϱ����¹��ϣ�����ֻҪ������������Լ������

    ENUM_SCR_OF_STOP_FLAG_MAX = 32 //���ܳ��������ֵ32
}enum_src_of_stop_flag;

typedef struct{
//���ֹͣ����ԭ��Դ
unsigned int src_of_stop_flag;
unsigned char bat_ID[6];
unsigned char stop_flag;
unsigned int current_diff_counter;
unsigned int bat_current_out_counter; 
unsigned int hall_miss_status_clear_delay_cnt;
//ѡ����Щԭ��Դ���Ա����õ�src_of_stop_flag��
unsigned int src_of_stop_flag_disable_flag; // ��Ӧsrc_of_stop_flag�е�ÿ��Bit��0������1�ǽ�ֹ
}st_event_to_stop_chg_param;
GLOBAL_BPROC st_event_to_stop_chg_param gl_event_to_stop_chg_param;

typedef struct{
    unsigned char is_bat_chg_enable; // 0, �������磬1������ǰ��С������磬2�������������
    unsigned char is_need_to_reflash_temp; //�Ƿ���Ҫˢ�µ�ز���ʱ��ĵ���¶�ֵ
    short temp_value;//������ʱ��������¶ȣ���λ��0.1��
    unsigned int delay_counter_1;//��һ�׶μ�����
    unsigned int delay_counter_2;//�ڶ��׶μ�����
    unsigned int delay_counter_charge; //ǰ�ڳ��׶μ���
    unsigned int low_current_delay_count; //�͵�������ʱ�䳤��
}st_bat_low_temp_param;
GLOBAL_BPROC st_bat_low_temp_param gl_bat_low_temp_param;

typedef struct{
    short low_temp; // 0.1��Ϊ��λ
    unsigned int low_temp_delay_cnt; // 1mSΪ��λ
    short high_temp;// 0.1��Ϊ��λ
    unsigned int high_temp_delay_cnt;// 1mSΪ��λ
    unsigned int low_current_delay_count;// 1mSΪ��λ
}st_bat_low_temp_ctrl_param;

//������������ѹ���������⵼�������ѹƫ��ʱ���ʵ����߳���������ѹ��
typedef struct{
unsigned char reseve[3];
unsigned char voltage_offset_value; // ��λ0.1V
unsigned int debount_time_cnt;
//��ǰ��������ѹ(δ��ƫ����)����λ0.01V
unsigned short expect_setting_voltage;
//��ǰ����������(δ��ƫ����)����λ0.01A
unsigned short expect_setting_current;
}st_charger_setting_V_offset_param;
GLOBAL_BPROC st_charger_setting_V_offset_param gl_charger_setting_V_offset_param;

//�������̵Ĳ���
typedef struct{
unsigned char connector_default_state:1; //������״̬; 1, δ����; 0, ���Ӻ�
unsigned char switch_default_state:1; //΢������״̬; 1, δ��λ; 0, ��λ
unsigned char reserve_bit:6;
}st_bat_theft_prevention_param;
#ifndef BATTERY_PROCESS_C
GLOBAL_BPROC st_bat_theft_prevention_param gl_bat_theft_prevention_param;
#else
GLOBAL_BPROC st_bat_theft_prevention_param gl_bat_theft_prevention_param = {0, 0, 0};
#endif


/*
��ȡ�����ʷ��¼
*/
//��ʷ��¼��ŵķ�Χ
#define HISTORY_RECORD_INDEX_START 1
#define HISTORY_RECORD_INDEX_END 536862720

typedef struct{
//���Ͷ�ȡ��ʷ���ݵ�NFCͨѶ����ļ��ʱ��
unsigned int NFC_CMD_interval_cnt;
//��Ҫ��ȡ�ĵ�ǰ��¼��ţ�������Ҫ��ȡ���Ǵ���һ����ſ�ʼ
unsigned int history_record_begin_index;
//��Ҫ��ȡ�ļ�¼��Ŀ
unsigned int history_record_cnt;
//��ǰ�Ѿ���ȡ�ļ�¼��Ŀ
unsigned int history_record_read_cnt;
//��ǰ��������¼���
unsigned int history_record_earliest_index;
//��ǰ��������¼���
unsigned int history_record_lastest_index;
//��Ҫ��ȡ�ĵ�ǰ��¼��ŵ�ʱ�� �� BCD��
unsigned char history_record_begin_index_datetime[6];
//��ǰ��������¼��ŵ�ʱ��
unsigned char history_record_earliest_index_datetime[6];
//��ǰ��������¼��ŵ�ʱ��
unsigned char history_record_lastest_index_datetime[6];
//������ʷ����
unsigned char history_record_buf[128];
//���ID
unsigned char bat_ID[6];
//��ʼ��¼��ź���Ŀ�Ƿ���Ч
unsigned char is_index_valid:1;
//buffer�е������Ƿ���Ч
unsigned char is_buf_valid:1;
//buffer�е������Ƿ��Ѿ��ϴ�
unsigned char is_buf_updated:1;
//��ǰ��������¼��ź�ʱ���ֵ�Ƿ���Ч
unsigned char is_earliest_param_valid:1;
//��ǰ��������¼��ź�ʱ���ֵ�Ƿ���Ч
unsigned char is_lastest_param_valid:1;
unsigned char bit_reserve:4;
}st_bat_history_record_param;

GLOBAL_BPROC st_bat_history_record_param gl_bat_history_record_param;



//BAT_ERROR_CHECK_DELAY_AFTER_BAT_UPDATE
GLOBAL_BPROC unsigned int sl_bat_error_check_delay_after_bat_update_cnt;


typedef struct{
    unsigned char OD_update_file_max_subindex;
    /*�����ļ���Ӳ���汾��*/
    unsigned char OD_update_file_HW_version[2];
    /*�����ļ��Ĺ̼��汾��*/
    unsigned char OD_update_file_FW_version[7];
    /*�����ļ���buffer��ַ*/
    unsigned int OD_update_file_buff_addr;
}st_update_file_param;

/*��������*/
#define OD_PMS_CMD_MASK_ULOCK (1<<0)
/*���ʹ��*/
#define OD_PMS_CMD_MASK_CHARGE_EN (1<<1)
/*�����¶ȶ�̬���ʹ��*/
#define OD_PMS_CMD_MASK_D_CHARGE_EN (1<<2)
/*5V��Դ���ʹ��*/
#define OD_PMS_CMD_5V_EN (1<<3)
/*12V��Դ���ʹ��*/
#define OD_PMS_CMD_12V_EN (1<<4)
/*��֤�����Ƿ�����*/
#define OD_PMS_BAT_AUTHORITY_EN (1<<5)

typedef struct{
unsigned char is_bat_detected;
/**************************************************************************************************/
/*�ߵ��²���ʱ����Ҫ���¶�����*/
short heater_NTC_temperature;
short cabinet_NTC_temperature;
short charger_temperature;

/**************************************�ֵ����************************************************/
/*�ֵ�; Index: 0x3010*/
unsigned char OD_3010_max_subindex;
/*
*bit0: ����Ƿ�����
*bit1: ������Ƿ�����
*bit2: �����״̬
*bit3~4: ǰ���ŵ����λ��״̬
*bit5: ǰ����������״̬
*bit6: ����״̬

*/
unsigned char OD_pms_status1;
/*
*bit0: ��ǰ���ó����ʹ�ܵ�״̬
*bit1: PMS���翪��״̬
*bit2: PMS����ȿ���״̬
*bit3: PMS���������ܿ���״̬
*bit4: PMS����֤��صĽ��
*bit5: �����֤PMS��Ľ��
*/
unsigned char OD_pms_status2;

/*�ֵ�; Index: 0x3011*/
unsigned char OD_3011_max_subindex;
/*�ñ�����ÿ��Bit����ͬ��˼�����ϵĺ궨��*/
unsigned char OD_pms_cmd; /*ʵ��״̬��ֻ��*/
unsigned char OD_pms_cmd_set; /*����OD_pms_cmd��ӦbitΪ1��ֻд*/
unsigned char OD_pms_cmd_clear; /*дOD_pms_cmd_clear bitΪ1����ʾ����OD_pms_cmd��ӦbitΪ0��ֻд*/
/*��������*/
unsigned short OD_charging_current_max;
/*�ɻ�������ֵ*/
unsigned char OD_swapping_soc;
/*������ѹ*/
unsigned short OD_charging_voltage_max;

/*�ֵ�; Index: 0x3012*/


/*�ֵ�; Index: 0x3013*/
unsigned char OD_3013_max_subindex;
/*PMS Ӳ���汾��*/
unsigned char OD_PMS_HW_version[2];
/*PMS�̼��汾��*/
unsigned char OD_PMS_FW_version[7];
/*PMS SN��*/
unsigned char OD_PMS_SN[14];


/*�ֵ�; Index: 0x3014*/
unsigned char OD_3014_max_subindex;
/*���ȵ���λ�õ�NTC�¶�ֵ*/
short OD_heater_ntc_temperature;
/*�ֶ�λ�õ�NTC�¶�ֵ*/
short OD_cabinet_ntc_temperature;
/*�̸�1�ĵ�ѹֵ����λ1mV*/
unsigned short OD_smoke_sensor1_value;
/*�̸�2�ĵ�ѹֵ����λ1mV*/
unsigned short OD_smoke_sensor2_value;
/*�̸�װ�õĹ�����������λ1mA*/
unsigned short OD_smoke_senor_total_current;
/*ǰ���ŵ����ǰ������������λ1mA*/
unsigned short OD_front_motor_working_current;

/*�ֵ�; Index: 0x3015*/
unsigned char OD_3015_max_subindex;
/*��������صĳ���������λ0.1A*/
unsigned short OD_charger_C;
/*��������صĳ���ѹ����λ0.1V*/
unsigned short OD_charger_V;
/*��������صĳ�繦�ʣ���λ0.1W*/
unsigned short OD_charger_P;
/*��������صĹ���1ֵ*/
unsigned char OD_charger_error1;
/*��������صĹ���2ֵ*/
unsigned char OD_charger_error2;
/*��������ص��¶�ֵ����Ҫƫ��40��*/
unsigned char OD_charger_temperature;
/*��������صı����ֽ�*/
unsigned char OD_charger_reserved;

/*�ֵ�; Index: 0x3016*/
unsigned char OD_3016_max_subindex;
/*��������ص�Ӳ���汾��*/
unsigned char OD_charger_HW_version[2];
/*��������صĹ̼��汾��*/
unsigned char OD_charger_FW_version[7];
/*��������ص�SN��*/
unsigned char OD_charger_SN[14];

/*�ֵ�; Index: 0x3017*/
unsigned char OD_3017_max_subindex;
/*��ǰ���ó�����ĳ���ѹ*/
unsigned short OD_charger_voltage_setting_value;
/*��ǰ���ó�����ĳ�����*/
unsigned short OD_charger_current_setting_value;

/*�ֵ�; Index: 0x3018*/
unsigned char OD_3018_max_subindex;
/*PMS��¼�Ĺ���*/
/*��������صĹ���*/
unsigned char OD_pms_active_detected_err_of_bat;
/*��������صĹ���*/
unsigned char OD_pms_detected_err_of_bat[4];
/*PMS��⵽����������*/
unsigned char OD_pms_other_detected_err;
/*PMSԭFAULT����*/
unsigned char OD_pms_fault;
/*PMSԭDAMAGE����*/
unsigned char OD_pms_damage;

/*�ֵ�; Index: 0x3019*/
/*PMS�����ļ��Ƿ���Ч��ÿ��bit����һ�������ļ�����Ӧ0x301A~0x301F*/
unsigned short OD_pms_update_file_flag;

/*�ֵ�; Index: 0x301A*/
st_update_file_param OD_pms_update_file_param[1];

/*�ֵ�; Index: 0x3020*/
/*����������ļ��Ƿ���Ч��ÿ��bit����һ�������ļ�����Ӧ0x3021~0x302F*/
unsigned short OD_charger_update_file_flag;

/*�ֵ�; Index: 0x3021*/
/*�ֵ�; Index: 0x3022*/
/*�ֵ�; Index: 0x3023*/
st_update_file_param OD_charger_update_file_param[3];


/*�ֵ�; Index: 0x3030*/
/*����������ļ��Ƿ���Ч��ÿ��bit����һ�������ļ�����Ӧ0x3031~0x30AF*/
unsigned char OD_bat_update_file_flag[16];

/*�ֵ�; Index: 0x3031*/
/*�ֵ�; Index: 0x3032*/
/*......*/
st_update_file_param OD_bat_update_file_param[64];



/***************************************************************************************************/

}st_pms_param;
GLOBAL_BPROC st_pms_param gl_pms_param;


GLOBAL_BPROC void Battery_chg_param_init(void);
GLOBAL_BPROC void Battery_chg_set_charge_enable(unsigned char bms_index);
GLOBAL_BPROC void Battery_chg_set_charge_disable(unsigned char bms_index);
GLOBAL_BPROC bool Battery_chg_is_charge_enable(unsigned char bms_index);
GLOBAL_BPROC void Battery_check_bat_v_via_charger_init(void);
GLOBAL_BPROC bool Battery_check_bat_v_is_off(void);
GLOBAL_BPROC bool Battery_check_bat_v_is_on(void);
GLOBAL_BPROC bool Battery_check_bat_v_is_end(void);
GLOBAL_BPROC bool Battery_check_bat_v_is_valid(void);
GLOBAL_BPROC void Battery_check_bat_v_set_valid_flag(void);
GLOBAL_BPROC bool Battery_check_bat_v_process(unsigned char bms_index);
GLOBAL_BPROC void Battery_save_bat_reg_state_when_detect_bat_error(unsigned char bms_index);
GLOBAL_BPROC unsigned int Battery_get_bat_reg_state_when_detect_bat_error(unsigned char* p);

GLOBAL_BPROC bool Battery_check_bat_is_on(void);
GLOBAL_BPROC bool Battery_check_bat_is_end(void);
GLOBAL_BPROC void Battery_check_bat_clear_on_and_error(void);
GLOBAL_BPROC unsigned char Battery_check_bat_is_bat_error(void);
GLOBAL_BPROC unsigned char Battery_check_bat_get_bat_error(void);
GLOBAL_BPROC void Bttery_check_bat_init(void);
GLOBAL_BPROC bool Battery_check_bat_process(void);
GLOBAL_BPROC void Battery_check_bat_error(unsigned char bms_index);


GLOBAL_BPROC void Battery_timer_count_cb(void);
GLOBAL_BPROC unsigned char Battery_process_get_state(void);
GLOBAL_BPROC void Battery_Process_Init(void);
GLOBAL_BPROC void Battery_Process(void);
GLOBAL_BPROC unsigned short Battery_get_reg_value(unsigned char bms_index, unsigned short reg_index);
GLOBAL_BPROC bool Battery_get_bat_desc(unsigned char bms_index, st_version_desc * version_desc);
GLOBAL_BPROC unsigned int Battery_get_oldest_record_index_number(void);
GLOBAL_BPROC unsigned int Battery_get_newest_record_index_number(void);
GLOBAL_BPROC void Battery_get_bat_ID(unsigned char bms_index, unsigned char bat_ID[6]);
//����ֵ�ľ�����0.1V
GLOBAL_BPROC unsigned short Battery_get_max_chg_voltage(unsigned char bms_index);
//����ֵ�ľ�����0.1A
GLOBAL_BPROC unsigned short Battery_get_max_chg_current(unsigned char bms_index);
//����ֵû��ƫ�ƣ�����С��0���϶ȵĻ�ͳһ����0
GLOBAL_BPROC unsigned short Battery_get_bat_temp(unsigned char bms_index);
GLOBAL_BPROC short Battery_get_bat_lowest_temp(unsigned char bms_index);
//����ֵ��0.1mA/hΪ��λ
GLOBAL_BPROC unsigned short Battery_get_bat_rated_cap(unsigned char bms_index);
//����ֵ��0.1VΪ��λ
GLOBAL_BPROC unsigned short Battery_get_bat_rated_volt(unsigned char bms_index);

GLOBAL_BPROC unsigned short Battery_get_voltage(unsigned char bms_index);

GLOBAL_BPROC unsigned short Battery_get_TCurrent(unsigned char bms_index);

GLOBAL_BPROC unsigned short Battery_get_SOC(unsigned char bms_index);

GLOBAL_BPROC unsigned char Battery_get_cell_numbers(unsigned char bms_index);

GLOBAL_BPROC unsigned short Battery_get_switch_state(unsigned char bms_index);
GLOBAL_BPROC void Battery_clear_switch_state(unsigned char bms_index, unsigned short on);
GLOBAL_BPROC bool Battery_is_discharge_enable(unsigned char bms_index);
GLOBAL_BPROC void Battery_set_switch_state(unsigned char bms_index, unsigned short on);
GLOBAL_BPROC void Battery_clear_reg_valid_flag(unsigned char bms_index);
GLOBAL_BPROC void Battery_clear_reg_valid_flag_on_version_info(unsigned char bms_index);
GLOBAL_BPROC void Battery_set_reg_need_flashed(unsigned char bms_index, unsigned char flag);
GLOBAL_BPROC bool Battery_is_reg_need_flashed(unsigned char bms_index, unsigned char flag);
GLOBAL_BPROC void Battery_set_reg_valid_flag(unsigned char bms_index, unsigned char flag);
GLOBAL_BPROC unsigned char Battery_get_reg_valid_flag(unsigned char bms_index);
GLOBAL_BPROC unsigned char Battery_is_chg_current_false(void);
GLOBAL_BPROC void Battery_bat_chg_current_false_flag_clear(void);
GLOBAL_BPROC void Battery_bat_chg_current_false_timer_reset(void);
GLOBAL_BPROC void Battery_checking_mode_on_bat_in_by_HW_init(void);
GLOBAL_BPROC bool Battery_is_bat_in_via_IO_switch(void);
GLOBAL_BPROC bool Battery_is_checking_mode_on_bat_in_by_HW(void);
GLOBAL_BPROC void Battery_chg_hw_ctrl_init(void);

GLOBAL_BPROC void Battery_event_to_stop_chg_param_init(void);
GLOBAL_BPROC unsigned int Battery_get_bat_reg_state_when_event_to_stop_chg(unsigned char* p);
GLOBAL_BPROC void Battery_set_event_to_stop_chg_flag(enum_src_of_stop_flag src );
GLOBAL_BPROC bool Battery_is_event_to_stop_chg_flag_be_set(enum_src_of_stop_flag src );
GLOBAL_BPROC void Battery_clear_event_to_stop_chg_flag(void);
GLOBAL_BPROC void Battery_set_event_to_stop_chg_flag_disable_flag(unsigned int flag);
GLOBAL_BPROC void Battery_clear_event_to_stop_chg_flag_disable_flag(unsigned int flag);
GLOBAL_BPROC unsigned int Battery_get_event_to_stop_chg_flag_disable_flag(void);
GLOBAL_BPROC unsigned int Battery_get_src_of_stop_flag(void);
GLOBAL_BPROC unsigned char Battery_get_event_to_stop_chg_flag(void);
GLOBAL_BPROC void Battery_set_event_to_stop_chg_bat_ID(unsigned char * p_id);
GLOBAL_BPROC void Battery_event_to_stop_chg_bat_ID_compare_and_set(unsigned char* p_in_id);
GLOBAL_BPROC void Battery_bat_fault_debounce_param_init(void);
GLOBAL_BPROC void Battery_bat_fault_debounce_param_timer_cb(void);
GLOBAL_BPROC bool Battery_is_bat_fault_debounce_valid(unsigned char nb);
GLOBAL_BPROC void Battery_bat_fault_debounce_process(void);


GLOBAL_BPROC unsigned char Battery_is_bat_chg_enable_by_temp(void);
GLOBAL_BPROC void Battery_set_bat_temp_value_reflash_flag(void);
GLOBAL_BPROC unsigned char Battery_get_bat_temp_value_reflash_flag(void);
//��¼�����ص�����¶ȣ��ز���ʱ�����
GLOBAL_BPROC void Battery_set_bat_temp_value_when_plus_in(void);
GLOBAL_BPROC short Battery_get_bat_temp_value_when_plus_in(void);
//��ȡ��ǰ��ص�����¶�
GLOBAL_BPROC short Battery_get_bat_current_lower_temp(void);
GLOBAL_BPROC void Battery_clear_counter_of_temp(void);
GLOBAL_BPROC void Battery_low_temp_param_reset(void);
GLOBAL_BPROC void Battery_bat_low_temp_process(void);
GLOBAL_BPROC void Battery_bat_low_temp_ctrl_calc(st_bat_low_temp_ctrl_param *bat_low_temp_ctrl_param);
GLOBAL_BPROC unsigned char Battery_get_charger_setting_voltage_offset(void);
void Battery_clear_charger_setting_voltage_offset(void);
GLOBAL_BPROC void Battery_set_charger_setting_voltage(unsigned short v_value);
GLOBAL_BPROC void Battery_set_charger_setting_current(unsigned short a_value);
GLOBAL_BPROC void Battery_charger_setting_voltage_offset_init(void);
GLOBAL_BPROC void Battery_charger_setting_voltage_offset_process(unsigned short bat_voltage,unsigned short bat_current);


GLOBAL_BPROC unsigned int Battery_history_record_get_real_index(unsigned int index);
GLOBAL_BPROC unsigned int Battery_history_record_get_real_diff_value(unsigned int begin_index, unsigned int end_index);
GLOBAL_BPROC void Battery_history_record_param_init(void);
GLOBAL_BPROC bool Battery_history_record_is_bat_ID_same(unsigned char bat_ID[6]);
GLOBAL_BPROC bool Battery_history_record_get_data(unsigned char bat_ID[6], unsigned char buf_out[128]);
GLOBAL_BPROC void Battery_history_record_set_data(unsigned char buf_out[128]);
GLOBAL_BPROC void Battery_history_record_set_index_param(unsigned char bat_ID[6], unsigned int log_head, unsigned char datetime[6], unsigned int log_cnt, unsigned int log_read_cnt);
GLOBAL_BPROC bool Battery_history_record_is_data_updated(void);
GLOBAL_BPROC void Battery_history_record_clear_updated_flag(void);
GLOBAL_BPROC void Battery_history_record_set_updated_flag(void);
GLOBAL_BPROC bool Battery_history_record_is_within_range(unsigned int range_begin, unsigned int range_end, unsigned int data);
GLOBAL_BPROC bool Battery_history_record_is_index_within_range(unsigned int index);
GLOBAL_BPROC bool Battery_history_record_is_need_to_read_record(void);
GLOBAL_BPROC unsigned char Battery_history_record_state_of_process(void);
GLOBAL_BPROC void Batter_history_record_set_nfc_cmd_interval_cnt(unsigned int cnt);
GLOBAL_BPROC void Battery_history_record_param_reset(void);
GLOBAL_BPROC void Battery_history_record_process(void);
GLOBAL_BPROC signed char Battery_history_record_datetime_compare(unsigned char *DT1, unsigned char *DT2);

#endif // MID_CONTROL_PROTOCOL_H__

/** @} */

