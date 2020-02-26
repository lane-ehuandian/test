

/**@file
 *
 * @defgroup app_fifo FIFO implementation
 * @{
 * @ingroup app_common
 *
 * @brief FIFO implementation.
 */

#ifndef COMM_BOARD_PROTOCOL_H__
#define COMM_BOARD_PROTOCOL_H__
#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>

#include "gd32f403.h"
#include "app_fifo.h"

#ifndef NULL
#define NULL	0
#endif



#ifndef COMM_BOARD_PROTOCOL_C
#define GLOBAL_CMP extern
#else
#define GLOBAL_CMP
#endif


 
#define APP_ERROR_BASE_NUM      (0x0)       ///< Global error base
 
#define APP_SUCCESS                           (APP_ERROR_BASE_NUM + 0)  ///< Successful command
#define APP_ERROR_INTERNAL                    (APP_ERROR_BASE_NUM + 1)  ///< Internal Error
#define APP_ERROR_NO_MEM                      (APP_ERROR_BASE_NUM + 2)  ///< No Memory for operation
#define APP_ERROR_NOT_FOUND                   (APP_ERROR_BASE_NUM + 3)  ///< Not found
#define APP_ERROR_INVALID_PARAM               (APP_ERROR_BASE_NUM + 4)  ///< Invalid Parameter
#define APP_ERROR_INVALID_STATE               (APP_ERROR_BASE_NUM + 5)  ///< Invalid state, operation disallowed in this state
#define APP_ERROR_BUSY                        (APP_ERROR_BASE_NUM + 6) ///< Busy
 
#define BUFFERSIZE          256
#define BUFFERSIZE_OF_COMM_UART 512


 
 typedef enum{
     MASTER_RS485_RUN_TX,
     MASTER_RS485_RUN_TX_WAIT_COMPLETE,
     MASTER_RS485_RUN_RX        
 }rs485_run_state_t;
 
 typedef enum{
     MASTER_RS485_RESET,
     MASTER_RS485_RUN,
     MASTER_RS485_SLEEP
 }rs485_state_t;
 
 typedef enum{
     CHG_STA_ENTER,
     CHG_STA_PROCESS_BAT1,
     CHG_STA_PROCESS_BAT2,
     DISCHG_STA_ENTER,
     DISCHG_STA_PROCESS_BAT1,
     DISCHG_STA_PROCESS_BAT2,
     SLEEP_STA_ENTER,
     SLEEP_STA
 }app_state_t;
 
 
 
 typedef struct{
     uint8_t rs485_tx_buf[BUFFERSIZE];
     int16_t rs485_tx_indx;
     bool rs485_tx_buf_busy;
 }rs485_tx_t;
 
 
 typedef struct{
     uint8_t rs485_rx_buf[BUFFERSIZE];
     int16_t rs485_rx_indx;
     bool rs485_rx_sta_head;
     bool rs485_rx_sta_tail;
     bool rs485_rx_timer_to;
 }rs485_rx_t;
 
 typedef enum{
     POLL=0,
     CHARGE=1,
     DISCHARGE=2,
     SLEEP=3
 }slave_rs485_cmd;
 
 typedef struct{
     uint8_t system_addr;
     slave_rs485_cmd command;
 }slave_command_t;
 
 
 typedef struct{
     uint8_t sp_bat_on_site_cnt;
     uint8_t sp_bat_sta;
     uint16_t sp_bat_voltage;
     int16_t sp_bat_current;
     uint8_t sp_bat_soc;
     int8_t sp_bat_temperature;
     uint8_t sp_bat_fault;
     uint16_t sp_cycle;
     uint32_t sp_id;
 }sp_battery_t;



 typedef struct{
 unsigned char data_valid_flag; //buf�е������Ƿ���Ч��0��Ч��1��Ч
 unsigned char data_head[3];
 unsigned char data_buf[128];
 unsigned char data_crc16[2];
 unsigned int data_offset; //����ƫ��
 }st_update_buf;
 
 GLOBAL_CMP st_update_buf gl_update_buf;


 typedef struct{
//    unsigned char stl_ctrl;
    unsigned char stl_cmd;
    unsigned char stl_dataL;
    unsigned char *stl_data;
}st_hub_cmd;
//#define HUM_FRAME_INDEX_CTRL 0
#define HUM_FRAME_INDEX_CMD 0
#define HUM_FRAME_INDEX_DATAL 1
#define HUM_FRAME_INDEX_DATA_START 2


/*
0x00	�ɹ���
0x01	����ʧ�ܡ�
0x02	������ʱ��
0x03	�豸δ׼���á�
0x04	��֧�ֵ����
0x05	Ӳ������
0x06	��������
0xFF    	δ֪����
*/

typedef enum{
    ENUM_COMM_ERROR_CODE_OK,
    ENUM_COMM_ERROR_CODE_FAILED,
    ENUM_COMM_ERROR_CODE_TIMEOUT,
    ENUM_COMM_ERROR_CODE_NOTREADY,
    //ENUM_COMM_ERROR_CODE_NOTSUPORT,
    //ENUM_COMM_ERROR_CODE_HW,
    //ENUM_COMM_ERROR_CODE_PARAM,
    
    ENUM_COMM_ERROR_CODE_UNKNOWN = 0xFF,
}enum_comm_error_code;

typedef enum{
    ENUM_COMM_INDEX_COMM,
    ENUM_COMM_INDEX_CHG,
	ENUM_COMM_INDEX_MASTER,
    ENUM_COMM_INDEX_MAX
}enum_comm_index;


//����λ��ͨѶʱ����д�������ݵĵط����ø�����ʱ�䣬�������ʱ�����ٷ�����Щ����
#ifdef COMM_BOARD_PROTOCOL_C
GLOBAL_CMP unsigned int slave_rs485_comm_TX_delay_cnt_max = 0;
#else
GLOBAL_CMP unsigned int slave_rs485_comm_TX_delay_cnt_max;
#endif
#define SLAVE_RS485_COMM_TX_DELAY_CNT_MAX_SHORT 110 
#define SLAVE_RS485_COMM_TX_DELAY_CNT_MAX_LONG 5000 


GLOBAL_CMP void slave_rs485_dev_addr_init(void);
GLOBAL_CMP unsigned char slave_rs485_get_dev_addr(void);
GLOBAL_CMP bool slave_rs485_data_handler_new_protocol(void);
GLOBAL_CMP bool slave_rs485_cmd_snd_bat_info(void);
GLOBAL_CMP void slave_rs485_cmd_timer_cb(void);
GLOBAL_CMP bool slave_rs485_cmd_process(st_hub_cmd *pl_hub_cmd);



GLOBAL_CMP void slave_rs485_cmd_update_cnf(unsigned char target, unsigned char cmd, enum_comm_error_code error_code);
GLOBAL_CMP void slave_rs485_cmd_update_return_error(unsigned char bms_index, unsigned char cmd, enum_comm_error_code error_code);

GLOBAL_CMP void slave_rs485_get_rx_data_from_buf(enum_comm_index index, unsigned char** data_out, unsigned int *data_len);
GLOBAL_CMP void slave_rs485_cmd_bypass_cnf(unsigned char *data, unsigned int data_len);







#include "list.h"

#pragma pack(1)

#define MASTER_485_CMD_POLL_ERROR_CNT_MAX 2

typedef struct struct_mater_state{
//���ID�ź��Ѿ����ˣ���ر�ID�źŵĶ������ӳ�60��
unsigned int ID_close_delay_cnt;
//�����У��ϵ��ӳ�10�����ϱ��������485����״̬(Fault128�е�һ��)
unsigned int charger_com_err_report_delay_cnt; 
//��ʱ���Ͳ�ѯ������������������
unsigned int poll_charger_cmd_cnt;
//����͵ĳ�ʱ����
unsigned int cmd_timeout_cnt;
//���շ�������󣬷�����һ�������ʱ����
unsigned int next_cmd_interval_cnt;
//�յ������������ʼ�ͽ����������60���ڲ�������������������������ݡ�
//�ȵ����յ��������������������󣬸�Ϊ3���ڲ�������������������������ݡ�
unsigned int normal_cmd_delay_when_chger_updating_cnt;
//�Ƿ����ڴ�������������̣�ֱ��ɾ�������е����������
unsigned char cmd_processing_flag;
//����ͳ�ʱ��־
unsigned char cmd_snd_faild_flag;
//һ�������ظ����ʹ�������
unsigned char cmd_snd_faild_cnt;
//�Ѿ�����ʧ�ܵĴ�������
unsigned char all_cmd_snd_faild_cnt;
//poll ����485ͨѶ���ϼ���
unsigned char chg_485_cmd_poll_faild_cnt;
//���յ�������ָ�������ֹͣ�������磬ֱ����λ���ٷ����������ָ��
unsigned char chger_enable_on_unlock_cmd_flag;
}st_mater_state;


typedef enum{
    ENUM_M_CMD_NULL,
    ENUM_M_CMD_POLL,
    ENUM_M_CMD_GET_DEV_VERSION = 0x20,
    ENUM_M_CMD_UPDATE_START = 0x21,
    ENUM_M_CMD_UPDATE_EXCHANGE = 0x22,
    ENUM_M_CMD_UPDATE_DONE = 0x23,
    ENUM_M_CMD_MAX
}ENUM_MASTER_CMD;

#pragma pack()

typedef struct struct_master_cmd{
    struct list_head list;
    //Ҫ���͸������������
    ENUM_MASTER_CMD cmd;
    //�����������ָ��
    unsigned char *data;
}st_master_cmd;

#define MASTER_CMD_NODE_MAX 3

#pragma pack(1)


typedef struct{
//���ʹ�ܡ�0x00:��ֹ��磻0x01:ʹ�ܳ�磻0x02:ʹ�ܼ��ȹ���
unsigned char chg_en;
//��ǰ��������ѹ����λ0.1V
unsigned char expect_voltage[2];
//��ǰ��������������λ0.1A
unsigned char expect_current[2];
//��ǰ����¶�
unsigned char bat_temp;
//�����������λ0.1mAh��
unsigned char rated_cap[2];
//��Ƶ�ѹ����λ0.1V��
unsigned char rated_voltage[2];
unsigned char reserve[6];
}st_poll_cmd_param;

//�����������ȫ�ֱ������ơ�
//����һ���洢������ȫ�ֱ���
GLOBAL_CMP st_poll_cmd_param gl_poll_cmd_param;


typedef struct{
//����������λ0.1A��
unsigned char chg_current[2];
//����ѹ����λ0.1V
unsigned char chg_voltage[2];
//��繦�ʣ���λ0.1W
unsigned char chg_power[2];
//Bit0��Charger��������
unsigned char charger_OAP_flag:1;
//Bit1��Charger��ѹ����
unsigned char charger_OVP_flag:1;
//Bit2��Charger���±���
unsigned char charger_OTP_flag:1;
//Bit3��ChargerǷ�����ڸ���ѹ�׶Σ�Charger����ʼ���޷�����300MA
unsigned char charger_UAP_flag:1;
//Bit4��Charger Ƿѹ������ѹ�׶Σ�3S����Charger��ѹʼ���޷���ѹ��20V
unsigned char charger_UVP_flag:1;
//Bit5��Ԥ��
unsigned char bit5_reserv:1;
//Bit6������ѹƥ�䡣
unsigned char charger_voltage_match_flag:1;
//Bit7��������ƥ�䡣
unsigned char charger_current_match_flag:1;
//V1.9�汾�Ժ��Э��
unsigned char charger_ID_flag:1;
unsigned char charger_fan_error_flag:1;
unsigned char charger_low_AC_supply_flag:1;
unsigned char charger_byte8_reserve:5;
//������¶�ֵ����λ���϶ȣ���Χ(-40~214)����ֵ��Ҫ��40����ʵ��ֵ��0xFF��ʾ��Ч
unsigned char charger_temp;
unsigned char charger_byte10_reserve;
}st_poll_cmd_cnf_param;
GLOBAL_CMP st_poll_cmd_cnf_param gl_poll_cmd_cnf_param;


//��¼���������״̬�����������ϴ�
typedef struct{
unsigned char is_need_update; // �Ƿ���Ҫͨ�������ϴ���������
unsigned char charger_err[2]; // ��Ӧ��������ص����������ֽ�
}st_charger_err_status;
GLOBAL_CMP st_charger_err_status gl_charger_err_status;


typedef struct{
//Ӳ�����汾��
unsigned char HW_main_ver;
//Ӳ���Ӱ汾��
unsigned char HW_sub_ver;
//�̼����汾��
unsigned char FW_main_ver;
//�̼��Ӱ汾��
unsigned char FW_sub_ver;
//�̼��޶��汾
unsigned char FW_revised_ver;
//�̼�build��32
unsigned char FW_build_ver[4];
}st_version_desc;

typedef struct{
    unsigned char Protocol_version;
    unsigned char dev_type;
    unsigned char Port_max;
    st_version_desc PMS_version;
}st_dev_info;


typedef struct{
    unsigned char is_valid; // 1,��ʾ����İ汾��Ϣ����Ч��
    st_version_desc version_desc;
    unsigned char charger_sn[14];
}st_charger_dev_version;
GLOBAL_CMP st_charger_dev_version gl_charger_dev_version;

typedef struct{
    unsigned char charger_test_enable_flag:1;
    unsigned char charger_id_enable_flag:1;
    unsigned char charger_cmd_enable_flag:1;
    unsigned char reserve_bits:4;
}st_charger_test_param;



#ifdef COMM_BOARD_PROTOCOL_C
/*�汾˵��
V12: 
���ӷ��Ȳ�������;
LED����ɫ���ж������޸ģ���ԭ����< ��Ϊ<=���Ա����밲׿��APPͬ��
�����485ͨѶЭ���еķ��ظ�ʽ���޸ġ�
�����ID�򿪺������ص�����Ҫ�رգ����ӳ�60���ٹر�
V13:
����������£�����·������ŵ�����̵�ҲҪ��˸
V14:
�����485ͨѶ�����ϱ�
���Ӷ�ȡPMS������Ϣ�ĵ�������

V15:
����¶ȹ���ʱ�򣬲���ָʾ�ƵĹ���״̬���ٰ�������¶ȹ��ϡ�
V16:
�����485ͨѶ�������β���Ϊ�д���
V17:
��������ʱ��Ӵ�6.5��

V18: 
NFC��ʱͨѶ���Ա�����ز�����ĵ硣
�����ص�ѹ����ĳֵ������ѯ���ʱ��Ӵ�
���Ҫ�������Ϊ0����������ԭ��Ҫ�������Ϊ0����رճ����
����SOHֵ

V19:
���ڷ����ڵ�شﵽ�¶ȾͿ�ʼת�������ж��Ƿ����ڳ�������

V20:
1. ���������ȡIO״̬��LED������ELOCK����������������������Ȳ�����
  ��ؼ����ѯ����
2. �޸ĵ�ؼ����ѯ��ʱ��BAT_POLL_ON_TIME_MAXΪ8��
3. �ϴ���fault�Ӳ��ܳ�����ʾ
4. �޸��쳣��ʾfault64��bug(�������쳣��Bug)

V21:
1. �޸�����slave_rs485_communication_init�еĳ�ʼ��BUG��

V22:
1. �жϼ����ѯ�Ĵ�������BUG������ɾ�����Դ���
2. ��λ�������ֹ���ʱ��Ҳ��������ѯ���
3. �����ѯ���ʱ�����û����ͨѶ״̬��NFC�����ߣ����߲��������ʡ�
    ԭ��ֻ�ǲ������ݣ����������й��������
4. ���β��������΢������IO�ڶ�ȡ�жϹ��ܡ�
5. ���Ӳ������ת����ʱ���ܡ�

V23: 
1. ���ű仯ʱ��ǰ��30����Ҫһֱ��ѯ(BAT_POLL_DELAY_MAX)
2. ��λ������������ţ���ʱ��ȷ���������ܴ��ڿ���״̬��
    (������������֮���⵽���ŵĻ�������)
3. �ϵ����������������Ĭ�Ϲر�״̬��

V26: 
1. ���Ӽ���ع��Ϲ��ܡ�(Masked)
2. ֧�ֳ����Զ���������(Done)
3. ��Ϊ����磺����ط�������н��г��ʱ��
    �ȴ򿪵�صĳ�ŵ�ܣ�����ʱ�򿪳������(Done)
4. ��Ϊ��ȡ�磺������ʱ���ȹرճ�����������
    ����ʱ�رյ�صĳ�ŵ�ܡ�
5. �޸�BUG: ������תʱ�򣬻�Ӱ��������������߼���
    (���:���ת��ʱ�������ǲ���������)
6. ��ؼĴ���ֵ��0xFFFF�����Чֵʱ���ʵ�����(Done)
7. ���: ������յ������ݰ�����ʱ������Ӧ���Ӱ�쵽�����ֵ����ݽ��ա�(Done)
8. ���������XMODEM ģʽ����ʱʱ���Ϊ26��BMS_XMODEM_TOTAL_TIMEOUT_MAX�� (Done)
9. ���Ź���Ϊ4�볬ʱ��(Done)
10. PMS��ַ�Ķ�ȡ���ڳ���ʼ��ʱ���Ժ�������ʹ��PMS��ַ��
    ֱ��ȡ�����д洢�ĵ�ַ�����ٴ�IO���ж�ȡ��
11. ��������س�����ȡ��ذ汾��ʱ���п��ܻ�ȡ������Ϣ���������޸����ơ�    

V27:
1. �¶ȵ���Чֵ��Ϊ0xFF��
2. ���SOC�ڼ�ⲻ�����ʱ���ϴ���ֵΪ��Чֵ0xFF��

V28:
1. �յ����ܳ��Ĺ����¼���ֹͣ����־��λ��ֱ�����Ŵ򿪻��߼�⵽���ID�б仯��
        (�����κ���������������ñ�־)

2.       ���������-��ص���>Ԥ��ֵ�����ٺ�����
3.       ���ʱ���κε�ش������¶ȴ���Ԥ��ֵ��70�ȣ�
4.       ���Ԥ����ʧ��(��Ҫ������������Ч��������ж�)
6.    ������������ʱ������⵽��ص�ѹ����26V��13S����32V��16S)�������������
7.    PMS��ص���ڲ���ѹ������⵽��ص�ѹ�ﵽ������ѹ�󣬳�������δ���ͣ�
            ����ʵ�����������һ�£�ͬʱ��ص�ѹ��Ȼ���ӣ���ʱ��Ͽ�������Ĺ���
            
8.    ��ȡ����ع������-------------- (ԭ�����߼������ж�)


12.   ���ŵƲ��ò�ͬ����˸ģʽ������Ϻ�������(�ڲ���ʱ����BUG��
        �ֶ�������أ�����ʧ�ܷ��ء�LED�������̵ơ���ʱ��ȥ��򿪲��ţ�
        �п��ܵƲ����ˣ���Ҳ������
       ԭ��:  �����Ƕ�ʱ���ص�����û�б����á�)

13. ��֤������ȹرղŹص�ء�������ܼ�⵽������رգ�4�볬ʱ���Թص�ء�

15. �޸ģ����жϹ��ϵ��ʱ�򣬹�ѹ���ϲ������Ρ�

V29:
1. �޸ģ����������-��ص���>Ԥ��ֵ����Ϊ800������

V30:
//5. ������һ��ʱ���20���ӣ�����ص�ѹ��SOCû������Ԥ����

---. ���Ӽ���ع��Ϲ���:(��û����ϸ����) (�����˲���ѭ��)
9. ���MOSʧЧ�����Ӽ����MOSʧЧ��
11. ���ŵ�MOSʧЧ�����Ӽ��ŵ�MOSʧЧ��

//10. PMS���ֳ�������ܿ��ƣ��ϱ����ϸ����ư塣

14. ������������mA���ϣ�����ʱ�䳬��10�롣�رճ�硣

15. �����ؼĴ����е������־����λ��Ӧ��ֹͣ��粢�رճ�����͵�س�ŵ翪�ء�
16. ��ػ�����λ���ĳ���������Ϊ0ʱ����ԭ��485���͹س��Ļ����ϣ��ٹس����ID��
17. ͸����������ʱ����ʱȡ�������ѯ��ʱ��180�롣
18. �����������ʱ����Ҫ����ơ���3�º��
19.��ع��Ϲ��ֱ࣬���û�ȥά�޷����ĺ�Ƴ�����
        ����Ҫ��ϸ��Ϣ�ж��Ƿ��ûص���2�º�ơ�
20.���������ȡ��ص���Ҫ��Ϣ�����ڵ�صĹ��Ϸ�����
21. NFC��ʧ�жϵĴ������޸ġ��ڲ��Źر�״̬�£�������30���ڲ��Ŵ�״̬�£�������8��
//22. fault bit3 ��ʾ�����жϼ̵�����
23. �Զ�������0x72 ���ӳ�������ص���Ϣ
24. ����΢�����ص��źŷ��ڵ����Ϣ�����е�damage�ֽ�bit4��
    û�м�⵽ʱ������Ϊ1����⵽����Ϊ0��
25. �ڲ��ſ���״̬�£�ֻ��΢�����ؼ�⵽��ز��ϴ������Ϣ��
26. �ڲ��Źص�״̬�£����΢������һֱû�м�⵽��أ����ϴ������Ϣ��
     ���΢�����ؼ�⵽��أ����Ժ���û��΢�����ع���ǰ���߼��ϴ������Ϣ��
27. �޸�����0x24���������õ�ѹֵ��
    ���Ҹ��ݸõ�ѹ���Ƴ������ѹ���õ�ѹֵ���������ѹֵ�Ƚϣ�ȡС�ġ�
28. �޸�������������ݳ��ȵļ����ԡ�
29. Զ��������ϸ澯��

����:
1. 3.2V���ϣ�SOC��0������Ŀ��������С��޸�:��ʱ��Ϊ3.8V���ж�
2. ΢������ȥ��
3. ��ػ������ش������10���Ӻ��Զ�����
4. ��������س�ŵ��ʱ�򣬳�ʱ��0x40���ϡ�(��Ϊ5���ӳ�ʱ)
5. ��UVP���ϣ����ǵ��SOC���ߵ�ѹ��
6. ���ͬʱ��UVP��OVP����
7. �ָ����ֵ��¹���
8. ������⵽����ʱ�򱣴��ֳ����ݡ�
9. ��о��ѹ����2V������

V31:
1. ��������س�ŵ�ܹ��ϵĴ����޸ġ�(30�汾���Է���BUG)
    a. ��Ҫ���ź��������������⡣
    b. ��Ҫ���������ǰ�ȹرճ�����͵�س�ŵ�ܡ�
//2. ���ӹ��ܣ��ز���ʱ�������Ƿ�����������Ӻá�
3. ���������������PMS��

V32:
1. ���ӹ��ܣ��ز���ʱ�������Ƿ�����������Ӻá�
2. ȥ��30�汾�Ĺ���:    
    //25. �ڲ��ſ���״̬�£�ֻ��΢�����ؼ�⵽��ز��ϴ������Ϣ��
    //26. �ڲ��Źص�״̬�£����΢������һֱû�м�⵽��أ����ϴ������Ϣ��
    //     ���΢�����ؼ�⵽��أ����Ժ���û��΢�����ع���ǰ���߼��ϴ������Ϣ��
    ͬʱ�޸ĺ�����Ϊ���ز��ž�ת����30�汾��  �ز��ż���΢�����ز�ת����   
3. ���������ص�ѹ�ķ�ʽ���жϵ�ص��������Ƿ�Ӵ��á�
    ʹ�õ����Ϣ�����е�damage bit5���ϴ�״̬����λ����1 ��ʾû�нӴ��ã�0 ��ʾ�Ӵ��á�
4. 30�����жϵ�ز���磬�жϷ����ǣ�
    �����SOCС��90%ʱ�򣬿��Ƿ��г�����(����50mA)��
    �ù��Ϲ��ൽDamage bit6�ı�־�����ǲ���رյ�غͳ������һ���ָ���磬����������
5. �����жϵ�ع��ϵ�ȥ��ʱ��3�롣(�ڼ������¼�������)
6. �޸�fault bit6 (64)��ԭ��ԭ��������ԭ����: 1. ���ű��쳣�򿪣�2. ������ָ��򲻿����š�
    ���ڸ�Ϊֻ��һ��ԭ�����: ���ű��쳣�򿪡�
7. ��damage bit6 (64)Ϊ��������Ŀǰ�����Ĵ�����: 1. ������ָ��򲻿�����
8. �޸ı�����ֵ�WARNING��
9. ����0x82�����������ͽ�ֹ�����������Ե�ع��ϵĹ��ܡ�

V33:
1. ���������ɺ����ױ���ϵ����ϡ�
    ����Ϊ��ظ�������ɺ󻹴��ڲ��ȶ�״̬��
    ���ڴ���ʽ�ǣ����������ɺ��ӳ�120�ټ���ع��ϡ�
2. �����485������յ����غ��ӳ�20�����ٷ�����һ�����

V34:
1. �����ͨѶЭ����Poll���������Byte7 bit5���޸Ķ����ˣ����ڰ���Bit5�еĹ���Ҳֹͣ��硣
2.  H,M,L�����ϣ�����12s���ϱ�����λ��
3. �����жϵ�ع��ϵ�ȥ��ʱ���Ϊ8�롣(�ڼ������¼�������)
3. ������������߶�·���ϣ���Ҫ����(20��)Ȼ���ס�ϱ����ϡ�damage4 ��0x72��������⡣

V35:
1. ��о��ѹ�����0.1V�������ϡ�
    a. ��ߵ�о��ѹ����3.8V
    b. ��ߺ���͵�ѹ�����0.1V
    c. ����ʱ�����10��
2. ���ſ���ʱ�򣬲����ñ������Ĺ��ϱ�־��    

V36:
1. ��ʱ�ر�V35�汾�ļ���о��ѹ�����0.1V�Ĺ��ϡ�
2. PMS���������󣬲���Ĭ�ϼ��һ��"����Ƿ�����������Ӻ�"��(V32 �汾�е�1 ����)

V37:
1. �޸���������س�ŵ�ܵ�����������ԭ����س������š��������š�Ԥ����ʧ�ܳ�ʱ��Ԥ����ʧ���ⲿ����/��·��
��4���������������������������Ӹ�Ϊ������������
2. �޸ģ��������ʱ��Ӧ���ڿ�ʼ����ʱ���ȹرճ�������ȴ�6���ٹص�ء�

V38:
1. ���Σ�"���SOCΪ0������ص�ѹ��(3.8*��о����)V���ϣ����жϵ��������" �ù�����ʾ����
2. ���Σ�"��UVP���ϣ����ǵ��SOC��30%���ϣ����ߵ�ص�ѹ��(3.8*��о����)V����"�ù�����ʾ����

V39:
1. �Ż����������ʱ�򣬱����г��������

V40:
1. ����֧�ֵ�����Ŀ��ֿ��ơ�ͬʱ�޸���һ���жϲ���״̬���߼�
2. ���ݵ���¶Ⱦ����Ƿ���ͳ�����
3. �鿴��׿���·�������������ѹ
4. ���ӿ�����ǡ�"0x33"�����У�Lock status, Bit7���±�����Ϊ������־�����豸����������������������־��
5. ���Ÿչر�ʱ�������Ϊ�����е�أ��ӳ��ϱ��ز��ŵ�״̬��
�����Ϊ����û�е�أ������ϱ��ز��ŵ�״̬������:���жϲ��Źر�ȥ��ʱ��
����е�ؾ����ӳ�ʱ����ӳ�1�룬ͬʱ�����ŵĺ�����Ϊ���ж��ӳٺ�ÿ100mS��ѯһ�Ρ�

V41:
1. ��������⵽������Ӳ��õĹ��Ϻ��ظ��ټ���Ρ�

V42:
1. ֧�ֳ����ͨѶЭ��V1.9������¹��ϱ���
2. ��һ���汾����    "  2. ���ݵ���¶Ⱦ����Ƿ���ͳ�����"�� �������й��Ӷ����ݡ�
��Ҫ��λ����ϣ��·��Ƿ������ù��ܡ�����������־�����FLASH�е�0xFE00000��ʼ��ַ
3. �豸�������� �������������"PMS�Ѿ��������ı�־"
4. �ڼ����MOS����ʱ�򣬿��ܴ���ʱ�䲻����Ŀǰ�޸Ĵ�ʩ�ǣ�����20��ʱ�䡣
5. ������������ѹ���������⵼�������ѹƫ��ʱ���ʵ����߳���������ѹ��

V43:
1. �µ�Э���ж��������ݽ������޸ġ�
2. ֧��48V��صĳ�������9A
3. ���ӳ��������ģʽ��ָ��
4. �����յ��������������ж��¶��Ƿ�������Ҫ���ȷ�Χ�ڣ�����ǣ�
���ռ�������������������ǣ��������س�硣��Ӳ���汾�����ǲ���֧�ּ��ȹ��ܵ�Ӳ����
5. �޸ļ����ѯʱ��: ���ڵ���3.0V����1����ʱ�䣬Ȼ����ڵ���2.7V����10����ʱ�䣬Ȼ��С��2.7V����30����ʱ��
6. ������������Ϊ0ʱ�򣬹رյ�س�ŵ�ܡ�
7. ������Ŀ��������Ϊ200mS��(��Ҫע�⣬�ڳ����У�һ�����ֵ������Ƿ����ɹ�������)
8. ���ȹ��ܷ��������󣬹رյ�ؿ��أ��ر�������⹦�ܡ�
9. �޸�: ԭ�������־�������磬��ΪSOCС��100%���ܳ�硣
10. ����ģʽ�£�����е�������������������Ϊ�й��ϣ�ֹͣ���Ⱥͳ�磬�ϱ���λ��(damage bit6)��ֻ������PMS���ָܻ�������
11. ����������Ա�����(��������·����ѹ������)(ƽ��ÿ����0.5�Σ�����20����)��
    ���¼�ϱ��������ͣ����ϱ�����λ������������·����ѹ�������������磻���»����Գ�硣
12. �������������ʱ�򣬱���Ҫȷ���ó����״̬���ϴ�����λ��1�Ρ�
13. ����ģʽ���������ȹ��ܣ����ڹ������ԡ����Ӳ������ÿ��PMS������ֻ��ִ��һ�Ρ�
14. ����PMS�жϰ����¶��жϣ����ڵ���60���϶��������������öϵ��־��
15. �޸�PMS�жϰ����¶ȣ����ڵ���60���϶���ֹͣ��磬ͬʱ����Ҫ�ָ���55���϶�һ�²����ٴγ�硣
16. �޸ģ�����΢�����ص�ģʽ�£��������й��ϲ��ܷŵ磬���ϱ����"��طŵ�������״̬"���ϡ�

V44:
1. �޸�: �����챦�ɳ�����Ĳ����������(����ģʽ�Ĺ̼��汾��ΪV1.0.0.1001)

V45:
1. �޸�: �����챦�ɳ�����������ó���7A���������⡣ֻ���µĳ�����̼�(�����汾��)��֧���·�9A��硣

V46:
1. �޸�: ͨ��ʹ�ó��������طŵ��ѹ��ȷ�ϵ���������Ƿ����ӺõĹ��ܡ�
                ԭ����BUG: ��NFCͨѶ���õ�����£����ܻᵼ�µ�ؿ���״̬���������͹ص����ã�
                �Ӷ����¼�����̳���ֹͣ״̬���������޷�������ȷ�������

V1005:���Ȱ汾���޸��ж��챦�����֧���¾�Э��İ汾�ŵ�������
            ԭ����2019.3.15��ǰ�������Ǿɰ汾�����ڸ�Ϊ2019.3.20��ǰ�������Ǿɰ汾��
V100:���Ȱ汾���޸İ汾�ţ����APK�ɰ汾�ļ��Ƚ��ֹ��ܡ�2019-5-27

V47(V1006):
1. �޸Ĺ��ܣ���΢�����صĲ�ҲҪ����������������Ƿ����ӺõĹ��ܡ�

V48(V1007):
1. ���ӹ��ܣ������ϱ�SN��(PMS�ͳ����)

V49(V1008):
1. �޸�΢�����ؼ��͵�������������Ⱥ�˳��
��ǰ��ͬʱ��ʼ������һ������¶��ǵ���������������ɡ�
���ڸ�Ϊ��Ҫ�ȵ�΢�����ؼ����ɺ������������������⡣

2. ��Ϊ�°汾��׿��������3�λ���ʧ�ܺ���Ի���ɹ���
������汾ɾ����V43�汾��Ĺ���: 
"
16. �޸ģ�����΢�����ص�ģʽ�£��������й��ϲ��ܷŵ磬���ϱ����"��طŵ�������״̬"���ϡ�
"

V50(V1009):
1. BUG, �����������س����SN����λ������Ϊ������������ص�SN������ʱ��д����롣�����޸���

V51(V1010):
1. �޸����ϴ�������״̬�Ĳ����޸ġ�

V52(���Ȱ�V1011, ���Ȱ�V101):(ʹ�ø߼��Ż�����ѡ��)
1. ���ӵ����֤���ܡ������Ҫ��֤PMS�ǺϷ��Ĳ��ܴ򿪳�ŵ�ܡ�
PMS��Ҫ��֤����ǺϷ��Ĳ����������س�硣

V53(���Ȱ�V1012):
1. �޸����в�������"SLAVE_RS485_CMD_R_RUNING_PARAM"�еĴ�������
ԭ�����ص�"����PMS��֤BAT����"��־���ܻ����ҵģ������޸��á�

V56(���Ȱ�V1013):
1. �޸�FLASH��������������Ĵ��롣ԭ���в����汾����ʱ��
�����ṹ�������������Ҳ�ᱻ�������°汾�����С�
�����޸���ϡ�

V57:(���Ȱ�V1014)
1. �޸Ĺ��Ӽ��΢�����غ�����������״̬�Ĳ��ԡ�
    //1.1 �������رպ����̼�����Ƿ����ߡ���������ߣ����ټ��������������״̬��Ĭ���ϴ�������û�����Ӻõ�״̬��
    1.2 �������رպ����̼��΢�������Ƿ�λ������Ѿ���λ�����ټ��΢�������Ƿ�λ��Ĭ���ϴ�΢������û�е�λ��״̬��
    //1.3 ���Ź������ڼ��΢�������ڼ��ص��ߡ�����м���ֵ��߻���ż�����ߣ����ټ��������������״̬��Ĭ���ϴ�������û�����Ӻõ�״̬��
    1.4 ���Ź������ڼ��΢�������ڼ��ػ�����ⲻ������������м���ֵ������źŻ���ż���������źţ����ټ��������������״̬��Ĭ���ϴ�������û�����Ӻõ�״̬��
2. �޸ļ����ѯʱ��: ���ڵ���3.0V����1����ʱ�䣬Ȼ����ڵ���2.7V����24Сʱʱ�䣬Ȼ��С��2.7V����72Сʱʱ��
3. ����������¶Ȳ���
4. �ز��ź�ı��������ѯʱ���Ϊ1���ӣ�ԭ����30���ӡ�

V58:(���Ȱ�V1015)
1. �޸�Э���е��������ݡ�
2. ������������У����һֱ��һ��ѭ��������������5�����к�ᱨ���ϡ����ڸ�Ϊ�����˸ù��Ϻ��ٹ�30����PMS�Զ�������

V59:(���Ȱ�V1016)
1. ��ȡ�����ʷ��¼
2. BUG: ��������������е�ط������ϣ�����ʧ�ܡ��޸���ɡ�
3. ������������֤״̬�ı�־λ
4. �����͵�о��ѹ����3.9Vʱ�򣬱���һֱ��ѯ��
5. PMS����ʱ���ӳ�����󲽽����ת�����������õ�485��ַ�����ӳ�ʱ�䡣
6. PMS����ʱ�򣬲��ڷ����ӳ������������ӳ�ʱ����15�롣
7. ��������·�еĵ���ֵ��Ȼ���жϳ���Ƿ��ǽӴ�������
8. PMS����ʱ�����ױ���������ϡ������޸ģ�����10���Ժ�������������ϴ�������Ĺ��ϱ�־fault128.
9. ����ư�ͨѶ��485�ڵĽ��ջ�������Ϊ512�ֽڣ�ԭ����256�ֽڡ�

V60:(���Ȱ�V1017)
1. �������"��ȡ��ʼ�����ʷ��¼��źͼ�¼����"
2. ��ص͹��Ĺ��������ѯ��ʱ������޸ģ���Ϊ������3.9Vʱ�򱣳�һֱ��ѯ��
���ڵ���3.0V����1����ʱ�䣬Ȼ����ڵ���2.9V����24Сʱʱ�䣬Ȼ��С��2.9V����Լ1193Сʱʱ��(4294967��)

3. ��ص͹��Ĺ��������ѯ�ڼ䣬��о��͵�ѹС��3V����ֹBMS����
4. ��ص͹��Ĺ��������ѯ�ڼ䣬��о��͵�ѹС��3V����ֹ��ȡBAT��ʷ����
5. ��ص͹��Ĺ��������ѯ�ڼ䣬��о��͵�ѹС��3V����ֹPMS��֤���
6. PMS��֤��صĹ��ܣ��Զ����򿪣����ǣ���֤�����Ӱ���硣
7. ��������е���������ҵ���е�������ʱ�����damage32(������δ���Ӻ�)��־

V61:(���Ȱ�V1018)
1. ������TVS�¶�ֵ��BUG�����λ�������ϴ���ֵ�ܴ��ʱ�����160�������Ѿ��޸���ϡ�

V62:(���Ȱ�V1019)
1. �����е�����¶������ٶ�ѡ������޸ġ���Ϊ�ȽϾ���ֵ��

V63:(���Ȱ�V1020)
1. BUG�޸�: ��ص͹��Ĺ��������ѯ�ڼ䣬ͬʱ��о��͵�ѹС��3V����ֹBMS����
2. BUG�޸�: ��ص͹��Ĺ��������ѯ�ڼ䣬ͬʱ��о��͵�ѹС��3V����ֹ��ȡBAT��ʷ����
3. BUG�޸�: ��ص͹��Ĺ��������ѯ�ڼ䣬ͬʱ��о��͵�ѹС��3V����ֹPMS��֤���

V1021(���Ȱ�)
1. BUG�޸�: ���״̬�л�������״̬ʱ�򣬻���ֵ�ض�ʧ�������Ѿ��޸���
2. BUG�޸�: ��Ϊ���ȹ����м�����е����������Ĺ��ϡ�
    ԭ��ֻ���1���ӣ����ڸ�Ϊ: ��������3A�ͼ��1S����������1A�ͼ��3S���е�����6S��
*/
st_version_desc gl_version_desc_PMS = {
    #ifdef CABINET_WARMOR_SUPPLY_BY_CHARGER
    //Ӳ�����汾��
    0x01,
    //Ӳ���Ӱ汾��
    0x01,
    #else
    //Ӳ�����汾��
    0x01,
    //Ӳ���Ӱ汾��
    0x00,
    #endif
    //�̼����汾��
    0x01,
    //�̼��Ӱ汾��
    0x00,
    //�̼��޶��汾
    0x00,
    //�̼�build��32
    #ifdef CABINET_WARMOR_SUPPLY_BY_CHARGER
    {0xFD,0x03,0,0}
    //{0x65,0x00,0,0}
    #else
    #ifdef BAT_WARMOR_BY_SELF_ENABLE
    {99,99,0,0}
    #else
    {63,0,0,0}
    #endif
    #endif
};
#else
GLOBAL_CMP st_version_desc gl_version_desc_PMS;
#endif




enum{
    ENUM_HUB_PROTOCOL_CTRL_UNSEGMENT,
    ENUM_HUB_PROTOCOL_CTRL_START,
    ENUM_HUB_PROTOCOL_CTRL_END,
    ENUM_HUB_PROTOCOL_CTRL_CONTINUE,
};



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
}st_bat_data;




typedef struct {
unsigned char cmd;
bool (*cmd_process_func)(st_hub_cmd *);
}st_slave_rs485_cmd;

enum{


SLAVE_RS485_CMD_DEV_VERSION = 0x20,

SLAVE_RS485_CMD_UPDATE_START = 0x21,
SLAVE_RS485_CMD_UPDATE_EXCHANGE = 0x22,
SLAVE_RS485_CMD_UPDATE_DONE = 0x23,
SLAVE_RS485_CMD_CFG = 0x24,
SLAVE_RS485_CMD_FAN_TEMP_SETTING = 0x25,// ���÷��ȵ��¶���ֵ
SLAVE_RS485_CMD_READ_BAT_RECORD = 0x26, // ��ȡ�����ʷ����
SLAVE_RS485_CMD_BYPASS = 0x27,
SLAVE_RS485_CMD_SN_SETTING = 0x28,


SLAVE_RS485_CMD_UNLOCK = 0x31,//��������
SLAVE_RS485_CMD_CHARGE_EN = 0x32,//ʹ�ܳ������
SLAVE_RS485_CMD_CHECK_BAT = 0x33,//��ѯ���״̬
SLAVE_RS485_CMD_CHECK_LOCK_PORT_CHG = 0x34,//��ѯ������λ�ͳ��״̬����
SLAVE_RS485_CMD_DISCHARGE_EN = 0x37,//�رճ������

SLAVE_RS485_CMD_GET_BAT_RECORD_PARAM = 0x49,


//��������

SLAVE_RS485_CMD_FAN_EN = 0x70,//�򿪷���
SLAVE_RS485_CMD_FAN_DISABLE = 0x71,//�رշ���
SLAVE_RS485_CMD_R_RUNING_PARAM = 0x72, // ��ȡ�������еĲ���
SLAVE_RS485_CMD_R_REAL_TIME_INFO = 0x73, // ��ȡ�������е���Ϣ
SLAVE_RS485_CMD_R_BAT_INFO = 0x74, // ��ȡ��ص���Ҫ��Ϣ
SLAVE_RS485_CMD_R_BAT_INFO_WHEN_CRITICAL_ERR = 0x75, // ��ȡ���ʹ����ʱ�򱣴�ļĴ�����Ϣ
SLAVE_RS485_CMD_R_BAT_INFO_WHEN_EVENT_TO_STOP_CHG = 0x76, //��ȡ��⵽��������ʱ���ֳ�����
SLAVE_RS485_CMD_R_BAT_INFO_WHEN_DETECT_BAT_ERROR = 0x77, //��ȡ��⵽��������ʱ���ֳ�����
SLAVE_RS485_CMD_R_ADC_RAW_VALUE = 0x78, // ��ȡADCԭʼ����
SLAVE_RS485_CMD_HEATER_ONOFF = 0x79, // �򿪻��߹رռ���ģʽ�����ù��Ƿ�������ȡ�


//������ϱ�־
SLAVE_RS485_CMD_CLEAR_PMS_ERROR_FLAG = 0x80, //���PMS���ֵĹ��ϱ�־
//����PMS
SLAVE_RS485_CMD_PMS_RESET = 0x81, 
//��ֹ�������������Ե���Ƿ��й���
SLAVE_RS485_CMD_DETECT_BAT_ERROR = 0x82,
//����������ڵ���һ�²����㴴�������
SLAVE_RS485_CMD_TEST_TEST = 0x83,

//����ֹͣ�����ϵı�־
SLAVE_RS485_CMD_EVENT_TO_STOP_CHARGING_FLAG_BE_DISABLE_FLAG = 0x84,
//���ֹͣ�����ϵı�־
SLAVE_RS485_CMD_EVENT_TO_STOP_CHARGING_FLAG_BE_DISABLE_FLAG_CLEAR = 0x85,
//����/������������زּ��ȵ�ʹ��
SLAVE_RS485_CMD_CHARGER_WARMER_ENABLE = 0x86,
//����PMS��֤��صĹ����������
SLAVE_RS485_CMD_PMS_BAT_AUTHORITY_ENABLE = 0x87,
//��ȡ��������¶�
SLAVE_RS485_CMD_READ_CHARGER_TEMP_VALUE = 0x88

};



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



GLOBAL_CMP unsigned char usart_rx_fifo_buf[CHARGER_USART_RX_BUF_MAX];
GLOBAL_CMP unsigned char usart_tx_fifo_buf[CHARGER_USART_RX_BUF_MAX];

typedef struct{
    struct rt_semaphore usart_rx_sem;
    unsigned char usart_buf[CHARGER_USART_RX_BUF_MAX];
    rt_device_t usart_dev;

    app_fifo_t usart_rx_fifo;
    app_fifo_t usart_tx_fifo;

    unsigned char usart_tx_flag:1;/* �������ڷ��͵ı�־ */
    unsigned char bits_reserved:7;
    /*�������ò���*/
    struct serial_configure usart_config_param;
}st_charger_param;
GLOBAL_CMP st_charger_param gl_charger_paramdfg;
GLOBAL_CMP st_charger_param gl_rs485_param;
/*������ģʽ�л�������ģʽ�������ģʽ���ŵ�ģʽ*/
typedef enum{
    ENUM_POWER_MODE_IDLE,
    ENUM_POWER_MODE_CHARGE,
    ENUM_POWER_MODE_HEATER,
    ENUM_POWER_MODE_CHARGE_HEATER,
    ENUM_POWER_MODE_LOADER
}enum_power_mode;
typedef struct{
    enum_power_mode mode;
}st_power_mode_param;
GLOBAL_CMP st_power_mode_param gl_power_mode_param;


#pragma pack()


GLOBAL_CMP struct list_head master_rs485_head;
GLOBAL_CMP struct list_head *plist;
GLOBAL_CMP st_mater_state sl_master_state;

#define MASTER_DATA_BUF_MAX 16
GLOBAL_CMP st_master_cmd master_cmd_normal_node[MASTER_CMD_NODE_MAX];
GLOBAL_CMP unsigned char master_cmd_normal_node_data[MASTER_CMD_NODE_MAX][MASTER_DATA_BUF_MAX];





GLOBAL_CMP bool slave_rs485_is_detect_bat_error_enable(void);
GLOBAL_CMP bool master_rs485_data_handler_new_protocol(void);
GLOBAL_CMP bool master_rs485_rcv_cmd_process(st_hub_cmd *pl_hub_cmd);
//GLOBAL_CMP void master_rs485_init(void);
GLOBAL_CMP void master_rs485_cmd_list_init(void);
GLOBAL_CMP bool master_rs485_cmd_add(ENUM_MASTER_CMD master_cmd, unsigned char *data);
GLOBAL_CMP bool master_rs485_is_cmd_list_empty(void);
GLOBAL_CMP bool master_rs485_cmd_get_from_list(ENUM_MASTER_CMD *master_cmd, unsigned char **data);
GLOBAL_CMP bool master_rs485_one_cmd_delet_from_list(void);
GLOBAL_CMP void master_rs485_timer_cb(void);
GLOBAL_CMP unsigned short master_rs485_get_chg_current_max_value(void);
GLOBAL_CMP void master_rs485_set_chg_current_max_value(unsigned short value);
GLOBAL_CMP unsigned short master_rs485_get_chg_voltage_max_value(void);
GLOBAL_CMP void master_rs485_set_chg_voltage_max_value(unsigned short value);
GLOBAL_CMP void master_rs485_set_chg_voltage_max_value_default(void);
GLOBAL_CMP void master_rs485_cmd_poll_process(void);
GLOBAL_CMP void master_rs485_snd_cmd_poll(unsigned char* data);
GLOBAL_CMP void master_rs485_snd_data(unsigned char* data, unsigned int len);
GLOBAL_CMP void master_rs485_snd_cmd_get_dev_version(unsigned char* data);
GLOBAL_CMP void master_rs485_snd_MCU_restart_debug_info(void);
//GLOBAL_CMP void master_rs485_communication(rs485_state_t rs485_state);
GLOBAL_CMP bool master_rs485_is_485_error(void);
GLOBAL_CMP bool master_rs485_is_charger_OK(void);
GLOBAL_CMP bool master_rs485_is_charger_status_changed(void);
GLOBAL_CMP bool master_rs485_is_charge_en(void);
GLOBAL_CMP void master_rs485_charger_warmer_HW_init(void);
GLOBAL_CMP unsigned char master_rs485_is_charger_warmer_err(void);
GLOBAL_CMP unsigned char master_rs485_get_charger_warmer_fsm_state(void);
GLOBAL_CMP void master_rs485_set_charger_warmer_flag(unsigned char flag);
GLOBAL_CMP unsigned char master_rs485_get_charger_warmer_flag(void);
GLOBAL_CMP void master_rs485_set_charger_warmer_test_flag(unsigned char flag);
GLOBAL_CMP bool master_rs485_is_charger_warmer_test_enable(void);
GLOBAL_CMP void master_rs485_is_charger_warmer_test_cb(void);
GLOBAL_CMP void master_rs485_charger_warmer_condition_process(void);
GLOBAL_CMP unsigned char master_rs485_get_charger_warmer_action_enable_flag(void);
GLOBAL_CMP unsigned short master_rs485_get_V_detected_by_charger(void);
GLOBAL_CMP void master_rs485_close_charger_by_cmd(void);
GLOBAL_CMP void master_rs485_open_charger_by_cmd(void);
GLOBAL_CMP void master_rs485_check_charger_by_cmd(void);
GLOBAL_CMP unsigned short master_rs485_get_expect_current(void);
//GLOBAL_CMP void master_rs485_process(void);
GLOBAL_CMP void master_rs485_charger_error_cycle_lock_init(void);
GLOBAL_CMP void master_rs485_charger_error_cycle_lock_process(unsigned char ocp_flag, 
                                                        unsigned char scp_flag, 
                                                        unsigned char ovp_flag, 
                                                        unsigned char otp_flag);
GLOBAL_CMP void master_rs485_charger_resistance_check_init(void);
GLOBAL_CMP void master_rs485_charger_resistance_check_process(void);
GLOBAL_CMP bool master_rs485_charger_resistance_check_get_flag(void);
GLOBAL_CMP unsigned char master_rs485_charger_resistance_check_get_value(void);


GLOBAL_CMP void power_mode_set_mode(enum_power_mode mode);
GLOBAL_CMP enum_power_mode power_mode_get_mode(void);

GLOBAL_CMP void thread_rs485_entry(void* parameter);

#endif // MID_CONTROL_PROTOCOL_H__

/** @} */
