

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
#define BATTERY_ERROR_OPTION_IGNORE_NONE 0 //不忽略
#define BATTERY_ERROR_OPTION_IGNORE_TEMPERATURE 1 // 忽略温度
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
    ENUM_NFC_CMD_INDEX_R_BLV,//查询bootloader版本号
    ENUM_NFC_CMD_INDEX_BYPASS,// 透传
    ENUM_NFC_CMD_INDEX_AUTHORITY, //授权命令
    ENUM_NFC_CMD_INDEX_HISTORY_RECORD,
    ENUM_NFC_CMD_INDEX_MAX
};

//用来标志哪些数据已经从BMS中读取完成，可以供参考使用
#define BMS_DEV_INFO_FLAG_BIT (1<<0)
#define BMS_READONLY_DATA1_FLAG_BIT (1<<1)
#define BMS_READONLY_DATA2_FLAG_BIT (1<<2)
#define BMS_CTRL_FLAG_BIT (1<<3)
#define BMS_PARAM1_FLAG_BIT (1<<4)
#define BMS_PARAM2_FLAG_BIT (1<<5)
#define BMS_USER_FLAG_BIT (1<<6)





// 非零表示有命令要发送
GLOBAL_BPROC unsigned char gl_cmd_buf[NFC_READER_COUNT_MAX][ENUM_NFC_CMD_INDEX_MAX];



#define BMS_CTRL_BIT_CHG_SWITCH (1<<0)
#define BMS_CTRL_BIT_SUPPLY_SWITCH (1<<1)
#define BMS_CTRL_BIT_PRESUPPLY_SWITCH (1<<2)



#define BMS_STATE_BIT_CHG_SWITCH (1<<0)
#define BMS_STATE_BIT_SUPPLY_SWITCH (1<<1)
#define BMS_STATE_BIT_PRESUPPLY_SWITCH (1<<2)
//充电器接入位
#define BMS_STATE_BIT_CHG_IN (1<<7)
//电池充满电标志位
#define BMS_STATE_BIT_FULLY_CHARGE (1<<14)




enum{
    ENUM_REG_ADDR_HWVER = 1, //硬件版本
    ENUM_REG_ADDR_FWMSV = 3, //固件版本
    ENUM_REG_ADDR_FWREV = 4, //固件版本- 修正版本
    ENUM_REG_ADDR_FWBNH = 5, //编译版本，高
    ENUM_REG_ADDR_FWBNL = 6, //编译版本，低
    
ENUM_REG_ADDR_MCUN12 = 7,
ENUM_REG_ADDR_MCUN34 = 8,
ENUM_REG_ADDR_MCUN56 = 9,
ENUM_REG_ADDR_MCUN78 = 10,

ENUM_REG_ADDR_BVOLT = 12, //额定电压
ENUM_REG_ADDR_BCAP = 13,//额定容量
ENUM_REG_ADDR_SN12 = 14,
ENUM_REG_ADDR_SN34 = 15,
ENUM_REG_ADDR_SN56 = 16,
ENUM_REG_ADDR_SN78 = 17,
ENUM_REG_ADDR_ERECH = 18, // 最早记录号高字
ENUM_REG_ADDR_ERECL = 19, // 最早记录号低字
ENUM_REG_ADDR_LRECH = 20, // 最近记录号高字
ENUM_REG_ADDR_LRECL = 21, // 最近记录号低字


ENUM_REG_ADDR_LTSTA = 22, //历史状态

//USER ID 
ENUM_REG_ADDR_RO_USERID12 = 256,
ENUM_REG_ADDR_RO_USERID34 = 257,
ENUM_REG_ADDR_RO_USERID56 = 258,
ENUM_REG_ADDR_RO_USERID78 = 259,

ENUM_REG_ADDR_STATE = 260,
ENUM_REG_ADDR_SOC = 261,
ENUM_REG_ADDR_TVOLT = 262, //目前输出电压
ENUM_REG_ADDR_TCURR = 263, //目前输出电流
ENUM_REG_ADDR_HTEMP = 264, //最高电池温度
ENUM_REG_ADDR_LTEMP = 266,//最低电池温度
ENUM_REG_ADDR_HVOLT = 268,//最高单体电压
ENUM_REG_ADDR_LVOLT = 270,// 最低单体电压
ENUM_REG_ADDR_CSOP = 273,//10s最大允许充电电流
ENUM_REG_ADDR_SOH = 274,//         健康状态
ENUM_REG_ADDR_CYCLE = 275,//循环次数

ENUM_REG_ADDR_DEVFT1 = 280, //运行故障字1
ENUM_REG_ADDR_DEVFT2 = 281, //运行故障字1
ENUM_REG_ADDR_OPFT1 = 282, //运行故障字1
ENUM_REG_ADDR_OPFT2 = 283, //运行故障字2
ENUM_REG_ADDR_OPWARN1 = 284, //运行告警字1
ENUM_REG_ADDR_OPWARN2 = 285, //运行告警字2
ENUM_REG_ADDR_CMOST = 286, //充电MOS温度
ENUM_REG_ADDR_DMOST = 287, //放电MOS温度
ENUM_REG_ADDR_FUELT = 288 , //电量计温度,
ENUM_REG_ADDR_CONT = 289, //连接器温度
ENUM_REG_ADDR_BTEMP1 = 290, //电池温度1
ENUM_REG_ADDR_BTEMP2 = 291, //电池温度2
ENUM_REG_ADDR_BVOLT1 = 292, // 单体电池1电压
ENUM_REG_ADDR_BVOLT2 = 293, // 单体电池2电压
ENUM_REG_ADDR_BVOLT3 = 294, // 单体电池3电压
ENUM_REG_ADDR_BVOLT4 = 295, // 单体电池4电压
ENUM_REG_ADDR_BVOLT5 = 296, // 单体电池5电压
ENUM_REG_ADDR_BVOLT6 = 297, // 单体电池6电压
ENUM_REG_ADDR_BVOLT7 = 298, // 单体电池7电压
ENUM_REG_ADDR_BVOLT8 = 299, // 单体电池8电压
ENUM_REG_ADDR_BVOLT9 = 300, // 单体电池9电压
ENUM_REG_ADDR_BVOLT10 = 301, // 单体电池10电压
ENUM_REG_ADDR_BVOLT11 = 302, // 单体电池11电压
ENUM_REG_ADDR_BVOLT12 = 303, // 单体电池12电压
ENUM_REG_ADDR_BVOLT13 = 304, // 单体电池13电压
ENUM_REG_ADDR_BVOLT14 = 305, // 单体电池14电压
ENUM_REG_ADDR_BVOLT15 = 306, // 单体电池15电压
ENUM_REG_ADDR_BVOLT16 = 307, // 单体电池16电压


ENUM_REG_ADDR_MAXCHGV = 315,//最大充电电压
ENUM_REG_ADDR_TVST = 317, // TVS温度
ENUM_REG_ADDR_BHR1 = 318, //电池1温升速度
ENUM_REG_ADDR_BHR2 = 319, //电池2温升速度
ENUM_REG_ADDR_FGHR = 320, // 电量计温升速度
ENUM_REG_ADDR_DROCV = 321, // 电芯电压最大下降速度
ENUM_REG_ADDR_IDROCV = 322, // 电芯电压最大下降速度编号


ENUM_REG_ADDR_CTRL = 512, 
ENUM_REG_ADDR_RESET = 534,


//USER ID 
ENUM_REG_ADDR_USERID12 = 4097,
ENUM_REG_ADDR_USERID34 = 4098,
ENUM_REG_ADDR_USERID56 = 4099,
ENUM_REG_ADDR_USERID78 = 4100,




};


//每次轮询的时间长度
#define BAT_POLL_ON_TIME_MAX 10000
// 轮询电池间隔时间设置
#define BAT_POLL_INTERVAL_MAX 60000
#define BAT_POLL_BIG_INTERVAL_MAX 86400000 // 24小时
#define BAT_POLL_BIG_BIG_INTERVAL_MAX 0xFFFFFFFE//259200000 // 72小时
// 仓门有变化时候，一直处于直轮询模式1分钟，过后再根据实际情况设置模式
#define BAT_POLL_DELAY_MAX 60000//1800000//360000         //改为60秒
//接收到升级BMS的start 命令，允许正常查询电池的时间，10分钟
#define BAT_POLL_BMS_UPDATE_DELAY_MAX 600000
//当有透传命令给电池时候，停止间隔轮询180秒
#define BAT_POLL_DELAY_MAX_BY_BYPASS_CMD 180000
typedef struct{
unsigned char is_status_changed_on_door;
unsigned char is_poll_allways_en_flag; // 0, 表示有时间间隔的轮询；1，表示一直轮询，无间隔时间
unsigned int poll_inerval_timer_cnt;
unsigned int poll_interval_MAX;
unsigned int poll_allways_en_delay_cnt; // 在仓门有变化的时候再reset计数，保证刚插进去的电池能轮询三分钟
unsigned int poll_interval_for_bms_update_cnt;
}st_bat_poll_param;
GLOBAL_BPROC st_bat_poll_param sl_bat_poll_param;

/*
检测电池故障
*/
typedef struct{
//该电池是否有故障。0，没有故障
unsigned char is_bat_error;
//是否需要检测电池故障，当检测到有电池，电池SOC，仓门关闭30秒后检测。
unsigned char is_need_bat_checking; // 0， 不需要检测；1，正在检测；2，检测完毕
//检测进度
unsigned char checking_step;
//每个进度的超时时间，超时后退出。
unsigned int checking_step_timeout_cnt;
//仓门关闭后的计数器
unsigned int door_close_delay_cnt;
//通用计数器
unsigned int comm_counter_cnt;
//每秒钟查询一次的计数器
unsigned int bat_process_cnt;

///////////////////////////////////
//如果没有检测到有预放，每5分钟再检测一次，最多检测10次
unsigned int bat_connected_faild_error_cnt; //计数用，5分钟计数
unsigned char bat_connected_faild_error_cnt_reload_max; // 最多检测次数，最多10次
///////////////////////////////////
//如果是Step的5分钟超时故障，过30分钟后自动重启PMS。
unsigned int bat_step_timeout_err_cnt;
///////////////////////////////////

}st_check_bat_err;

GLOBAL_BPROC st_check_bat_err gl_check_bat_err;

//电池插入时候，使用充电器来检测电池是否在位
typedef struct{
unsigned char is_bat_v_valid; //充电器是否检测到电池输出电压
//是否需要检测电池电压，当检测到有电池，电池SOC，仓门关闭就开始检测
unsigned char is_need_bat_v_checking; // 0， 不需要检测；1，正在检测；2，检测完毕
unsigned short bat_v_charger_cmd_cnt; //开始打开电池预放后，与充电器通讯的次数
unsigned int bat_v_charger_comm_timeout_cnt; //开始打开电池预放后，与充电器通讯的时间计数
unsigned short bat_v_value; //充电器检测到的电池输出电压值，单位0.1V
unsigned short bat_v_charger_check_totol_time_cnt; //整个过程的最大时间。
unsigned char bat_v_charger_charge_valid_cnt; // 充电器有电流输出并且电池有电流输入的次数，充电器应答时候计数
}st_check_bat_v;
GLOBAL_BPROC st_check_bat_v gl_check_bat_v;


typedef enum{
    ENUM_SCR_OF_STOP_FLAG_CURRENT_DIFF, //充电器电流-电池电流>预期值（几百毫安）
    ENUM_SCR_OF_STOP_FLAG_HIGH_TEMP, //充电时，任何电池传感器温度大于预期值（80度）
    ENUM_SCR_OF_STOP_FLAG_PRE_CURRENT, //电池预启动失败(需要霍尔传感器有效的情况下判断)
    ENUM_SCR_OF_STOP_FLAG_CHG_SOC_UNWORK, //充电持续一段时间后（20分钟），电池电压和SOC没有增加预期量
    ENUM_SCR_OF_STOP_FLAG_LOW_VOLTAGE, //充电器启动充电时，当检测到电池电压低于26V（13S）和32V（16S)，不能启动充电
    /*
        PMS监控电池内部电压，当检测到电池电压达到充满电压后，充电电流仍未降低，
            与电池实际需求电流不一致，同时电池电压仍然增加，此时需断开充电器的供电
    */
    ENUM_SCR_OF_STOP_FLAG_MAX_VOLTAGE_EXCEEDED, 
    ENUM_SCR_OF_STOP_FLAG_CURRENT_OUT, //电池有超过几百毫安的电流输出(在充电柜是不允许的)
    ENUM_SCR_OF_STOP_FLAG_SOC_ERROR, //SOC为0，但是电压高。
    ENUM_SCR_OF_STOP_FLAG_UVP_ERROR,//或者有UVP故障但是电压与SOC高
    ENUM_SCR_OF_STOP_FLAG_UVP_OVP_ERROR, //同时出现低压和高压告警
    ENUM_SCR_OF_STOP_FLAG_UTP_OTP_ERROR, //同时出现低温和高温告警
    ENUM_SCR_OF_STOP_FLAG_HALL_OFF, //电池霍尔开关没有检测到磁铁
    ENUM_SCR_OF_STOP_FLAG_CELL_VOLTAGE_LOW, // 单芯电压低于2V
    
    ENUM_SCR_OF_STOP_FLAG_CHARGER_OCP_LOCK, //关仓门后，充电器反复上报过流故障，锁定故障，停止充电
    ENUM_SCR_OF_STOP_FLAG_CHARGER_SCP_LOCK, //关仓门后，充电器反复上报短路故障，锁定故障，停止充电

    ENUM_SCR_OF_STOP_FLAG_CELL_V_DIFF_VALUE_EXCEEDED, // 电芯电压差异大，最高电芯电压与最低电芯电压差值大于等于0.1V
    
    ENUM_SCR_OF_STOP_FLAG_CHARGER_OVP_LOCK, //关仓门后，充电器反复上报过压故障，锁定故障，停止充电
    ENUM_SCR_OF_STOP_FLAG_CHARGER_OTP_LOCK, //关仓门后，充电器反复上报过温故障，但是只要充电器允许，可以继续充电

    ENUM_SCR_OF_STOP_FLAG_MAX = 32 //不能超过该最大值32
}enum_src_of_stop_flag;

typedef struct{
//标记停止充电的原因源
unsigned int src_of_stop_flag;
unsigned char bat_ID[6];
unsigned char stop_flag;
unsigned int current_diff_counter;
unsigned int bat_current_out_counter; 
unsigned int hall_miss_status_clear_delay_cnt;
//选择哪些原因源可以被设置到src_of_stop_flag中
unsigned int src_of_stop_flag_disable_flag; // 对应src_of_stop_flag中的每个Bit，0是允许，1是禁止
}st_event_to_stop_chg_param;
GLOBAL_BPROC st_event_to_stop_chg_param gl_event_to_stop_chg_param;

typedef struct{
    unsigned char is_bat_chg_enable; // 0, 不允许充电，1，允许前期小电流充电，2，允许正常充电
    unsigned char is_need_to_reflash_temp; //是否需要刷新电池插入时候的电池温度值
    short temp_value;//插入电池时候电池最低温度，单位是0.1℃
    unsigned int delay_counter_1;//第一阶段计数器
    unsigned int delay_counter_2;//第二阶段计数器
    unsigned int delay_counter_charge; //前期充电阶段计数
    unsigned int low_current_delay_count; //低电流充电的时间长度
}st_bat_low_temp_param;
GLOBAL_BPROC st_bat_low_temp_param gl_bat_low_temp_param;

typedef struct{
    short low_temp; // 0.1℃为单位
    unsigned int low_temp_delay_cnt; // 1mS为单位
    short high_temp;// 0.1℃为单位
    unsigned int high_temp_delay_cnt;// 1mS为单位
    unsigned int low_current_delay_count;// 1mS为单位
}st_bat_low_temp_ctrl_param;

//当充电器输出电压精度有问题导致输出电压偏低时候，适当调高充电器输出电压。
typedef struct{
unsigned char reseve[3];
unsigned char voltage_offset_value; // 单位0.1V
unsigned int debount_time_cnt;
//当前期望充电电压(未加偏移量)，单位0.01V
unsigned short expect_setting_voltage;
//当前期望充电电流(未加偏移量)，单位0.01A
unsigned short expect_setting_current;
}st_charger_setting_V_offset_param;
GLOBAL_BPROC st_charger_setting_V_offset_param gl_charger_setting_V_offset_param;

//防盗流程的参数
typedef struct{
unsigned char connector_default_state:1; //连接器状态; 1, 未连接; 0, 连接好
unsigned char switch_default_state:1; //微动开关状态; 1, 未到位; 0, 到位
unsigned char reserve_bit:6;
}st_bat_theft_prevention_param;
#ifndef BATTERY_PROCESS_C
GLOBAL_BPROC st_bat_theft_prevention_param gl_bat_theft_prevention_param;
#else
GLOBAL_BPROC st_bat_theft_prevention_param gl_bat_theft_prevention_param = {0, 0, 0};
#endif


/*
读取电池历史记录
*/
//历史记录编号的范围
#define HISTORY_RECORD_INDEX_START 1
#define HISTORY_RECORD_INDEX_END 536862720

typedef struct{
//发送读取历史数据的NFC通讯命令的间隔时间
unsigned int NFC_CMD_interval_cnt;
//需要读取的当前记录编号，真正需要读取的是从下一个编号开始
unsigned int history_record_begin_index;
//需要读取的记录数目
unsigned int history_record_cnt;
//当前已经读取的记录数目
unsigned int history_record_read_cnt;
//当前电池最早记录编号
unsigned int history_record_earliest_index;
//当前电池最近记录编号
unsigned int history_record_lastest_index;
//需要读取的当前记录编号的时间 。 BCD码
unsigned char history_record_begin_index_datetime[6];
//当前电池最早记录编号的时间
unsigned char history_record_earliest_index_datetime[6];
//当前电池最近记录编号的时间
unsigned char history_record_lastest_index_datetime[6];
//缓存历史数据
unsigned char history_record_buf[128];
//电池ID
unsigned char bat_ID[6];
//起始记录编号和数目是否有效
unsigned char is_index_valid:1;
//buffer中的数据是否有效
unsigned char is_buf_valid:1;
//buffer中的数据是否已经上传
unsigned char is_buf_updated:1;
//当前电池最早记录编号和时间的值是否有效
unsigned char is_earliest_param_valid:1;
//当前电池最近记录编号和时间的值是否有效
unsigned char is_lastest_param_valid:1;
unsigned char bit_reserve:4;
}st_bat_history_record_param;

GLOBAL_BPROC st_bat_history_record_param gl_bat_history_record_param;



//BAT_ERROR_CHECK_DELAY_AFTER_BAT_UPDATE
GLOBAL_BPROC unsigned int sl_bat_error_check_delay_after_bat_update_cnt;


typedef struct{
    unsigned char OD_update_file_max_subindex;
    /*升级文件的硬件版本号*/
    unsigned char OD_update_file_HW_version[2];
    /*升级文件的固件版本号*/
    unsigned char OD_update_file_FW_version[7];
    /*升级文件的buffer地址*/
    unsigned int OD_update_file_buff_addr;
}st_update_file_param;

/*开锁命令*/
#define OD_PMS_CMD_MASK_ULOCK (1<<0)
/*充电使能*/
#define OD_PMS_CMD_MASK_CHARGE_EN (1<<1)
/*根据温度动态充电使能*/
#define OD_PMS_CMD_MASK_D_CHARGE_EN (1<<2)
/*5V电源输出使能*/
#define OD_PMS_CMD_5V_EN (1<<3)
/*12V电源输出使能*/
#define OD_PMS_CMD_12V_EN (1<<4)
/*认证功能是否启动*/
#define OD_PMS_BAT_AUTHORITY_EN (1<<5)

typedef struct{
unsigned char is_bat_detected;
/**************************************************************************************************/
/*高低温测试时候需要的温度数据*/
short heater_NTC_temperature;
short cabinet_NTC_temperature;
short charger_temperature;

/**************************************字典参数************************************************/
/*字典; Index: 0x3010*/
unsigned char OD_3010_max_subindex;
/*
*bit0: 电池是否在线
*bit1: 充电器是否在线
*bit2: 电磁锁状态
*bit3~4: 前仓门电机锁位置状态
*bit5: 前仓门锁运行状态
*bit6: 风扇状态

*/
unsigned char OD_pms_status1;
/*
*bit0: 当前设置充电器使能的状态
*bit1: PMS板充电开关状态
*bit2: PMS板加热开关状态
*bit3: PMS板其他功能开关状态
*bit4: PMS板认证电池的结果
*bit5: 电池认证PMS板的结果
*/
unsigned char OD_pms_status2;

/*字典; Index: 0x3011*/
unsigned char OD_3011_max_subindex;
/*该变量的每个Bit代表不同意思，如上的宏定义*/
unsigned char OD_pms_cmd; /*实际状态，只读*/
unsigned char OD_pms_cmd_set; /*设置OD_pms_cmd对应bit为1。只写*/
unsigned char OD_pms_cmd_clear; /*写OD_pms_cmd_clear bit为1，表示设置OD_pms_cmd对应bit为0。只写*/
/*最大充电电流*/
unsigned short OD_charging_current_max;
/*可换电门限值*/
unsigned char OD_swapping_soc;
/*最大充电电压*/
unsigned short OD_charging_voltage_max;

/*字典; Index: 0x3012*/


/*字典; Index: 0x3013*/
unsigned char OD_3013_max_subindex;
/*PMS 硬件版本号*/
unsigned char OD_PMS_HW_version[2];
/*PMS固件版本号*/
unsigned char OD_PMS_FW_version[7];
/*PMS SN码*/
unsigned char OD_PMS_SN[14];


/*字典; Index: 0x3014*/
unsigned char OD_3014_max_subindex;
/*加热电阻位置的NTC温度值*/
short OD_heater_ntc_temperature;
/*仓顶位置的NTC温度值*/
short OD_cabinet_ntc_temperature;
/*烟感1的电压值，单位1mV*/
unsigned short OD_smoke_sensor1_value;
/*烟感2的电压值，单位1mV*/
unsigned short OD_smoke_sensor2_value;
/*烟感装置的工作电流，单位1mA*/
unsigned short OD_smoke_senor_total_current;
/*前仓门电机当前工作电流，单位1mA*/
unsigned short OD_front_motor_working_current;

/*字典; Index: 0x3015*/
unsigned char OD_3015_max_subindex;
/*充电器返回的充电电流，单位0.1A*/
unsigned short OD_charger_C;
/*充电器返回的充电电压，单位0.1V*/
unsigned short OD_charger_V;
/*充电器返回的充电功率，单位0.1W*/
unsigned short OD_charger_P;
/*充电器返回的故障1值*/
unsigned char OD_charger_error1;
/*充电器返回的故障2值*/
unsigned char OD_charger_error2;
/*充电器返回的温度值。需要偏移40℃*/
unsigned char OD_charger_temperature;
/*充电器返回的保留字节*/
unsigned char OD_charger_reserved;

/*字典; Index: 0x3016*/
unsigned char OD_3016_max_subindex;
/*充电器返回的硬件版本号*/
unsigned char OD_charger_HW_version[2];
/*充电器返回的固件版本号*/
unsigned char OD_charger_FW_version[7];
/*充电器返回的SN号*/
unsigned char OD_charger_SN[14];

/*字典; Index: 0x3017*/
unsigned char OD_3017_max_subindex;
/*当前设置充电器的充电电压*/
unsigned short OD_charger_voltage_setting_value;
/*当前设置充电器的充电电流*/
unsigned short OD_charger_current_setting_value;

/*字典; Index: 0x3018*/
unsigned char OD_3018_max_subindex;
/*PMS记录的故障*/
/*主动检测电池的故障*/
unsigned char OD_pms_active_detected_err_of_bat;
/*被动检测电池的故障*/
unsigned char OD_pms_detected_err_of_bat[4];
/*PMS检测到的其他故障*/
unsigned char OD_pms_other_detected_err;
/*PMS原FAULT故障*/
unsigned char OD_pms_fault;
/*PMS原DAMAGE故障*/
unsigned char OD_pms_damage;

/*字典; Index: 0x3019*/
/*PMS升级文件是否有效。每个bit代表一个升级文件，对应0x301A~0x301F*/
unsigned short OD_pms_update_file_flag;

/*字典; Index: 0x301A*/
st_update_file_param OD_pms_update_file_param[1];

/*字典; Index: 0x3020*/
/*充电器升级文件是否有效。每个bit代表一个升级文件，对应0x3021~0x302F*/
unsigned short OD_charger_update_file_flag;

/*字典; Index: 0x3021*/
/*字典; Index: 0x3022*/
/*字典; Index: 0x3023*/
st_update_file_param OD_charger_update_file_param[3];


/*字典; Index: 0x3030*/
/*充电器升级文件是否有效。每个bit代表一个升级文件，对应0x3031~0x30AF*/
unsigned char OD_bat_update_file_flag[16];

/*字典; Index: 0x3031*/
/*字典; Index: 0x3032*/
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
//返回值的精度是0.1V
GLOBAL_BPROC unsigned short Battery_get_max_chg_voltage(unsigned char bms_index);
//返回值的精度是0.1A
GLOBAL_BPROC unsigned short Battery_get_max_chg_current(unsigned char bms_index);
//返回值没有偏移，但是小于0摄氏度的话统一返回0
GLOBAL_BPROC unsigned short Battery_get_bat_temp(unsigned char bms_index);
GLOBAL_BPROC short Battery_get_bat_lowest_temp(unsigned char bms_index);
//返回值是0.1mA/h为单位
GLOBAL_BPROC unsigned short Battery_get_bat_rated_cap(unsigned char bms_index);
//返回值是0.1V为单位
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
//记录插入电池的最低温度，关舱门时候调用
GLOBAL_BPROC void Battery_set_bat_temp_value_when_plus_in(void);
GLOBAL_BPROC short Battery_get_bat_temp_value_when_plus_in(void);
//获取当前电池的最低温度
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

