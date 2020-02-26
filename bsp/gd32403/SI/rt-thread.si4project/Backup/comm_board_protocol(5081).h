

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
 unsigned char data_valid_flag; //buf中的数据是否有效；0无效；1有效
 unsigned char data_head[3];
 unsigned char data_buf[128];
 unsigned char data_crc16[2];
 unsigned int data_offset; //数据偏移
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
0x00	成功。
0x01	操作失败。
0x02	操作超时。
0x03	设备未准备好。
0x04	不支持的命令。
0x05	硬件错误。
0x06	参数错误。
0xFF    	未知错误
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


//与上位机通讯时候，填写发送数据的地方设置个计数时间，超过这个时间则不再发送这些数据
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
//如果ID信号已经打开了，则关闭ID信号的动作需延迟60秒
unsigned int ID_close_delay_cnt;
//心跳中，上电延迟10秒再上报充电器的485连接状态(Fault128中的一种)
unsigned int charger_com_err_report_delay_cnt; 
//定时发送查询命令给充电器，计数器
unsigned int poll_charger_cmd_cnt;
//命令发送的超时计数
unsigned int cmd_timeout_cnt;
//接收返回命令后，发送下一条命令的时间间隔
unsigned int next_cmd_interval_cnt;
//收到充电器升级开始和交换的命令后，60秒内不再主动给充电器发送命令数据。
//等到再收到充电器升级结束的命令后，改为3秒内不再主动给充电器发送命令数据。
unsigned int normal_cmd_delay_when_chger_updating_cnt;
//是否正在处理发送命令的流程，直到删除链表中的命令才清零
unsigned char cmd_processing_flag;
//命令发送超时标志
unsigned char cmd_snd_faild_flag;
//一条命令重复发送次数计数
unsigned char cmd_snd_faild_cnt;
//已经发送失败的次数计数
unsigned char all_cmd_snd_faild_cnt;
//poll 命令485通讯故障计数
unsigned char chg_485_cmd_poll_faild_cnt;
//接收到开仓门指令后，立刻停止充电器充电，直到上位机再发送允许充电的指令
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
    //要发送给充电器的命令
    ENUM_MASTER_CMD cmd;
    //命令带的数据指针
    unsigned char *data;
}st_master_cmd;

#define MASTER_CMD_NODE_MAX 3

#pragma pack(1)


typedef struct{
//充电使能。0x00:禁止充电；0x01:使能充电；0x02:使能加热功能
unsigned char chg_en;
//当前期望充电电压，单位0.1V
unsigned char expect_voltage[2];
//当前期望充电电流，单位0.1A
unsigned char expect_current[2];
//当前电池温度
unsigned char bat_temp;
//设计容量，单位0.1mAh。
unsigned char rated_cap[2];
//设计电压，单位0.1V。
unsigned char rated_voltage[2];
unsigned char reserve[6];
}st_poll_cmd_param;

//充电与否由这个全局变量控制。
//定义一个存储参数的全局变量
GLOBAL_CMP st_poll_cmd_param gl_poll_cmd_param;


typedef struct{
//充电电流，单位0.1A。
unsigned char chg_current[2];
//充电电压，单位0.1V
unsigned char chg_voltage[2];
//充电功率，单位0.1W
unsigned char chg_power[2];
//Bit0：Charger过流保护
unsigned char charger_OAP_flag:1;
//Bit1：Charger过压保护
unsigned char charger_OVP_flag:1;
//Bit2：Charger过温保护
unsigned char charger_OTP_flag:1;
//Bit3：Charger欠流，在给升压阶段，Charger电流始终无法大于300MA
unsigned char charger_UAP_flag:1;
//Bit4：Charger 欠压，在升压阶段（3S），Charger电压始终无法升压到20V
unsigned char charger_UVP_flag:1;
//Bit5：预留
unsigned char bit5_reserv:1;
//Bit6：充电电压匹配。
unsigned char charger_voltage_match_flag:1;
//Bit7：充电电流匹配。
unsigned char charger_current_match_flag:1;
//V1.9版本以后的协议
unsigned char charger_ID_flag:1;
unsigned char charger_fan_error_flag:1;
unsigned char charger_low_AC_supply_flag:1;
unsigned char charger_byte8_reserve:5;
//充电器温度值；单位摄氏度，范围(-40~214)；该值需要减40才是实际值；0xFF表示无效
unsigned char charger_temp;
unsigned char charger_byte10_reserve;
}st_poll_cmd_cnf_param;
GLOBAL_CMP st_poll_cmd_cnf_param gl_poll_cmd_cnf_param;


//记录充电器故障状态，在心跳中上传
typedef struct{
unsigned char is_need_update; // 是否需要通过心跳上传到服务器
unsigned char charger_err[2]; // 对应充电器返回的两个故障字节
}st_charger_err_status;
GLOBAL_CMP st_charger_err_status gl_charger_err_status;


typedef struct{
//硬件主版本号
unsigned char HW_main_ver;
//硬件子版本号
unsigned char HW_sub_ver;
//固件主版本号
unsigned char FW_main_ver;
//固件子版本号
unsigned char FW_sub_ver;
//固件修订版本
unsigned char FW_revised_ver;
//固件build号32
unsigned char FW_build_ver[4];
}st_version_desc;

typedef struct{
    unsigned char Protocol_version;
    unsigned char dev_type;
    unsigned char Port_max;
    st_version_desc PMS_version;
}st_dev_info;


typedef struct{
    unsigned char is_valid; // 1,表示下面的版本信息是有效的
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
/*版本说明
V12: 
增加风扇测试命令;
LED变绿色的判断条件修改，由原来的< 改为<=，以保持与安卓机APP同步
充电器485通讯协议中的返回格式有修改。
充电器ID打开后，如果电池掉线需要关闭，则延迟60秒再关闭
V13:
开仓门情况下，如果下发开仓门的命令，绿灯也要闪烁
V14:
充电器485通讯故障上报
增加读取PMS参数信息的调试命令

V15:
电池温度故障时候，仓门指示灯的故障状态不再包含电池温度故障。
V16:
充电器485通讯故障两次才认为有错误
V17:
开仓门锁时间加大到6.5秒

V18: 
NFC适时通讯，以保护电池不让其耗电。
如果电池电压低于某值，则轮询间隔时间加大
电池要求充电电流为0，或者其他原因要求充电电流为0，则关闭充电器
增加SOH值

V19:
舱内风扇在电池达到温度就开始转，不再判断是否正在充电的条件

V20:
1. 增加命令，读取IO状态、LED参数、ELOCK参数、步进电机参数、风扇参数、
  电池间隔轮询参数
2. 修改电池间隔轮询的时间BAT_POLL_ON_TIME_MAX为8秒
3. 上传的fault加不能充电的提示
4. 修复异常提示fault64的bug(检测仓门异常的Bug)

V21:
1. 修复函数slave_rs485_communication_init中的初始化BUG。

V22:
1. 判断间隔轮询的代码中有BUG，忘记删除测试代码
2. 上位机命令禁止充电时候，也进入间隔轮询电池
3. 间隔轮询电池时候，如果没有在通讯状态，NFC将休眠，天线不发出功率。
    原来只是不发数据，但是天线有功率输出。
4. 屏蔽步进电机的微动开关IO口读取判断功能。
5. 增加步进电机转动超时功能。

V23: 
1. 仓门变化时候，前面30分钟要一直轮询(BAT_POLL_DELAY_MAX)
2. 上位机发送命令开仓门，打开时候，确保后电机锁能处于开的状态。
    (但是在锁弹开之后检测到关门的话，会锁)
3. 上电或者重启，后电机锁默认关闭状态。

V26: 
1. 增加检测电池故障功能。(Masked)
2. 支持充电器远程升级命令。(Done)
3. 改为，充电：当电池放入充电柜中进行充电时，
    先打开电池的充放电管，再延时打开充电器；(Done)
4. 改为，取电：当换电时，先关闭充电器的输出，
    再延时关闭电池的充放电管。
5. 修改BUG: 后电机运转时候，会影响仓门锁开仓门逻辑。
    (解决:电机转动时候本来就是不允许开仓门)
6. 电池寄存器值是0xFFFF这个无效值时候，适当处理。(Done)
7. 解决: 处理接收到的数据包不及时，导致应答包影响到其它仓的数据接收。(Done)
8. 电池升级的XMODEM 模式，超时时间改为26秒BMS_XMODEM_TOTAL_TIMEOUT_MAX。 (Done)
9. 看门狗改为4秒超时。(Done)
10. PMS地址的读取放在程序开始的时候。以后如有再使用PMS地址，
    直接取变量中存储的地址，不再从IO口中读取。
11. 在升级电池程序后获取电池版本号时候，有可能获取到的信息错误。现在修改完善。    

V27:
1. 温度的无效值改为0xFF。
2. 电池SOC在检测不到电池时候上传的值为无效值0xFF。

V28:
1. 收到不能充电的故障事件后，停止充电标志置位，直到仓门打开或者检测到电池ID有变化。
        (其他任何条件都不能清零该标志)

2.       充电器电流-电池电流>预期值（几百毫安）
3.       充电时，任何电池传感器温度大于预期值（70度）
4.       电池预启动失败(需要霍尔传感器有效的情况下判断)
6.    充电器启动充电时，当检测到电池电压低于26V（13S）和32V（16S)，不能启动充电
7.    PMS监控电池内部电压，当检测到电池电压达到充满电压后，充电电流仍未降低，
            与电池实际需求电流不一致，同时电池电压仍然增加，此时需断开充电器的供电
            
8.    读取到电池过充故障-------------- (原来的逻辑里有判断)


12.   仓门灯采用不同的闪烁模式代表故障和升级。(在测试时候发现BUG，
        手动升级电池，升级失败返回。LED闪两次绿灯。这时，去点打开仓门，
        有可能灯不闪了，门也不开。
       原因:  发现是定时器回调函数没有被调用。)

13. 保证充电器先关闭才关电池。如果不能检测到充电器关闭，4秒超时可以关电池。

15. 修改，在判断故障电池时候，过压故障不能屏蔽。

V29:
1. 修改，充电器电流-电池电流>预期值（改为800毫安）

V30:
//5. 充电持续一段时间后（20分钟），电池电压和SOC没有增加预期量

---. 增加检测电池故障功能:(还没有详细测试) (发现退不出循环)
9. 充电MOS失效（增加检测充电MOS失效）
11. 检测放电MOS失效（增加检测放电MOS失效）

//10. PMS发现充电器不受控制，上报故障给控制板。

14. 电池有输出几百mA以上，持续时间超过10秒。关闭充电。

15. 如果电池寄存器中的满电标志被置位，应该停止充电并关闭充电器和电池充放电开关。
16. 电池或者上位机的充电需求电流为0时候，在原来485发送关充电的基础上，再关充电器ID。
17. 透明传输给电池时候，暂时取消间隔轮询，时长180秒。
18. 充电器报故障时候需要闪红灯。闪3下红灯
19.电池故障归类，直接拿回去维修分析的红灯常闪，
        还需要详细信息判断是否拿回的闪2下红灯。
20.增加命令读取电池的重要信息。用于电池的故障分析。
21. NFC丢失判断的次数被修改。在仓门关闭状态下，次数是30；在仓门打开状态下，次数是8。
//22. fault bit3 表示请求切断继电器。
23. 自定义命令0x72 增加充电器返回的信息
24. 仓内微动开关的信号放在电池信息命令中的damage字节bit4。
    没有检测到时候设置为1，检测到设置为0。
25. 在仓门开的状态下，只有微动开关检测到电池才上传电池信息。
26. 在仓门关的状态下，如果微动开关一直没有检测到电池，则不上传电池信息；
     如果微动开关检测到电池，则以后按照没有微动开关功能前的逻辑上传电池信息。
27. 修改命令0x24，增加设置电压值。
    并且根据该电压限制充电器电压；该电压值与电池需求电压值比较，取小的。
28. 修改所有命令的数据长度的兼容性。
29. 远程清除故障告警。

问题:
1. 3.2V以上，SOC是0，该项目检测有误判。修改:暂时改为3.8V来判断
2. 微动开关去抖
3. 电池霍尔开关错误可以10分钟后自动消除
4. 主动检测电池充放电管时候，超时，0x40故障。(改为5分钟超时)
5. 报UVP故障，但是电池SOC或者电压高
6. 电池同时报UVP和OVP故障
7. 又高温又低温故障
8. 主动监测到故障时候保存现场数据。
9. 单芯电压低于2V报故障

V31:
1. 主动检测电池充放电管故障的代码修改。(30版本测试发现BUG)
    a. 需要关门后才能启动主动检测。
    b. 需要在主动检测前先关闭充电器和电池充放电管。
//2. 增加功能，关舱门时候检测电池是否跟连接器连接好。
3. 增加命令，可以重启PMS。

V32:
1. 增加功能，关舱门时候检测电池是否跟连接器连接好。
2. 去除30版本的功能:    
    //25. 在仓门开的状态下，只有微动开关检测到电池才上传电池信息。
    //26. 在仓门关的状态下，如果微动开关一直没有检测到电池，则不上传电池信息；
    //     如果微动开关检测到电池，则以后按照没有微动开关功能前的逻辑上传电池信息。
    同时修改后电机锁为，关仓门就转动。30版本是  关仓门加有微动开关才转动。   
3. 充电器检测电池电压的方式来判断电池的连接器是否接触好。
    使用电池信息命令中的damage bit5来上传状态给上位机，1 表示没有接触好，0 表示接触好。
4. 30分钟判断电池不充电，判断方法是，
    当电池SOC小于90%时候，看是否有充电电流(大于50mA)。
    该故障归类到Damage bit6的标志。但是不会关闭电池和充电器，一旦恢复充电，故障消除。
5. 增加判断电池故障的去抖时间3秒。(在检测故障事件中增加)
6. 修改fault bit6 (64)的原因，原来产生的原因有: 1. 仓门被异常打开；2. 开仓门指令打不开仓门。
    现在改为只有一个原因产生: 仓门被异常打开。
7. 增damage bit6 (64)为其他错误。目前包括的错误有: 1. 开仓门指令打不开仓门
8. 修改编译出现的WARNING。
9. 增加0x82命令，可以允许和禁止主动启动测试电池故障的功能。

V33:
1. 电池升级完成后容易被诊断到故障。
    是因为电池刚升级完成后还处在不稳定状态。
    现在处理方式是，电池升级完成后，延迟120再检测电池故障。
2. 充电器485命令，接收到返回后，延迟20毫秒再发送下一条命令。

V34:
1. 充电器通讯协议中Poll命令返回数据Byte7 bit5被修改定义了，现在把这Bit5中的故障也停止充电。
2.  H,M,L级故障，过滤12s再上报给上位机
3. 增加判断电池故障的去抖时间改为8秒。(在检测故障事件中增加)
3. 充电器过流或者短路故障，需要计数(20次)然后记住上报故障。damage4 和0x72命令中详解。

V35:
1. 电芯电压差大于0.1V，报故障。
    a. 最高电芯电压大于3.8V
    b. 最高和最低电压差大于0.1V
    c. 持续时间大于10秒
2. 仓门开的时候，不设置被动检测的故障标志。    

V36:
1. 暂时关闭V35版本的检测电芯电压差大于0.1V的故障。
2. PMS程序重启后，不再默认检测一次"电池是否跟连接器连接好"。(V32 版本中的1 功能)

V37:
1. 修改主动检测电池充放电管的启动条件，原来电池初级过放、二级过放、预启动失败超时、预启动失败外部重载/短路，
这4个条件可以允许启动，现在增加改为不允许启动。
2. 修改，主动监测时候，应该在开始流程时候，先关闭充电器并等待6秒再关电池。

V38:
1. 屏蔽，"如果SOC为0，而电池电压在(3.8*电芯数量)V以上，则判断电池有问题" 该故障提示屏蔽
2. 屏蔽，"报UVP故障，但是电池SOC在30%以上，或者电池电压在(3.8*电芯数量)V以上"该故障提示屏蔽

V39:
1. 优化，主动检测时候，避免有冲击电流。

V40:
1. 增加支持电磁锁的开仓控制。同时修改了一下判断仓门状态的逻辑
2. 根据电池温度决定是否充电和充电电流
3. 查看安卓机下发的最大充电电流电压
4. 增加开机标记。"0x33"命令中，Lock status, Bit7重新被定义为开机标志。在设备参数配置命令里，清除开机标志。
5. 仓门刚关闭时候，如果认为仓内有电池，延迟上报关仓门的状态；
如果认为仓内没有电池，立刻上报关仓门的状态。策略:在判断仓门关闭去抖时候，
如果有电池就在延迟时候多延长1秒，同时检测仓门的函数改为在中断延迟后每100mS轮询一次。

V41:
1. 当主动检测到电池连接不好的故障后，重复再检测多次。

V42:
1. 支持充电器通讯协议V1.9定义的新故障报警
2. 上一个版本功能    "  2. 根据电池温度决定是否充电和充电电流"， 做到所有柜子都兼容。
需要上位机配合，下发是否启动该功能。功能启动标志存放在FLASH中的0xFE00000开始地址
3. 设备参数配置 命令中增加清除"PMS已经重启过的标志"
4. 在检测充电MOS故障时候，可能存在时间不够。目前修改措施是，增加20秒时间。
5. 当充电器输出电压精度有问题导致输出电压偏低时候，适当调高充电器输出电压。

V43:
1. 新的协议中对心跳数据进行了修改。
2. 支持48V电池的充电电流到9A
3. 增加充电器加热模式新指令
4. 当接收到允许加热命令后，判断温度是否是在需要加热范围内，如果是，
按照加热流程来做，如果不是，则允许电池充电。用硬件版本区分是不是支持加热功能的硬件。
5. 修改间隔轮询时间: 大于等于3.0V设置1分钟时间，然后大于等于2.7V设置10分钟时间，然后小于2.7V设置30分钟时间
6. 当电池需求电流为0时候，关闭电池充放电管。
7. 电磁锁的开锁脉冲改为200mS。(需要注意，在充电柜中，一键开仓的命令是否会造成供电问题)
8. 加热功能符合条件后，关闭电池开关，关闭主动检测功能。
9. 修改: 原来满电标志不允许充电，改为SOC小于100%才能充电。
10. 加热模式下，电池有电流输入或者输出，则认为有故障，停止加热和充电，上报上位机(damage bit6)。只有重启PMS才能恢复正常。
11. 充电器经常性报故障(过流，短路，过压，过温)(平均每分钟0.5次，连续20分钟)，
    则记录上报故障类型，并上报到上位机。过流、短路、过压发生后不再允许充电；过温还可以充电。
12. 充电器发生故障时候，必须要确保该充电器状态被上传到上位机1次。
13. 测试模式下启动加热功能，用于工厂测试。增加测试命令。每次PMS重启后只能执行一次。
14. 增加PMS判断板载温度判断，大于等于60摄氏度则在心跳中设置断电标志。
15. 修改PMS判断板载温度，大于等于60摄氏度则停止充电，同时，需要恢复到55摄氏度一下才能再次充电。
16. 修改，在有微动开关的模式下，如果电池有故障不能放电，则不上报电池"电池放电连接器状态"故障。

V44:
1. 修改: 兼容天宝旧充电器的测试命令加热(加热模式的固件版本号为V1.0.0.1001)

V45:
1. 修改: 兼容天宝旧充电器不能设置超过7A电流的问题。只有新的充电器固件(读到版本号)才支持下发9A充电。

V46:
1. 修改: 通过使用充电器检测电池放电电压来确认电池连接器是否连接好的功能。
                原来的BUG: 在NFC通讯不好的情况下，可能会导致电池开关状态被反复开和关的设置，
                从而导致检测流程出现停止状态，以至于无法返回正确检测结果。

V1005:加热版本，修改判断天宝充电器支持新旧协议的版本号的条件。
            原来是2019.3.15号前发布的是旧版本，现在改为2019.3.20号前发布的是旧版本。
V100:加热版本；修改版本号，配合APK旧版本的加热禁仓功能。2019-5-27

V47(V1006):
1. 修改功能，无微动开关的仓也要启动检测电池连接器是否连接好的功能。

V48(V1007):
1. 增加功能，增加上报SN号(PMS和充电器)

V49(V1008):
1. 修改微动开关检测和电池连接器检测的先后顺序。
以前是同时开始，但是一般情况下都是电池连接器检测先完成。
现在改为，要等到微动开关检测完成后，再启动电池连接器检测。

2. 因为新版本安卓程序允许3次换电失败后可以换电成功，
所以这版本删除掉V43版本里的功能: 
"
16. 修改，在有微动开关的模式下，如果电池有故障不能放电，则不上报电池"电池放电连接器状态"故障。
"

V50(V1009):
1. BUG, 不能正常返回充电器SN到上位机。因为拷贝充电器返回的SN到缓存时候写错代码。现在修复。

V51(V1010):
1. 修复，上传连接器状态的策略修改。

V52(加热板V1011, 加热板V101):(使用高级优化编译选项)
1. 增加电池认证功能。电池需要认证PMS是合法的才能打开充放电管。
PMS需要认证电池是合法的才能允许给电池充电。

V53(加热板V1012):
1. 修改运行参数命令"SLAVE_RS485_CMD_R_RUNING_PARAM"中的处理函数，
原来返回的"启动PMS认证BAT功能"标志可能会是乱的，现在修复好。

V56(加热板V1013):
1. 修改FLASH参数区保存参数的代码。原来有参数版本更新时候，
参数结构体填充区的数据也会被拷贝到新版本参数中。
现在修复完毕。

V57:(加热板V1014)
1. 修改柜子检测微动开关和连接器连接状态的策略。
    //1.1 仓门锁关闭后，立刻检查电池是否在线。如果不在线，不再检测电池连接器连接状态，默认上传连接器没有连接好的状态。
    1.2 仓门锁关闭后，立刻检查微动开关是否到位。如果已经到位，不再检测微动开关是否到位，默认上传微动开关没有到位的状态。
    //1.3 仓门关锁后，在检测微动开关期间电池掉线。如果中间出现掉线或者偶尔掉线，不再检测电池连接器连接状态，默认上传连接器没有连接好的状态。
    1.4 仓门关锁后，在检测微动开关期间电池霍尔检测不到磁铁。如果中间出现掉霍尔信号或者偶尔掉霍尔信号，不再检测电池连接器连接状态，默认上传连接器没有连接好的状态。
2. 修改间隔轮询时间: 大于等于3.0V设置1分钟时间，然后大于等于2.7V设置24小时时间，然后小于2.7V设置72小时时间
3. 充电器增加温度参数
4. 关舱门后的必须持续轮询时间改为1分钟，原来是30分钟。

V58:(加热板V1015)
1. 修改协议中的心跳数据。
2. 主动检测流程中，如果一直在一个循环内跳不出来，5分钟中后会报故障。现在改为，报了该故障后，再过30分钟PMS自动重启。

V59:(加热板V1016)
1. 读取电池历史记录
2. BUG: 充电器升级过程中电池发生故障，升级失败。修复完成。
3. 心跳中增加认证状态的标志位
4. 电池最低电芯电压大于3.9V时候，保持一直轮询。
5. PMS重启时候，延迟允许后步进电机转动。根据配置的485地址决定延迟时间。
6. PMS重启时候，仓内风扇延迟允许启动。延迟时间是15秒。
7. 计算充电线路中的电阻值，然后判断充电是否是接触不良。
8. PMS重启时候容易报充电器故障。现在修改，开机10秒以后才能在心跳中上传充电器的故障标志fault128.
9. 与控制板通讯的485口的接收缓冲区改为512字节，原来是256字节。

V60:(加热板V1017)
1. 增加命令，"读取起始电池历史记录编号和记录条数"
2. 电池低功耗管理。间隔轮询的时间策略修改，改为，大于3.9V时候保持一直轮询，
大于等于3.0V设置1分钟时间，然后大于等于2.9V设置24小时时间，然后小于2.9V设置约1193小时时间(4294967秒)

3. 电池低功耗管理。间隔轮询期间，电芯最低电压小于3V，禁止BMS升级
4. 电池低功耗管理。间隔轮询期间，电芯最低电压小于3V，禁止读取BAT历史数据
5. 电池低功耗管理。间隔轮询期间，电芯最低电压小于3V，禁止PMS认证电池
6. PMS认证电池的功能，自动被打开，但是，认证结果不影响充电。
7. 当充电器有电流输出并且电池有电流输入时候，清除damage32(连接器未连接好)标志

V61:(加热板V1018)
1. 心跳中TVS温度值有BUG，搞错单位，导致上传的值很大几率被限在160。现在已经修改完毕。

V62:(加热板V1019)
1. 心跳中的最大温度上升速度选择策略修改。改为比较绝对值。

V63:(加热板V1020)
1. BUG修改: 电池低功耗管理。间隔轮询期间，同时电芯最低电压小于3V，禁止BMS升级
2. BUG修改: 电池低功耗管理。间隔轮询期间，同时电芯最低电压小于3V，禁止读取BAT历史数据
3. BUG修改: 电池低功耗管理。间隔轮询期间，同时电芯最低电压小于3V，禁止PMS认证电池

V1021(加热板)
1. BUG修改: 充电状态切换到加热状态时候，会出现电池丢失。现在已经修复。
2. BUG修改: 改为加热过程中检测电池有电流输出输入的故障。
    原来只检测1秒钟，现在改为: 电流大于3A就检测1S，电流大于1A就检测3S，有电流就6S。
*/
st_version_desc gl_version_desc_PMS = {
    #ifdef CABINET_WARMOR_SUPPLY_BY_CHARGER
    //硬件主版本号
    0x01,
    //硬件子版本号
    0x01,
    #else
    //硬件主版本号
    0x01,
    //硬件子版本号
    0x00,
    #endif
    //固件主版本号
    0x01,
    //固件子版本号
    0x00,
    //固件修订版本
    0x00,
    //固件build号32
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
unsigned char stl_SOC; // 单位百分比%
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

    unsigned char usart_tx_flag:1;/* 串口正在发送的标志 */
    unsigned char bits_reserved:7;
    /*串口配置参数*/
    struct serial_configure usart_config_param;
}st_charger_param;
GLOBAL_CMP st_charger_param gl_charger_paramdfg;
GLOBAL_CMP st_charger_param gl_rs485_param;
/*功率线模式切换；加热模式，充电器模式，放电模式*/
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
