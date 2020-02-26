

/**@file
 *
 * @defgroup app_fifo FIFO implementation
 * @{
 * @ingroup app_common
 *
 * @brief FIFO implementation.
 */

#ifndef NFC_CMD_PROCESS_H__
#define NFC_CMD_PROCESS_H__

#include "fm175xx.h"
#include "comm_board_protocol.h"

#ifndef NFC_CMD_PROCESS_C
#define GLOBAL_NCP extern
#else
#define GLOBAL_NCP
#endif



typedef struct{
unsigned char fn_code;
unsigned char (*function_rsp_cb)(unsigned char*, unsigned char, unsigned char*, unsigned char);
unsigned char (*function_snd_faild_cb)(unsigned char*, unsigned char, unsigned char*, unsigned char);
}st_nfc_cmd_table;

//判断是在bootloader状态还是app状态
typedef struct{
unsigned char hw_main_version;
unsigned char hw_sub_version;
unsigned char is_bl_mode; // 1,为处于bootloader模式
}st_bms_app_bl_status;
GLOBAL_NCP st_bms_app_bl_status gl_bms_app_bl_status[NFC_READER_COUNT_MAX];



// 1字节对齐
#pragma pack(1)

#define DEV_INFO_REG_BIGIN_ADDR 0
#define DEV_INFO_REG_END_ADDR 32
#define DEV_INFO_REG_TOTAL_SIZE (DEV_INFO_REG_END_ADDR-DEV_INFO_REG_BIGIN_ADDR+1)


#define READ_ONLY_REG_BIGIN_ADDR 256
#define READ_ONLY_REG_END_ADDR 322//315//308
#define READ_ONLY_REG_TOTAL_SIZE (READ_ONLY_REG_END_ADDR-READ_ONLY_REG_BIGIN_ADDR+1)

#define CTRL_REG_BIGIN_ADDR 512
#define CTRL_REG_END_ADDR 538//536
#define CTRL_REG_TOTAL_SIZE (CTRL_REG_END_ADDR-CTRL_REG_BIGIN_ADDR+1) 


#define PARAM_REG_BIGIN_ADDR 768
#define PARAM_REG_END_ADDR 832//830
#define PARAM_REG_TOTAL_SIZE (PARAM_REG_END_ADDR-PARAM_REG_BIGIN_ADDR+1)


#define USER_REG_BIGIN_ADDR 4096
#define USER_REG_END_ADDR 4100
#define USER_REG_TOTAL_SIZE (USER_REG_END_ADDR-USER_REG_BIGIN_ADDR+1)



typedef struct{
unsigned char bits16_H;
unsigned char bits16_L;
}st_modbus_reg_unit;

#define BMS_MODBUS_REG_MAX ((DEV_INFO_REG_TOTAL_SIZE)+\
                            (READ_ONLY_REG_TOTAL_SIZE)+\
                            (CTRL_REG_TOTAL_SIZE)+\
                            (PARAM_REG_TOTAL_SIZE)+\
                            (USER_REG_TOTAL_SIZE))//97
                            
#define BMS_REG_VALUE_READY_OK 0x5A
#define BMS_REG_VALUE_READY_FALSE 0




//BMS的信息
typedef struct{
unsigned char online; //BMS是否在线, 0x00, 离线；0x01,在线
unsigned char waiting_cmd_ack; //等待电池的返回信息
unsigned char addr_485; //BMS地址
unsigned char reg_value_ready;// 标记下面的寄存器值是否可用
// 标记下面的寄存器值是否已经刷新过了。1表示需要刷新，0表示已经刷新过
//当需要某个寄存器段刷新后才能使用时候，设置对应位。
unsigned char reg_value_is_need_flashed; 
st_modbus_reg_unit reg_unit[BMS_MODBUS_REG_MAX];//存放寄存器值
}st_bms_info;

#if (NFC_READER_COUNT_MAX > 0)
GLOBAL_NCP st_bms_info gl_bms_info_0;
#endif

#if (NFC_READER_COUNT_MAX > 1)
GLOBAL_NCP st_bms_info gl_bms_info_1;
#endif

#if (NFC_READER_COUNT_MAX > 2)
GLOBAL_NCP st_bms_info gl_bms_info_2;
#endif

#if (NFC_READER_COUNT_MAX > 3)
GLOBAL_NCP st_bms_info gl_bms_info_3;
#endif

#if (NFC_READER_COUNT_MAX > 4)
GLOBAL_NCP st_bms_info gl_bms_info_4;
#endif

#if (NFC_READER_COUNT_MAX > 5)
GLOBAL_NCP st_bms_info gl_bms_info_5;
#endif

GLOBAL_NCP st_bms_info *gl_bms_info_p[NFC_READER_COUNT_MAX];


//对齐结束
#pragma pack()


GLOBAL_NCP unsigned short MM_get_reg_addr_position_in_table(unsigned short pl_reg_addr);

GLOBAL_NCP unsigned char MM_rsp_cmd_process(unsigned char *pl_rcv_buff,
                                        unsigned char rcv_buf_len,
                                        unsigned char *pl_snd_buff,
                                        unsigned char snd_buf_len);
GLOBAL_NCP unsigned char MM_err_process(unsigned char *pl_rcv_buff,
                                        unsigned char rcv_buf_len,
                                        unsigned char *pl_snd_buff,
                                        unsigned char snd_buf_len);
GLOBAL_NCP unsigned char MM_read_reg_rsp_cb(unsigned char *pl_rcv_buff,
                                        unsigned char rcv_buf_len,
                                        unsigned char *pl_snd_buff,
                                        unsigned char snd_buf_len);
GLOBAL_NCP unsigned char MM_read_reg_err_cb(unsigned char *pl_rcv_buff,
                                        unsigned char rcv_buf_len,
                                        unsigned char *pl_snd_buff,
                                        unsigned char snd_buf_len);
GLOBAL_NCP unsigned char MM_write_reg_rsp_cb(unsigned char *pl_rcv_buff,
                                        unsigned char rcv_buf_len,
                                        unsigned char *pl_snd_buff,
                                        unsigned char snd_buf_len);
GLOBAL_NCP unsigned char MM_write_reg_err_cb(unsigned char *pl_rcv_buff,
                                        unsigned char rcv_buf_len,
                                        unsigned char *pl_snd_buff,
                                        unsigned char snd_buf_len);
GLOBAL_NCP unsigned char MM_write_multi_reg_rsp_cb(unsigned char *pl_rcv_buff,
                                        unsigned char rcv_buf_len,
                                        unsigned char *pl_snd_buff,
                                        unsigned char snd_buf_len);
GLOBAL_NCP unsigned char MM_write_multi_reg_err_cb(unsigned char *pl_rcv_buff,
                                        unsigned char rcv_buf_len,
                                        unsigned char *pl_snd_buff,
                                        unsigned char snd_buf_len);

GLOBAL_NCP unsigned char MM_snd_update_packet_cb(unsigned char *pl_rcv_buff,
                                        unsigned char rcv_buf_len,
                                        unsigned char *pl_snd_buff,
                                        unsigned char snd_buf_len);
GLOBAL_NCP unsigned char MM_snd_update_packet_err_cb(unsigned char *pl_rcv_buff,
                                        unsigned char rcv_buf_len,
                                        unsigned char *pl_snd_buff,
                                        unsigned char snd_buf_len);

GLOBAL_NCP unsigned char MM_authority_rsp_cb(unsigned char *pl_rcv_buff,
                                        unsigned char rcv_buf_len,
                                        unsigned char *pl_snd_buff,
                                        unsigned char snd_buf_len);

GLOBAL_NCP unsigned char MM_authority_err_cb(unsigned char *pl_rcv_buff,
                                        unsigned char rcv_buf_len,
                                        unsigned char *pl_snd_buff,
                                        unsigned char snd_buf_len);
GLOBAL_NCP unsigned char MM_history_record_rsp_cb(unsigned char *pl_rcv_buff,
                                        unsigned char rcv_buf_len,
                                        unsigned char *pl_snd_buff,
                                        unsigned char snd_buf_len);
GLOBAL_NCP unsigned char MM_history_record_err_cb(unsigned char *pl_rcv_buff,
                                            unsigned char rcv_buf_len,
                                            unsigned char *pl_snd_buff,
                                            unsigned char snd_buf_len);

GLOBAL_NCP unsigned char MM_snd_update_packet(unsigned char bms_index, unsigned char slave_addr,unsigned char *data, unsigned int data_len);
GLOBAL_NCP void MM_nfc_cmd_init(void);

GLOBAL_NCP void Authority_init(void);
GLOBAL_NCP void Authority_reset(void);
GLOBAL_NCP bool Authority_bat_pms_rsp(st_hub_cmd *pl_hub_cmd);
GLOBAL_NCP bool Authority_bat_pms_digest_rsp(st_hub_cmd *pl_hub_cmd);
GLOBAL_NCP bool Authority_pms_bat_rsp(st_hub_cmd *pl_hub_cmd);
GLOBAL_NCP bool Authority_NFC_cmd_process(st_hub_cmd *pl_hub_cmd);
GLOBAL_NCP unsigned char MM_authority_rsp_cb(unsigned char *pl_rcv_buff,
                                        unsigned char rcv_buf_len,
                                        unsigned char *pl_snd_buff,
                                        unsigned char snd_buf_len);
GLOBAL_NCP unsigned char MM_authority_err_cb(unsigned char *pl_rcv_buff,
                                        unsigned char rcv_buf_len,
                                        unsigned char *pl_snd_buff,
                                        unsigned char snd_buf_len);
GLOBAL_NCP unsigned char MM_snd_authority_data(unsigned char slave_addr,unsigned char* data, unsigned int data_len);
GLOBAL_NCP unsigned char MM_snd_authority_process(void);
GLOBAL_NCP void Authority_process(void);
GLOBAL_NCP bool Is_Authority_done_about_pms_bat(void);
GLOBAL_NCP bool Is_Authority_done_about_bat_pms(void);
GLOBAL_NCP bool Is_Authority_done(void);
GLOBAL_NCP void Authority_done_flag_clear(void);
GLOBAL_NCP void Authority_timer_count_cb(void);
GLOBAL_NCP void Authority_set_onoff(unsigned char onoff);
GLOBAL_NCP void Authority_set_done_flag_about_bat_pms(void);
GLOBAL_NCP unsigned char Authority_get_onoff_about_pms_bat(void);
#endif // 
/** @} */
