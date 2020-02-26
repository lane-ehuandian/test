
#define NTAG_APP_C
#include "fm175xx.h"
#include "ntag_app.h"

#include "type_a.h"
#include "nfc_crc.h"
#include "nfc_cmd_process.h"
#include "xmodem.h"
#include "modbus_master.h"
#include "battery_process.h"
//#include "elock.h"

//#define UNMASK_CODE


#define ERROR		1
#define	OK			0


unsigned char picc_ats[14];

unsigned long CID;                                         //如果没有用到CID，CID=0
unsigned char nfc_buf[16];
unsigned int nfc_bitlen;

//在开始传输NFC前置1，传输完成后置0
static bool gl_ntag_nfc_sending_receiving_flag = 0;

/****************************************************************/
/*名称: TypeA_PPSS 													           */
/*功能: 该函数实现ISO14443A-4协议中的RATS处理			     */
/*输入: PPS0用于决定PPS1是否传输，PPS1    */ 			
/*       ats 用于保存RATS命令返回的数据                */
/*输出:															                   */
/* OK: 应答正确												                 */
/* ERROR: 应答错误												             */
/****************************************************************/


unsigned char TypeA_PPSS(unsigned char Pps0, unsigned char Pps1, unsigned char *PpsAck)
{
    unsigned char send_buff[4];
    unsigned char send_num;
    unsigned char result;
	unsigned int rece_bitlen;
    
    send_buff[0] = 0xD0;
    send_num=1;
    if(Pps0&0x10)                                        //如果需要改变传输速率
    {
        send_buff[1] = Pps0;
        send_buff[2] = Pps1;
        send_num += 2;
    }
    Write_Reg(BitFramingReg,0x00);
    Set_BitMask(TxModeReg,0x80);                         //设置发送CRC

    Set_BitMask(RxModeReg,0x80);                         //设置接收CRC
    Clear_BitMask(Status2Reg,0x08);
    Pcd_SetTimer(5);
    Clear_FIFO();
    result = Pcd_Comm(Transceive,send_buff,send_num,PpsAck,&rece_bitlen);
    return result;
}

/****************************************************************/
/*名称: TypeA_RATS 													           */
/*功能: 该函数实现ISO14443A-4协议中的RATS处理			     */
/*输入: param byte bit8-bit5：FSDI,bit4-bit0 ：CID     */ 			
/*       ats 用于保存RATS命令返回的数据                */
/*输出:															                   */
/* OK: 应答正确												                 */
/* ERROR: 应答错误												             */
/****************************************************************/


unsigned char TypeA_RATS(unsigned char param, unsigned char *ats)
{
    unsigned char send_buff[4];
    unsigned char result;
	unsigned int rece_bitlen;
    
    send_buff[0] = 0xE0;
    send_buff[1] = param;                                //
    CID = param&0x0f;
    Write_Reg(BitFramingReg,0x00);
    Set_BitMask(TxModeReg,0x80);                         //设置发送CRC
    Set_BitMask(RxModeReg,0x80);                         //设置接收CRC
    Clear_BitMask(Status2Reg,0x08);
    Pcd_SetTimer(5);
    Clear_FIFO();
    result = Pcd_Comm(Transceive,send_buff,2,ats,&rece_bitlen);
    return result;
}



/****************************************************************/
/*名称: FM11NC08Active 													*/
/*功能: 该函数实现ISO14443A-4协议RATS和PPSS 往后的数据就是透明传输*/
/*输入:  	ats RATS命令返回数据		*/
/*输出:															*/
/* OK: 应答正确													*/
/* ERROR: 应答错误												*/
/****************************************************************/

unsigned char FM11NC08Active(unsigned char *ats)
{
    unsigned char result;
    result=TypeA_RATS(0x20,ats);                                //0x20表示读卡器只能接收32字节长度的数据
    return result;                                              //如果需要接收更长帧的数据，可以修改读卡器接收函数，最长可以接收256字节
}

static unsigned char NTAG_rcv_CRC_check(unsigned char *pl_rcv_data, unsigned int pl_rcv_len)
{
    unsigned char vl_crc[2];
    unsigned char vl_rcv_crc[2];
    unsigned int vl_rcv_crc_pos;
    
    if(pl_rcv_len < 2)
        return 0;
    #if 0
    //crc 校验
    ComputeCrc(CRC_A, pl_rcv_data, (pl_rcv_len - 2), &vl_crc[0], &vl_crc[1]);
    if((pl_rcv_data[pl_rcv_len - 2] != vl_crc[0])||(pl_rcv_data[pl_rcv_len - 1] != vl_crc[1]))
    {
        return 0;
    }
    #else
    if(0x03 == pl_rcv_data[4])
    {
        /*是读寄存器命令*/

        vl_rcv_crc_pos = pl_rcv_data[5] + 6;
        vl_rcv_crc[0] = pl_rcv_data[vl_rcv_crc_pos];
        vl_rcv_crc[1] = pl_rcv_data[vl_rcv_crc_pos+1];

        //crc 校验
        ComputeCrc(CRC_A, pl_rcv_data, (pl_rcv_len - 2), &vl_crc[0], &vl_crc[1]);
        if((vl_rcv_crc[0] != vl_crc[0])||(vl_rcv_crc[1] != vl_crc[1]))
        {
            return 0;
        }
    }
    else
    {
        //crc 校验
        ComputeCrc(CRC_A, pl_rcv_data, (pl_rcv_len - 2), &vl_crc[0], &vl_crc[1]);
        if((pl_rcv_data[pl_rcv_len - 2] != vl_crc[0])||(pl_rcv_data[pl_rcv_len - 1] != vl_crc[1]))
        {
            return 0;
        }
    }
    
    #endif
    return 1;
    
}

//读卡器搜卡失败次数计算最大值
#define NTAG_FOUND_NFC_CNT_MAX 30 // 2018-9-11
#define NTAG_FOUND_NFC_CNT_MAX_WHEN_ELOCK_OPEN 8
static unsigned char gl_ntag_found_nfc_cnt[NFC_READER_COUNT_MAX];

void NTAG_clear_reg_valid_flag(unsigned char bms_index)
{
    if((//(ELOCK_LOCK == ELock_get_status((ENUM_ELOCK_INDEX)ENUM_ELOCK_NUM_0))&&
        (gl_ntag_found_nfc_cnt[bms_index] >= NTAG_FOUND_NFC_CNT_MAX))
        ||//((ELOCK_OPEN == ELock_get_status((ENUM_ELOCK_INDEX)ENUM_ELOCK_NUM_0))&&
        (gl_ntag_found_nfc_cnt[bms_index] >= NTAG_FOUND_NFC_CNT_MAX_WHEN_ELOCK_OPEN))//)
    {
        Battery_clear_reg_valid_flag(bms_index);
        gl_ntag_found_nfc_cnt[bms_index] = 0;
    }
    else
    {
        gl_ntag_found_nfc_cnt[bms_index]++;
    }
}

typedef enum {
ENUM_NTAG_STATE_INIT,
ENUM_NTAG_STATE_SEARCH,    
ENUM_NTAG_STATE_TRANS

}enum_ntag_state;

//天线选择
unsigned char gl_NTAG_task_nfc_ant_selc[NFC_READER_COUNT_MAX];
enum_ntag_state gl_NTAG_task_state[NFC_READER_COUNT_MAX];//={ENUM_NTAG_STATE_INIT, ENUM_NTAG_STATE_INIT};

unsigned char vl_BMS_index; //从0开始

static unsigned int sl_test_cnt = 0;
unsigned char NTAG_task_x(void)
{
    unsigned char result;
    //unsigned char vl_BMS_index; //从0开始
    unsigned int vl_tmp_cnt;
    
    if(gl_NTAG_task_state[vl_BMS_index] == ENUM_NTAG_STATE_INIT)
    {
        //清除寄存器有效的标志
        NTAG_clear_reg_valid_flag(vl_BMS_index);
        //暂时在这里设置电池离线
        gl_bms_info_p[vl_BMS_index]->online = 0;        
        FM175XX_HardReset();
        FM175XX_SoftReset();
        if(gl_NTAG_task_nfc_ant_selc[vl_BMS_index])
        {
            Set_Rf(1);
        }
        else
        {
            Set_Rf(2);
        }
        Pcd_ConfigISOType(0);
        gl_NTAG_task_state[vl_BMS_index] = ENUM_NTAG_STATE_SEARCH;
    }
    
    if(gl_NTAG_task_state[vl_BMS_index] == ENUM_NTAG_STATE_SEARCH)
    {        
        result = TypeA_CardActivate(PICC_ATQA,PICC_UID,PICC_SAK);
        if (result==OK)
        {
            result = FM11NC08Active(picc_ats);
            if(result == OK)
            {
                //搜卡成功
                MM_NFC_init_done(vl_BMS_index);
                FM17522_Delayms(10);//因为卡片的程序中加了延迟，这里也相应延迟一下

                rt_kprintf("\r\nDetected BAT by NFC!");
                //////////////////////////////////////////////////////////////////////////
                //////////////////////////////////////////////////////////////////////////
                //读取电池信息
                Battery_info_polling_Process(vl_BMS_index);
                gl_NTAG_task_state[vl_BMS_index] = ENUM_NTAG_STATE_TRANS;
                //暂时在这里设置电池在线
                gl_bms_info_p[vl_BMS_index]->online = 1;
                //这里只是让程序再跑出去进行一些参数设置，没有实际作用
                result = Read_Reg(TxControlReg);
                //复位读卡失败计算
                gl_ntag_found_nfc_cnt[vl_BMS_index] = 0;
            }
            else
            {
                //清除命令缓存
                Battery_cmd_buf_clear(vl_BMS_index);
                gl_bms_info_p[vl_BMS_index]->online = 0;
                //清除寄存器有效的标志
                NTAG_clear_reg_valid_flag(vl_BMS_index);
                #ifdef BAT_ERROR_CHECK_DELAY_AFTER_BAT_UPDATE
                if(gl_bms_app_bl_status[gl_modbus_param.BMS_index].is_bl_mode)
                {
                    sl_bat_error_check_delay_after_bat_update_cnt = 120000; // 120秒
                    //清除主动测试电池故障的流程
                    Battery_check_bat_clear_on_and_error();
                }
                #endif
                //清除仓中电池处于bootloader模式的标志
                gl_bms_app_bl_status[vl_BMS_index].is_bl_mode = 0;
                gl_NTAG_task_nfc_ant_selc[vl_BMS_index] = ~gl_NTAG_task_nfc_ant_selc[vl_BMS_index];

                if (0 == Battery_get_reg_valid_flag(vl_BMS_index))
                {
                    //进入低功耗
                    FM175XX_SoftPowerdown();
                    //重新初始化读卡器任务
                    NTAG_task_reset();
                }
                return ERROR;
            }
        }
        else
        {
            //清除命令缓存
            Battery_cmd_buf_clear(vl_BMS_index);
            gl_bms_info_p[vl_BMS_index]->online = 0;
            //清除寄存器有效的标志
            NTAG_clear_reg_valid_flag(vl_BMS_index);
            #ifdef BAT_ERROR_CHECK_DELAY_AFTER_BAT_UPDATE
            if(gl_bms_app_bl_status[gl_modbus_param.BMS_index].is_bl_mode)
            {
                sl_bat_error_check_delay_after_bat_update_cnt = 120000; // 120秒
                //清除主动测试电池故障的流程
                Battery_check_bat_clear_on_and_error();
            }
            #endif
            //清除仓中电池处于bootloader模式的标志
            gl_bms_app_bl_status[vl_BMS_index].is_bl_mode = 0;
            gl_NTAG_task_nfc_ant_selc[vl_BMS_index] = ~gl_NTAG_task_nfc_ant_selc[vl_BMS_index];
            if (0 == Battery_get_reg_valid_flag(vl_BMS_index))
            {
                //进入低功耗
                FM175XX_SoftPowerdown();
                //重新初始化读卡器任务
                NTAG_task_reset();
            }
            return ERROR;
        }
    }

    if(gl_NTAG_task_state[vl_BMS_index] == ENUM_NTAG_STATE_TRANS)
    {
        //填充要发送的数据
        //如果是正在升级，也退出
        if((gl_bms_update_state[vl_BMS_index] == ENUM_BMS_UPDATE_XMODEM_PROCESS)&&
            (gl_xmodem_state[vl_BMS_index] != ENUM_XMODEM_IDLE))
        {
            //如果正在升级
        }
        else
        {      
            if(0 == Battery_send_cmd(vl_BMS_index))
            {
                //没有数据，退出
                if((sl_bat_poll_param.is_poll_allways_en_flag == 0)&&
                    (sl_bat_poll_param.poll_inerval_timer_cnt >= BAT_POLL_ON_TIME_MAX))
                {
                    //进入低功耗
                    FM175XX_SoftPowerdown();
                }
                vl_BMS_index++;
                vl_BMS_index %= NFC_READER_COUNT_MAX;
                return OK;
            }
        }
        //判断是否是该读卡器发送数据
        if(vl_BMS_index != gl_modbus_param.BMS_index)
        {
            vl_BMS_index++;
            vl_BMS_index %= NFC_READER_COUNT_MAX;
            return OK;
        }
        for(;;)
        {
            if(!gl_modbus_param.MM_NFC_snd_len[vl_BMS_index])
            {
                vl_BMS_index++;
                vl_BMS_index %= NFC_READER_COUNT_MAX;
                return OK;
            }
            gl_ntag_nfc_sending_receiving_flag = 1;
            gl_modbus_param.MM_NFC_rcv_len[vl_BMS_index] = 0;
            Pcd_SetTimer(300);
            result = Pcd_Comm(Transceive,
                gl_modbus_param.MM_NFC_snd_buff[vl_BMS_index],
                gl_modbus_param.MM_NFC_snd_len[vl_BMS_index],
                gl_modbus_param.MM_NFC_rcv_buff[vl_BMS_index],
                &vl_tmp_cnt);
            gl_ntag_nfc_sending_receiving_flag = 0;
            //
            gl_modbus_param.MM_NFC_rcv_len[vl_BMS_index] = (vl_tmp_cnt+7)/8;
            /////////////////////
            sl_test_cnt++;
            ////////////////////
            if(result == OK)
            {
                if(0 == NTAG_rcv_CRC_check((unsigned char *)(gl_modbus_param.MM_NFC_rcv_buff[vl_BMS_index]), 
                                            gl_modbus_param.MM_NFC_rcv_len[vl_BMS_index]))
                {
                    result = ERROR;
                    sl_test_cnt = 0;
                }
            }

            if (result==OK)
            {             
                MM_NFC_snd_done_cb();
                if(Battery_is_one_cmd_buf_empty(vl_BMS_index))
                {
                    vl_BMS_index++;
                    vl_BMS_index %= NFC_READER_COUNT_MAX;
                    break;
                }
                //如果是正在升级，也退出
                if((gl_bms_update_state[vl_BMS_index] == ENUM_BMS_UPDATE_XMODEM_PROCESS)&&
                    (gl_xmodem_state[vl_BMS_index] != ENUM_XMODEM_IDLE))
                {
                    //如果正在升级
                    break;
                }
                else
                {
                    //填充要发送的数据
                    if(0 == Battery_send_cmd(vl_BMS_index))
                    {
                        //没有数据，退出
                        break;
                    }
                }
            }
            else
            {
                sl_test_cnt = 0;
                gl_modbus_param.MM_NFC_rcv_len[vl_BMS_index] = 0;
                if(0 == MM_NFC_snd_ERR())
                {
                    //继续重发
                    continue;
                }
                //重新搜卡
                gl_NTAG_task_state[vl_BMS_index] = ENUM_NTAG_STATE_SEARCH;
                //进入低功耗
                FM175XX_SoftPowerdown();
                //重新初始化读卡器任务
                NTAG_task_reset();
                return ERROR;
            }
        }
    }
    return OK;
}

unsigned char NTAG_task_reset(void)
{
    gl_modbus_param.MM_NFC_nead_reset[vl_BMS_index] = 1;
    gl_NTAG_task_state[vl_BMS_index] = ENUM_NTAG_STATE_INIT;
    return 1;
}

void NTAG_task_process(void)
{
    static unsigned char vl_one_time = 0;
    if(!vl_one_time)
    {
        unsigned char i;
        
        vl_one_time = 1;
        for(i = 0; i < NFC_READER_COUNT_MAX; i++)
        {
            gl_modbus_param.mm_state[i] = ENUM_MM_NFC_INIT;
            gl_modbus_param.MM_NFC_snd_len[i] = 0;//发送缓存中存在的数据长度
            gl_modbus_param.MM_NFC_rcv_len[i] = 0;//接收缓存中存在的数据长度
            gl_modbus_param.MM_NFC_snd_err_cnt[i] = 0;//发送错误计数
            gl_modbus_param.MM_NFC_nead_reset[i] = 0; //等于1表示NFC读卡器需要重启
            gl_modbus_param.MM_IDLE_timer_cnt = 0;

            gl_NTAG_task_state[i] = ENUM_NTAG_STATE_INIT;

            gl_ntag_found_nfc_cnt[i] = 0;
        }
        #ifdef UNMASK_CODE
        I2C_NFC_Init();
        #endif
        MM_nfc_cmd_init();
    }

    if(gl_modbus_param.MM_NFC_nead_reset[vl_BMS_index])
    {
        gl_modbus_param.MM_NFC_nead_reset[vl_BMS_index] = 0;

        #ifdef UNMASK_CODE
        I2C_NFC_Init();
        #endif
        MM_init(vl_BMS_index);

        vl_BMS_index++;
        vl_BMS_index %= NFC_READER_COUNT_MAX;

    }
    NTAG_task_x();
}


//得到是否在发射NFC读取数据的标志
bool NTAG_get_nfc_working_status(void)
{
    return gl_ntag_nfc_sending_receiving_flag;
}

