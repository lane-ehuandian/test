
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

unsigned long CID;                                         //���û���õ�CID��CID=0
unsigned char nfc_buf[16];
unsigned int nfc_bitlen;

//�ڿ�ʼ����NFCǰ��1��������ɺ���0
static bool gl_ntag_nfc_sending_receiving_flag = 0;

/****************************************************************/
/*����: TypeA_PPSS 													           */
/*����: �ú���ʵ��ISO14443A-4Э���е�RATS����			     */
/*����: PPS0���ھ���PPS1�Ƿ��䣬PPS1    */ 			
/*       ats ���ڱ���RATS����ص�����                */
/*���:															                   */
/* OK: Ӧ����ȷ												                 */
/* ERROR: Ӧ�����												             */
/****************************************************************/


unsigned char TypeA_PPSS(unsigned char Pps0, unsigned char Pps1, unsigned char *PpsAck)
{
    unsigned char send_buff[4];
    unsigned char send_num;
    unsigned char result;
	unsigned int rece_bitlen;
    
    send_buff[0] = 0xD0;
    send_num=1;
    if(Pps0&0x10)                                        //�����Ҫ�ı䴫������
    {
        send_buff[1] = Pps0;
        send_buff[2] = Pps1;
        send_num += 2;
    }
    Write_Reg(BitFramingReg,0x00);
    Set_BitMask(TxModeReg,0x80);                         //���÷���CRC

    Set_BitMask(RxModeReg,0x80);                         //���ý���CRC
    Clear_BitMask(Status2Reg,0x08);
    Pcd_SetTimer(5);
    Clear_FIFO();
    result = Pcd_Comm(Transceive,send_buff,send_num,PpsAck,&rece_bitlen);
    return result;
}

/****************************************************************/
/*����: TypeA_RATS 													           */
/*����: �ú���ʵ��ISO14443A-4Э���е�RATS����			     */
/*����: param byte bit8-bit5��FSDI,bit4-bit0 ��CID     */ 			
/*       ats ���ڱ���RATS����ص�����                */
/*���:															                   */
/* OK: Ӧ����ȷ												                 */
/* ERROR: Ӧ�����												             */
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
    Set_BitMask(TxModeReg,0x80);                         //���÷���CRC
    Set_BitMask(RxModeReg,0x80);                         //���ý���CRC
    Clear_BitMask(Status2Reg,0x08);
    Pcd_SetTimer(5);
    Clear_FIFO();
    result = Pcd_Comm(Transceive,send_buff,2,ats,&rece_bitlen);
    return result;
}



/****************************************************************/
/*����: FM11NC08Active 													*/
/*����: �ú���ʵ��ISO14443A-4Э��RATS��PPSS ��������ݾ���͸������*/
/*����:  	ats RATS���������		*/
/*���:															*/
/* OK: Ӧ����ȷ													*/
/* ERROR: Ӧ�����												*/
/****************************************************************/

unsigned char FM11NC08Active(unsigned char *ats)
{
    unsigned char result;
    result=TypeA_RATS(0x20,ats);                                //0x20��ʾ������ֻ�ܽ���32�ֽڳ��ȵ�����
    return result;                                              //�����Ҫ���ո���֡�����ݣ������޸Ķ��������պ���������Խ���256�ֽ�
}

static unsigned char NTAG_rcv_CRC_check(unsigned char *pl_rcv_data, unsigned int pl_rcv_len)
{
    unsigned char vl_crc[2];
    unsigned char vl_rcv_crc[2];
    unsigned int vl_rcv_crc_pos;
    
    if(pl_rcv_len < 2)
        return 0;
    #if 0
    //crc У��
    ComputeCrc(CRC_A, pl_rcv_data, (pl_rcv_len - 2), &vl_crc[0], &vl_crc[1]);
    if((pl_rcv_data[pl_rcv_len - 2] != vl_crc[0])||(pl_rcv_data[pl_rcv_len - 1] != vl_crc[1]))
    {
        return 0;
    }
    #else
    if(0x03 == pl_rcv_data[4])
    {
        /*�Ƕ��Ĵ�������*/

        vl_rcv_crc_pos = pl_rcv_data[5] + 6;
        vl_rcv_crc[0] = pl_rcv_data[vl_rcv_crc_pos];
        vl_rcv_crc[1] = pl_rcv_data[vl_rcv_crc_pos+1];

        //crc У��
        ComputeCrc(CRC_A, pl_rcv_data, (pl_rcv_len - 2), &vl_crc[0], &vl_crc[1]);
        if((vl_rcv_crc[0] != vl_crc[0])||(vl_rcv_crc[1] != vl_crc[1]))
        {
            return 0;
        }
    }
    else
    {
        //crc У��
        ComputeCrc(CRC_A, pl_rcv_data, (pl_rcv_len - 2), &vl_crc[0], &vl_crc[1]);
        if((pl_rcv_data[pl_rcv_len - 2] != vl_crc[0])||(pl_rcv_data[pl_rcv_len - 1] != vl_crc[1]))
        {
            return 0;
        }
    }
    
    #endif
    return 1;
    
}

//�������ѿ�ʧ�ܴ����������ֵ
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

//����ѡ��
unsigned char gl_NTAG_task_nfc_ant_selc[NFC_READER_COUNT_MAX];
enum_ntag_state gl_NTAG_task_state[NFC_READER_COUNT_MAX];//={ENUM_NTAG_STATE_INIT, ENUM_NTAG_STATE_INIT};

unsigned char vl_BMS_index; //��0��ʼ

static unsigned int sl_test_cnt = 0;
unsigned char NTAG_task_x(void)
{
    unsigned char result;
    //unsigned char vl_BMS_index; //��0��ʼ
    unsigned int vl_tmp_cnt;
    
    if(gl_NTAG_task_state[vl_BMS_index] == ENUM_NTAG_STATE_INIT)
    {
        //����Ĵ�����Ч�ı�־
        NTAG_clear_reg_valid_flag(vl_BMS_index);
        //��ʱ���������õ������
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
                //�ѿ��ɹ�
                MM_NFC_init_done(vl_BMS_index);
                FM17522_Delayms(10);//��Ϊ��Ƭ�ĳ����м����ӳ٣�����Ҳ��Ӧ�ӳ�һ��

                rt_kprintf("\r\nDetected BAT by NFC!");
                //////////////////////////////////////////////////////////////////////////
                //////////////////////////////////////////////////////////////////////////
                //��ȡ�����Ϣ
                Battery_info_polling_Process(vl_BMS_index);
                gl_NTAG_task_state[vl_BMS_index] = ENUM_NTAG_STATE_TRANS;
                //��ʱ���������õ������
                gl_bms_info_p[vl_BMS_index]->online = 1;
                //����ֻ���ó������ܳ�ȥ����һЩ�������ã�û��ʵ������
                result = Read_Reg(TxControlReg);
                //��λ����ʧ�ܼ���
                gl_ntag_found_nfc_cnt[vl_BMS_index] = 0;
            }
            else
            {
                //��������
                Battery_cmd_buf_clear(vl_BMS_index);
                gl_bms_info_p[vl_BMS_index]->online = 0;
                //����Ĵ�����Ч�ı�־
                NTAG_clear_reg_valid_flag(vl_BMS_index);
                #ifdef BAT_ERROR_CHECK_DELAY_AFTER_BAT_UPDATE
                if(gl_bms_app_bl_status[gl_modbus_param.BMS_index].is_bl_mode)
                {
                    sl_bat_error_check_delay_after_bat_update_cnt = 120000; // 120��
                    //����������Ե�ع��ϵ�����
                    Battery_check_bat_clear_on_and_error();
                }
                #endif
                //������е�ش���bootloaderģʽ�ı�־
                gl_bms_app_bl_status[vl_BMS_index].is_bl_mode = 0;
                gl_NTAG_task_nfc_ant_selc[vl_BMS_index] = ~gl_NTAG_task_nfc_ant_selc[vl_BMS_index];

                if (0 == Battery_get_reg_valid_flag(vl_BMS_index))
                {
                    //����͹���
                    FM175XX_SoftPowerdown();
                    //���³�ʼ������������
                    NTAG_task_reset();
                }
                return ERROR;
            }
        }
        else
        {
            //��������
            Battery_cmd_buf_clear(vl_BMS_index);
            gl_bms_info_p[vl_BMS_index]->online = 0;
            //����Ĵ�����Ч�ı�־
            NTAG_clear_reg_valid_flag(vl_BMS_index);
            #ifdef BAT_ERROR_CHECK_DELAY_AFTER_BAT_UPDATE
            if(gl_bms_app_bl_status[gl_modbus_param.BMS_index].is_bl_mode)
            {
                sl_bat_error_check_delay_after_bat_update_cnt = 120000; // 120��
                //����������Ե�ع��ϵ�����
                Battery_check_bat_clear_on_and_error();
            }
            #endif
            //������е�ش���bootloaderģʽ�ı�־
            gl_bms_app_bl_status[vl_BMS_index].is_bl_mode = 0;
            gl_NTAG_task_nfc_ant_selc[vl_BMS_index] = ~gl_NTAG_task_nfc_ant_selc[vl_BMS_index];
            if (0 == Battery_get_reg_valid_flag(vl_BMS_index))
            {
                //����͹���
                FM175XX_SoftPowerdown();
                //���³�ʼ������������
                NTAG_task_reset();
            }
            return ERROR;
        }
    }

    if(gl_NTAG_task_state[vl_BMS_index] == ENUM_NTAG_STATE_TRANS)
    {
        //���Ҫ���͵�����
        //���������������Ҳ�˳�
        if((gl_bms_update_state[vl_BMS_index] == ENUM_BMS_UPDATE_XMODEM_PROCESS)&&
            (gl_xmodem_state[vl_BMS_index] != ENUM_XMODEM_IDLE))
        {
            //�����������
        }
        else
        {      
            if(0 == Battery_send_cmd(vl_BMS_index))
            {
                //û�����ݣ��˳�
                if((sl_bat_poll_param.is_poll_allways_en_flag == 0)&&
                    (sl_bat_poll_param.poll_inerval_timer_cnt >= BAT_POLL_ON_TIME_MAX))
                {
                    //����͹���
                    FM175XX_SoftPowerdown();
                }
                vl_BMS_index++;
                vl_BMS_index %= NFC_READER_COUNT_MAX;
                return OK;
            }
        }
        //�ж��Ƿ��Ǹö�������������
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
                //���������������Ҳ�˳�
                if((gl_bms_update_state[vl_BMS_index] == ENUM_BMS_UPDATE_XMODEM_PROCESS)&&
                    (gl_xmodem_state[vl_BMS_index] != ENUM_XMODEM_IDLE))
                {
                    //�����������
                    break;
                }
                else
                {
                    //���Ҫ���͵�����
                    if(0 == Battery_send_cmd(vl_BMS_index))
                    {
                        //û�����ݣ��˳�
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
                    //�����ط�
                    continue;
                }
                //�����ѿ�
                gl_NTAG_task_state[vl_BMS_index] = ENUM_NTAG_STATE_SEARCH;
                //����͹���
                FM175XX_SoftPowerdown();
                //���³�ʼ������������
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
            gl_modbus_param.MM_NFC_snd_len[i] = 0;//���ͻ����д��ڵ����ݳ���
            gl_modbus_param.MM_NFC_rcv_len[i] = 0;//���ջ����д��ڵ����ݳ���
            gl_modbus_param.MM_NFC_snd_err_cnt[i] = 0;//���ʹ������
            gl_modbus_param.MM_NFC_nead_reset[i] = 0; //����1��ʾNFC��������Ҫ����
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


//�õ��Ƿ��ڷ���NFC��ȡ���ݵı�־
bool NTAG_get_nfc_working_status(void)
{
    return gl_ntag_nfc_sending_receiving_flag;
}

