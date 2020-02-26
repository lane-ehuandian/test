/*************************************************************/
//2014.07.15�޸İ�
/*************************************************************/
#define FM17522_C
#include "FM17522_process.h"
#include "fm175xx.h"
#include "IO_config.h"
#include <rtthread.h>
#include <rtdevice.h>

#define ERROR		1
#define	OK			0


/*************************************************************/
/*��������	    Read_Reg                                                             */
/*���ܣ�	    ���Ĵ�������                                                 */
/*���������	reg_add���Ĵ�����ַ                           */
/*����ֵ��	    �Ĵ�����ֵ                                               */
/*************************************************************/
unsigned char Read_Reg(unsigned char reg_addr)
{
    unsigned char reg_value;

    i2c_read_fm17522_reg(reg_addr, &reg_value);
            
    return reg_value;
}


#if 0
/*************************************************************/
/*��������	    Read_Reg All                                 */
/*���ܣ�	    ��64���Ĵ�������                             */ 
/*���������	��                                           */
/*�������		*reg_value���Ĵ�����ֵָ��,����Ϊ64�ֽ�      */
/*����ֵ��	    OK                                           */
/*************************************************************/

unsigned char Read_Reg_All(pt *pl_pt)
{
    PT_BEGIN(pl_pt);
	for (gl_fm_RRA_param.i=0;gl_fm_RRA_param.i<64;gl_fm_RRA_param.i++)	   
    {   
        I2CREAD(gl_fm_RRA_param.i, *(gl_fm_RRA_param.reg_value+gl_fm_RRA_param.i));
   }
    PT_END(pl_pt);
    PT_INIT(pl_pt);
    return OK;
}
#endif

/*************************************************************/
/*��������	    Write_Reg									 */
/*���ܣ�	    д�Ĵ�������								 */
/*															 */
/*���������	reg_add���Ĵ�����ַ��reg_value���Ĵ�����ֵ	 */
/*����ֵ��	    OK											 */
/*				ERROR										 */
/*************************************************************/

unsigned char Write_Reg(unsigned char reg_addr, unsigned char reg_value)
{
    i2c_write_fm17522_reg(reg_addr, reg_value);
	return OK;
}



/*************************************************************/
/*��������	    Read_FIFO									 */
/*���ܣ�	    ��ȡFIFO  									 */
/*															 */
/*���������	length����ȡFIFO���ݳ���					 */
/*���������	*fifo_data��FIFO���ݴ��ָ��				 */
/*����ֵ��	    											 */
/*															 */
/*************************************************************/


    
unsigned char Read_FIFO(unsigned char length, unsigned char *fifo_data)
{	

    i2c_read_fm17522_fifo(FIFODataReg, fifo_data, length);
    
	return 0;
}




/*************************************************************/
/*��������	    Write_FIFO									 */
/*���ܣ�	    д��FIFO  									 */
/*															 */
/*���������	length����ȡ���ݳ��ȣ�						 */
/*				*fifo_data�����ݴ��ָ��					 */
/*���������												 */
/*����ֵ��	    											 */
/*															 */
/*************************************************************/

    
unsigned char Write_FIFO(unsigned char length, unsigned char *fifo_data)
{
    i2c_write_fm17522_fifo(FIFODataReg, fifo_data, length);
	return 0;
}





/*************************************************************/
/*��������	    Clear_FIFO									 */
/*���ܣ�	    ���FIFO  									 */
/*															 */
/*���������	��											*/
/*���������												 */
/*����ֵ��	    OK											 */
/*				ERROR										 */
/*************************************************************/

unsigned char Clear_FIFO(void)
{
    unsigned char result;
    //gl_fm_SBM_param.reg_add = FIFOLevelReg;
    //gl_fm_SBM_param.mask = 0x80;
    //FUNC_WAIT_WITH_RETURN(Set_BitMask, gl_fm_SBM_param);
    Set_BitMask(FIFOLevelReg,0x80);

    //gl_fm_RG_param.reg_add = FIFOLevelReg;
    //FUNC_WAIT_WITH_RESULT_AND_RETURN(Read_Reg,gl_fm_RG_param,gl_fm_CFIFO_param.result);
    result = Read_Reg(FIFOLevelReg);
    
    if (result == 0)
    {
        return OK;
    }
    else
    {
        return ERROR;
    }
}


/*************************************************************/
/*��������	    Set_BitMask									 */
/*���ܣ�	    ��λ�Ĵ�������								 */
/*���������	reg_add���Ĵ�����ַ��mask���Ĵ�����λ		 */
/*����ֵ��	    OK											 */
/*				ERROR										 */
/*************************************************************/

unsigned char Set_BitMask(unsigned char reg_addr, unsigned char mask)
{
    unsigned char result;
    result = Read_Reg(reg_addr);
    Write_Reg(reg_addr, result|mask);
    result = OK;
	return result;
}



/*********************************************/
/*��������	    Clear_BitMask   */
/*���ܣ�	    ���λ�Ĵ�������    */
				
/*���������	reg_add���Ĵ�����ַ��mask���Ĵ������λ */
/*����ֵ��	    OK  
				ERROR   */
/*********************************************/

unsigned char Clear_BitMask(unsigned char reg_addr, unsigned char mask)
{
    unsigned char result;
    result = Read_Reg(reg_addr);
    Write_Reg(reg_addr, result & (~mask));
    result = OK;
	return result;
}



/*********************************************/
/*��������	    Set_RF  */
/*���ܣ�	    ������Ƶ���    */
				
/*���������	mode����Ƶ���ģʽ  
				0���ر����
				1,����TX1���
				2,����TX2���
				3��TX1��TX2�������TX2Ϊ�������  */
/*����ֵ��	    OK
				ERROR   */
/*********************************************/

unsigned char Set_Rf(unsigned char mode)
{
    unsigned char result;
    
    result = Read_Reg(TxControlReg);
    
    if	((result&0x03) == mode)//chenke, �ж��Ƿ�TX1��TX2�Ѿ����
    {
	    return OK;
    }
    if (mode==0)
	{
	    result=Clear_BitMask(TxControlReg,0x03); //�ر�TX1��TX2���
	}
    if (mode==1)
	{
    	result=Set_BitMask(TxControlReg,0x01); //����TX1���
	}
    if (mode==2)
	{
    	result=Set_BitMask(TxControlReg,0x02); //����TX2���
	}
    if (mode==3)
	{
    	result=Set_BitMask(TxControlReg,0x03); //��TX1��TX2���
	}
    //Delay_100us(1000);//��TX�������Ҫ��ʱ�ȴ������ز��ź��ȶ�
    FM17522_Delayms(1);

    return result;
}

void Pcd_Comm_timer_cb(void)
{
    if(gl_fm17522_param.i2c_comm_timeout_cnt)
        gl_fm17522_param.i2c_comm_timeout_cnt--;
}
/*********************************************/
/*��������	    Pcd_Comm    */
/*���ܣ�	    ������ͨ��    */
				
/*���������	Command��ͨ�Ų������
				pInData�������������飻
				InLenByte���������������ֽڳ��ȣ�
				pOutData�������������飻
				pOutLenBit���������ݵ�λ����    */
/*����ֵ��	    OK
				ERROR    */
/*********************************************/  

unsigned char Pcd_Comm(unsigned char Command,
    unsigned char *pInData,
    unsigned char InLenByte,
    unsigned char *pOutData,
    unsigned int *pOutLenBit)
{
    unsigned char result;
    unsigned char rx_temp;//��ʱ�����ֽڳ���
    unsigned char rx_len;//���������ֽڳ���
    unsigned char lastBits;//��������λ����
    unsigned char irq;
        
    rx_temp = 0;
    rx_len = 0;
    lastBits = 0;
    
	Clear_FIFO();
    
   	Write_Reg(CommandReg,Idle);
//    rx_temp = Read_Reg(CommandReg);
    
   	Write_Reg(WaterLevelReg,0x20);//����FIFOLevel=32�ֽ�
//   	rx_temp = Read_Reg(WaterLevelReg);
    
	Write_Reg(ComIrqReg,0x7F);//���IRQ��־
//	rx_temp = Read_Reg(ComIrqReg);

    rx_temp = 0;
    
 	if(Command == MFAuthent)
	{
		Write_FIFO(InLenByte,pInData);//������֤��Կ	
		Set_BitMask(BitFramingReg,0x80);//��������
	}
    Set_BitMask(TModeReg,0x80);//�Զ�������ʱ��

 	Write_Reg(CommandReg,Command);

    //�Լ�������ʱ�жϣ�100ms��ʱ
    gl_fm17522_param.i2c_comm_timeout_cnt = 400;
	while(1)//ѭ���ж��жϱ�ʶ
    {
        if(!gl_fm17522_param.i2c_comm_timeout_cnt)
        {
            //�Լ��жϳ�ʱ�����ش���
            result = ERROR;
            return result;
        }
        
        irq = Read_Reg(ComIrqReg);//��ѯ�жϱ�־		

        if(irq&0x01)	//TimerIRq  ��ʱ��ʱ���þ�
        {
            result=ERROR;		
            break;
        }
        if(Command == MFAuthent)
        {
            if(irq & 0x10)	//IdelIRq  command�Ĵ���Ϊ���У�ָ��������
            {
                result=OK;
                break;
            }
        }
        if(Command == Transmit)	
        {
            if((irq & 0x04) && (InLenByte > 0))	//LoAlertIrq+�����ֽ�������0
            {
                if (InLenByte < 32)
                {
                    Write_FIFO(InLenByte,pInData);	
                    InLenByte=0;
                }
                else
                {
                    Write_FIFO(32,pInData);

                    InLenByte = InLenByte - 32;
                    pInData = pInData + 32;
                }
                Set_BitMask(BitFramingReg,0x80);	//��������

                Write_Reg(ComIrqReg,0x04);	//���LoAlertIrq
            }	

            if((irq & 0x40) && (InLenByte == 0))	//TxIRq
            {
                result=OK;
                break;
            }
        }

        if(Command == Transceive)
        {
            if((irq & 0x04) && (InLenByte > 0))	//LoAlertIrq + �����ֽ�������0
            {	
                if (InLenByte > 32)
                {
                    Write_FIFO(32,pInData);
                    InLenByte = InLenByte - 32;
                    pInData = pInData + 32;
                }
                else
                {
                    Write_FIFO(InLenByte,pInData);
                    InLenByte = 0;
                }
                Set_BitMask(BitFramingReg,0x80);//��������
                Write_Reg(ComIrqReg,0x04);//���LoAlertIrq
            }
            if(irq & 0x08)	//HiAlertIRq
            {
                rx_temp = Read_Reg(FIFOLevelReg);

                #define FM175XX_GET_RX_DATA_LEVEL 32
                if((irq & 0x40) && (InLenByte == 0) && (rx_temp > FM175XX_GET_RX_DATA_LEVEL))//TxIRq	+ �����ͳ���Ϊ0 + FIFO���ȴ���32
                {
                    Read_FIFO(FM175XX_GET_RX_DATA_LEVEL, pOutData + rx_len); //����FIFO����
                    rx_len = rx_len + FM175XX_GET_RX_DATA_LEVEL;
                    Write_Reg(ComIrqReg,0x08);	//��� HiAlertIRq
                }
            }
            if((irq & 0x20) && (InLenByte == 0))	//RxIRq=1
            {
                result = OK;
                break;
            }
        }
    }
//    if (Read_Reg(ErrorReg)&0x0F)	//Mifare��Ƭ��д����ʱ�᷵��4��BIT,��Ҫ���δ�����
//        {
// 			result = ERROR;
//        }
//        else
    {   
        if (Command == Transceive)
        {
            rx_temp = Read_Reg(FIFOLevelReg);
            Read_FIFO(rx_temp, pOutData + rx_len); //����FIFO����
            rx_len= rx_len + rx_temp;//���ճ����ۼ�
            
            //lastBits = Read_Reg(ControlReg) & 0x07;
            lastBits = Read_Reg(ControlReg);
            lastBits &=  0x07;

            if (lastBits)
                *pOutLenBit = (rx_len - 1) * (unsigned int)8 + lastBits;  
            else
                *pOutLenBit = rx_len * (unsigned int)8;   
        }
    }
    Set_BitMask(ControlReg,0x80);           // stop timer now
    Write_Reg(CommandReg,Idle); 
    Clear_BitMask(BitFramingReg,0x80);//�رշ���
    return result;
}


/*********************************************/
/*��������	    Pcd_SetTimer    */
/*���ܣ�	    ���ý�����ʱ    */
/*���������	delaytime����ʱʱ�䣨��λΪ���룩    */
/*����ֵ��	    OK    */
/*********************************************/

 unsigned char Pcd_SetTimer(unsigned long delaytime)//�趨��ʱʱ�䣨ms��
{
	unsigned long TimeReload;
	unsigned int Prescaler;

	Prescaler = 0;
	TimeReload = 0;
	while(Prescaler < 0xfff)
	{
		TimeReload = ((delaytime*(long)13560) - 1) / (Prescaler*2 + 1);
		if( TimeReload < 0xffff)
			break;
		Prescaler++;
	}
	TimeReload = TimeReload&0xFFFF;
	Set_BitMask(TModeReg, Prescaler>>8);
	Write_Reg(TPrescalerReg, Prescaler&0xFF);
	Write_Reg(TReloadMSBReg, TimeReload>>8);
	Write_Reg(TReloadLSBReg,TimeReload&0xFF);
	return OK;
}


/*********************************************/
/*��������	    Pcd_ConfigISOType    */
/*���ܣ�	    ���ò���Э��    */
/*���������	type 0��ISO14443AЭ�飻    */
/*					 1��ISO14443BЭ�飻    */
/*����ֵ��	    OK    */
/*********************************************/
unsigned char Pcd_ConfigISOType(unsigned char type)
{
    if (type == 0)                     //ISO14443_A
    { 
        Set_BitMask(ControlReg, 0x10); //ControlReg 0x0C ����readerģʽ// chenke, FM17522 ��bit4��RFTԤ��       
        Set_BitMask(TxAutoReg, 0x40); //TxASKReg 0x15 ����100%ASK��Ч        
        Write_Reg(TxModeReg, 0x00);  //TxModeReg 0x12 ����TX CRC��Ч��TX FRAMING =TYPE A// chenke, TX CRC��Ч, TXSPEED 106Kbit        
        Write_Reg(RxModeReg, 0x00); //RxModeReg 0x13 ����RX CRC��Ч��RX FRAMING =TYPE A// chenke, RX CRC��Ч, RXSPEED 106Kbit, RxNoErr = 0, RxMultipl = 0, 
    }
    if (type == 1)                     //ISO14443_B
    { 
        Write_Reg(ControlReg, 0x10); //ControlReg 0x0C ����readerģʽ        
        Write_Reg(TxModeReg, 0x83); //TxModeReg 0x12 ����TX CRC��Ч��TX FRAMING =TYPE B        
        Write_Reg(RxModeReg, 0x83); //RxModeReg 0x13 ����RX CRC��Ч��RX FRAMING =TYPE B        
        Write_Reg(GsNReg, 0xF4); //GsNReg 0x27 ����ON�絼        
        Write_Reg(GsNOffReg, 0xF4); //GsNOffReg 0x23 ����OFF�絼        
        Write_Reg(TxAutoReg, 0x00);// TxASKReg 0x15 ����100%ASK��Ч        
    }
    return OK;
}



/*********************************************/
/*��������	    FM175XX_SoftReset    */
/*���ܣ�	    ��λ����    */
				
/*���������	    */
				

/*����ֵ��	    OK
				ERROR    */
/*********************************************/

unsigned char  FM175XX_SoftReset(void)
{	
    unsigned char result;
    Write_Reg(CommandReg,SoftReset);//chenke, ��λFM17522
    result = Set_BitMask(ControlReg,0x10);//17520��ʼֵ����//chenke, FM17522 RFTλ��1
    result = OK;
    return result;
}


//static unsigned char sl_hardreset_onetime = 1;
unsigned char FM175XX_HardReset(void)
{	
    //����NPD��
    rt_pin_write(FM17522_NPD, PIN_LOW);
    FM17522_Delayms(1);
    //����NPD��
    rt_pin_write(FM17522_NPD, PIN_HIGH);
    FM17522_Delayms(10);
    return OK;
}



/*********************************************/
/*��������	    FM175XX_SoftPowerdown    */
/*���ܣ�	    ����͹��Ĳ���    */
/*���������	    */
/*����ֵ��	    OK������͹���ģʽ��    */
/*				ERROR���˳��͹���ģʽ��    */
/*********************************************/
unsigned char FM175XX_SoftPowerdown(void)
{
    unsigned char result;
    result = Read_Reg(CommandReg);
    
	//if (gl_fm_FMSP_param.result&0x10)
	//{
		//Clear_BitMask(CommandReg,0x10);//�˳��͹���ģʽ
    //    CLEAR_BITMASK(CommandReg,0x10);
    //    PT_INIT(pl_pt);
	//	return ERROR;
	//}
	//else
    {   
        Set_BitMask(CommandReg,0x10);//����͹���ģʽ
    }
    return OK;
}


/*********************************************/
/*��������	    FM175XX_HardPowerdown    */
/*���ܣ�	    Ӳ���͹��Ĳ���    */
/*���������	    */
/*����ֵ��	    OK������͹���ģʽ��    */
/*				ERROR���˳��͹���ģʽ��    */
/*********************************************/
//unsigned char FM175XX_HardPowerdown(void)
//{	
//	NPD=~NPD;
//	if(NPD==1)
//		return OK; //����͹���ģʽ
//	else
//		return ERROR;//�˳��͹���ģʽ
//}
/*********************************************/
/*��������	    Read_Ext_Reg    */
/*���ܣ�	    ��ȡ��չ�Ĵ���    */
/*���������	reg_add���Ĵ�����ַ    */
/*����ֵ��	    �Ĵ�����ֵ    */
/*********************************************/
unsigned char Read_Ext_Reg(unsigned char reg_add)
{
    unsigned char result;
 	Write_Reg(0x0F,0x80+reg_add);
    
 	result = Read_Reg(0x0F);
    return result;
}
/*********************************************/
/*��������	    Write_Ext_Reg    */
/*���ܣ�	    д����չ�Ĵ���    */
/*���������	reg_add���Ĵ�����ַ��reg_value���Ĵ�����ֵ    */
/*����ֵ��	    OK
				ERROR    */
/*********************************************/

unsigned char Write_Ext_Reg(unsigned char reg_add, unsigned char reg_value)
{
	Write_Reg(0x0F,0x40+reg_add);
	Write_Reg(0x0F,0xC0+reg_value);
	return OK;
} 

void FM17522_Delayms(unsigned int delayms)
{
    rt_thread_mdelay(delayms);
	return ;
}

