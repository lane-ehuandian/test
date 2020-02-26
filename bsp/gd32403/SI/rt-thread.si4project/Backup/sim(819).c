
#include <math.h>
#include "sim.h"
#include "IO_config.h"
#include "common.h"
#include "fsm.h"
#include "smart_system.h"
#include "datarom.h"
#include "gprs.h"
#ifdef SIM5320
#include "Gprs5320.h"
#endif

#ifdef SIM868
#include "Gprs868.h"
#include "Gps868.h"
#endif

#ifdef CFG_SIM_SLEEP
#include "GprsBike.h"
#endif
#ifdef GPRS_FW_UPGRADE
#include "GprsUpg.h"
#endif
typedef struct{
    rt_device_t usart_dev;
	struct serial_configure usart_config_param;
//	SafeBuf     shell_SafeBuf;
}sim_param_t;
sim_param_t sim_param;

static SimCard __g_SimCard;
SimCard* g_pSimCard = &__g_SimCard;

static AtCmdCtrl __g_SimAtCmdCtrl;
AtCmdCtrl* g_pSimAtCmdCtrl = &__g_SimAtCmdCtrl;
static SwTimer g_SimTimer;
static SwTimer g_SimPowerTimer;

uint8 g_isGprsRun = TRUE;
uint8 g_isGpsRun = TRUE;

#define GPRS_TX_SIZE  128

#define SIM_TIME_ID_POWERON1   0
#define SIM_TIME_ID_POWERON2     1
#define SIM_TIME_ID_RESET     2
#define SIM_TIME_ID_READY     3

#ifdef SIM_NEW
#include "SimNew.c"
#else

void Sim_DoPowerOn(void);

#ifdef CFG_SIM_SLEEP
void Sim_UartWakeUpIsr()
{	
//	if(g_isPowerDown)
//	{
////	   	PostMsg(MSG_SIM_WAKEUP);
//		SetWakeUpType(WAKEUP_SIM);
//	}
}

#endif

//如果PowerReset连续5次，都没有启动成功,则必须执行一次完全关电，
//完全关电必须要Sim_PowerOff持续拉低30秒.
#define POWERE_OFF_DEFAULT 3000
uint32 g_SimPowerOffDelay = POWERE_OFF_DEFAULT;
void Sim_PowerReset(uint8 reason)
{
	//PFL(DL_MAIN, "Sim power reset\n");
	
	if(g_SimPowerOffDelay++ >= (POWERE_OFF_DEFAULT+5))
	{
		g_SimPowerOffDelay = 60000;
	}
	else if(g_SimPowerOffDelay >= 60000)
	{
		g_SimPowerOffDelay = POWERE_OFF_DEFAULT;
	}
	
	Sim_PowerOff();
	
	LOG2(ET_SIM_PWR_ON, reason, g_SimPowerOffDelay/1000);
	
	SwTimer_Start(&g_SimPowerTimer, g_SimPowerOffDelay, SIM_TIME_ID_RESET);
}

int Sim_SyncSendAtCmdEx(const char* pAtCmd, const char* pAck, int waitMs, char** pRsp)
{
	*pRsp = g_pSimAtCmdCtrl->m_Rsp;
	return AtCmdCtrl_SyncSend(g_pSimAtCmdCtrl, pAtCmd, pAck, waitMs);
}

int Sim_SyncSendAtCmd(const char* pAtCmd, const char* pAck, int waitMs)
{
	return AtCmdCtrl_SyncSend(g_pSimAtCmdCtrl, pAtCmd, pAck, waitMs);
}

//int Sim_AnsySendAtCmd(const char* pAtCmd, const char* pAck, int waitMs, uint32 nTag)
//{
//	return AtCmdCtrl_AnsySend(g_pSimAtCmdCtrl, pAtCmd, pAck, waitMs, nTag);
//}

//void Sim_RxByte(uint8 data)
static rt_err_t Sim_RxByte(rt_device_t dev, rt_size_t size)
{
	static uint8_t buff[64];
	static int len = 0;
	unsigned char ch;
	if(rt_device_read(sim_param.usart_dev, -1, &ch, 1) == 1)
	{
		SafeBuf_WriteByte(&g_pSimAtCmdCtrl->m_SafeBuf, ch);
		if(ch=='\n')
	    {
		    if(!g_pSimCard->isCommOk)
		    {
//			    Printf("Sim power on\n");
		    }
		    g_pSimCard->isCommOk = TRUE;	
	    }
	}
    return RT_EOK;
}

static rt_err_t sim_tx_done(rt_device_t dev,void * buffer)
{
//    gl_rs485_param.usart_tx_flag = 0;
    return RT_EOK;
}

Bool Sim_TxAtCmd(const char* pData, int len)
{	
	if(pData[0] == 'A' && pData[1] == 'T')
	{
		PFL(DL_SIM, "%s", pData);
	}
	else
	{
		PFL(DL_SIM, "Data[%d]: %02X %02X %02X ", len, pData[0], pData[1], pData[2]);
		//DUMP_BYTE_EX("SimTx:", pData, len);
	}
	
//	UART_Write(UART_SIM, (uint8_t*)pData, len);
	rt_device_write(sim_param.usart_dev, 0, (uint8_t*)pData, len);
	return True;
}

void Sim_TimerStart(uint32 timeOutMs, uint32 nId)
{
//	Printf("****Sim timer[%d] start %d.\n", nId, timeOutMs);
	SwTimer_Start(&g_SimTimer, timeOutMs, nId);
}

Bool Sim_Filter(const char* pRcv, uint16* len)
{
	if(!AtCmdCtrl_IsLineEnd(pRcv, len)) return False;
	
	if(IS_FOUND(pRcv, "AT+"))
	{
		//说明SIM模组已经被复位
		Gprs_Reset();
		Gps_Reset();
	}

	if(g_pGprs->isSimReady != SIMCARD_READY)
	{
		if(IS_FOUND(pRcv, "RDY\r\n") || IS_FOUND(pRcv, "Ready\r\n"))
		{
//			Printf("Sim is Ready\n");
			//启动READY定时器,如果在改时间内没有连接成功,则
			g_pGprs->isSimReady = SIMCARD_READY;
			Sim_TimerStart(120000, SIM_TIME_ID_READY);
		}
	}
	
	if(g_pGprs->isConnected)
	{
		//Printf("Stop SIM_TIME_ID_READY\n");
		SwTimer_StopEx(&g_SimTimer, SIM_TIME_ID_READY);
	}
	 
	return True;
}

 
void Sim_TimerProc()
{
	if(SwTimer_isTimerOutId(&g_SimTimer, SIM_TIME_ID_POWERON1))
	{
		SIM_PWR_KEY_ON();
		//SIM_PWR_KEY_IO 先拉高1.5秒,再拉低并保持
		Sim_TimerStart(2000, SIM_TIME_ID_POWERON2);
	}
	else if(SwTimer_isTimerOutId(&g_SimTimer, SIM_TIME_ID_POWERON2))
	{
//		CLK_EnableModuleClock(UART1_MODULE);
		// Open smartcard interface 0 in UART mode.
//		NVIC_EnableIRQ(UART1_IRQn);
//	    sim_param.usart_config_param.baud_rate = BAUD_RATE_115200; /* 9600 bits/s */
//        sim_param.usart_config_param.data_bits = DATA_BITS_8;      /* 8 databits */
//        sim_param.usart_config_param.stop_bits = STOP_BITS_1;      /* 1 stopbit */
//        sim_param.usart_config_param.parity = PARITY_NONE;      /* EVEN parity  */
//        sim_param.usart_config_param.bit_order = BIT_ORDER_LSB;    /* LSB first sent */
//        sim_param.usart_config_param.invert = NRZ_NORMAL;       /* Normal mode */
//        sim_param.usart_config_param.bufsz = RT_SERIAL_RB_BUFSZ; /* Buffer size */
//        sim_param.usart_config_param.reserved = 0;
   		sim_param.usart_dev = rt_device_find(SIM_USART_DEV_NAME);
        rt_device_open(sim_param.usart_dev, RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM);
//		rt_device_control(sim_param.usart_dev, RT_DEVICE_CTRL_CONFIG, &sim_param.usart_config_param);
        rt_device_set_tx_complete(sim_param.usart_dev, sim_tx_done);
        rt_device_set_rx_indicate(sim_param.usart_dev, Sim_RxByte);
		
		g_pSimCard->isCommOk = True;

		PFL(DL_MAIN, "Sim power on.\n", _FUNC_);
		//启动READY定时器
		Sim_TimerStart(10000, SIM_TIME_ID_READY);
		SIM_PWR_KEY_OFF();
	}
	else if(SwTimer_isTimerOutId(&g_SimTimer, SIM_TIME_ID_READY))
	{
		Sim_PowerReset(SIM_READY_TIMEOUT);
	}
}

void Sim_DelaySendCmdInd(int cmdInd, uint32 timeOutMs)
{
	AtCmdCtrl_DelaySendCmdInd(g_pSimAtCmdCtrl, cmdInd, timeOutMs);
}

Bool Sim_SendCmdInd(int cmdInd)
{
	return AtCmdCtrl_SendCmdByInd(g_pSimAtCmdCtrl, cmdInd);
}

static uint32 g_SendDataDelay = GPRS_SENDDATA_TIME_LONG;
void Sim_SetTxDataTime(uint32 ms)
{
	g_SendDataDelay = ms;
}
Bool Sim_SendCmdItem(const AtCmdItem* pCmd)
{
	char* buff = (char*)g_CommonBuf;
	uint32 delay = 5000;
	
	if(CMD_CIPSEND == pCmd->cmdInd)
	{
		return Gprs_SendData(pCmd);
	}
	else if(CMD_CIPSENDDATA == pCmd->cmdInd)
	{
		g_SimPowerOffDelay = POWERE_OFF_DEFAULT;
		Sign_TimerReset();
		//DUMP_BYTE_EX(("Tx:"), g_pGprs->pTcpPkt, g_pGprs->tcpPktlen);
		AtCmdCtrl_AnsySendData(g_pSimAtCmdCtrl, g_pGprs->pTcpPkt, g_pGprs->tcpPktlen, pCmd->atAck, g_SendDataDelay, (uint32)pCmd);
		Sim_SetTxDataTime(GPRS_SENDDATA_TIME_LONG);
		return True;
	}

	strcpy(buff, pCmd->atCmd);
	if(CMD_CGNSPWR == pCmd->cmdInd)
	{
		sprintf(buff, pCmd->atCmd, g_pGps->isPowerOnNew);
	}
	else if(CMD_CIPOPEN == pCmd->cmdInd)
	{
		sprintf(buff, pCmd->atCmd, g_pDataRom->m_IpAddr, g_pDataRom->m_IpPort); 
		delay = 20000;
	}
	else if(GPS_CMD_CGPSINFOCFG == pCmd->cmdInd)
	{
		sprintf(buff, pCmd->atCmd, g_pGps->m_TimeOutS); 
	}
	else if(CMD_NETCLOSE == pCmd->cmdInd)
	{
		delay = 20000;
	}
#ifdef CFG_SIM_SLEEP	
	else if(CMD_SIM_CSCLK == pCmd->cmdInd)
	{
		sprintf(buff, pCmd->atCmd, Gprs_GetCsclk()); 
		delay = 1000;
	}
#endif	
	else if(CMD_SMS_TEXT == pCmd->cmdInd)
	{
		delay = 30000;
	}
	else if(g_pSimCard->SendCmdItem)
	{
		g_pSimCard->SendCmdItem(pCmd, buff, &delay);
	}

	return AtCmdCtrl_AnsySend(g_pSimAtCmdCtrl, buff, pCmd->atAck, delay, (uint32)pCmd);
}

AtCmdItem* Sim_GetCmdByInd(int cmdInd)
{
	return AtCmdCtrl_GetCmdInd(g_pSimAtCmdCtrl, cmdInd);
}

Bool Sim_CanGoSleep()
{
	return !AtCmdCtrl_IsBusy(g_pSimAtCmdCtrl);
}

Bool Sim_IsSleep()
{
	return g_pSimCard->isLowPower && g_pSimCard->isSimSleep;
}

Bool Sim_Sleep()
{
	SwTimer_Stop(&g_SimTimer);
//	if(g_pSimCard->isLowPower)
//	{
//		return False;
//	}
	
#ifdef CFG_SIM_SLEEP
	Gprs_SetCsclk(ENABLE_SLOW_CLK);
#endif	
	g_pSimCard->isSimSleep = False;
	g_pSimCard->isSimSleepNew = True;
	g_pSimCard->isLowPower = True;
	
	//PFL(DL_SIM, "%s\n", _FUNC_);
	
	SwTimer_Stop(&g_SimTimer);
	
	AtCmdCtrl_ResetEx(g_pSimAtCmdCtrl);
	//SwTimer_Stop(&g_pGps->m_Timer);
	
	Gprs_Sleep();
	Gps_Sleep();
	Gprs_WdStop();
	//g_pGprs->isConnected = False;
	//g_pGprs->isRcvSms = False;
	//SwTimer_Stop(&g_pGprs->m_HbTimer);	//停止心跳

	//g_pGps->isPowerOn = 1;
	//g_pGps->isPowerOnNew = 0;
	return True;
}

Bool Sim_Wakeup()
{
	if(g_pSimCard->isLowPower == False) 
	{
		return False;
	}
	
#ifdef CFG_SIM_SLEEP
	Gprs_SetCsclk(DISABLE_SLOW_CLK);
#endif	
	//PFL(DL_SIM, "%s\n", _FUNC_);
	
	g_pSimCard->isSimSleepNew = False;
	//g_pGps->isPowerOnNew = True;
	//g_pGps->isPowerOn = False;
//	SIM_WAKEUP();

	g_pSimCard->isLowPower = False;
	
	SetErrorCode(ERR_TYPE_GPRS, 0, ET_NONE);
	SetErrorCode(ERR_TYPE_GPS, 0, ET_NONE);

//	CLK_EnableModuleClock(UART1_MODULE);
//	NVIC_EnableIRQ(UART1_IRQn);
//	SYS->PB_L_MFP &= ~(SYS_PB_L_MFP_PB4_MFP_Msk | SYS_PB_L_MFP_PB5_MFP_Msk );
//	SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB4_MFP_UART1_RX | SYS_PB_L_MFP_PB5_MFP_UART1_TX);
//	UART_Open(UART1, 115200);
//	UART_ENABLE_INT(UART1, UART_IER_RDA_IE_Msk);
//	
	AtCmdCtrl_ResetEx(g_pSimAtCmdCtrl);
//	SwTimer_Stop(&g_pGps->m_Timer);
//	SwTimer_Stop(&g_pGprs->m_HbTimer);
	
	Gprs_Wakeup();
	Gps_Wakeup();
	Gprs_WdStart();
	return True;
}

void Sim_PowerOn()
{
#ifdef CFG_SIM_SLEEP
	Sim_Wakeup();
#else
	//Sim_PowerReset();
	Sim_DoPowerOn();
#endif		
}

void Sim_DoPowerOn()
{
	if(!Sim_Wakeup()) 
	{
		return;
	}
	
	//上电过程，POWER_KEN先拉高500ms
	SIM_PWR_ON();
	SIM_PWR_KEY_OFF();
	Sim_TimerStart(500, SIM_TIME_ID_POWERON1);
	
	g_pSimCard->isSimSleep = False;
	g_pGps->isPowerOnNew = True;
}

void Sim_PowerOff()
{
	if(!Sim_Sleep()) return;
	
	PFL(DL_MAIN, "%s(%d)\n", _FUNC_, g_SimPowerOffDelay);
	Gprs_Reset();
	Gps_Reset();
	
//	CLK_DisableModuleClock(UART1_MODULE);
//	NVIC_DisableIRQ(UART1_IRQn);
//	GPIO_SetMode(SIM_UART_RX_PORT, SIM_UART_RX_PIN, GPIO_PMD_INPUT);
//	GPIO_SetMode(SIM_UART_RX_PORT, BIT5, GPIO_PMD_INPUT);

	SIM_PWR_KEY_OFF();
	SIM_PWR_OFF();

	g_pSimCard->isSimSleep = True;
	g_pGps->isPowerOn = False;
	g_pSimCard->isCommOk = False;
	g_pGprs->isSyncTime = False;
}
void Sim_Run()
{
	
	AtCmdCtrl_Run(g_pSimAtCmdCtrl);
	if(SwTimer_isTimerOut(&g_SimPowerTimer))
	{
		g_pSimCard->isLowPower = True;
		Sim_DoPowerOn();
		goto End;
	}

	Sim_TimerProc();
	Gprs_WdCheck();
	
	if(!g_pSimCard->isCommOk)
	{
		goto End;
	}
	
	if(g_isGprsRun) Gprs_Run();
#ifdef CFG_NO_GPS
#else
	if(g_isGpsRun)  Gps_Run();
#endif	

	if(!AtCmdCtrl_IsIdle(g_pSimAtCmdCtrl))
	{
		goto End;
	}
	
#ifdef CFG_NO_GPS
#else
	if(g_pGps->isPowerOn != g_pGps->isPowerOnNew 
		&& g_pGprs->isSyncTime 
		&& !GprsUpg_IsSwitch())
	{
		//if(!g_pGps->isPowerOnNew || g_pGprs->isConnected)
			Sim_SendCmdInd(CMD_CGNSPWR);
		goto End;
	}
#endif	
	
#ifdef CFG_SIM_SLEEP	
	if(g_pSimCard->isSimSleep != g_pSimCard->isSimSleepNew)
	{
		//如果没有启动完毕，不能进入睡眠
		if(g_pSimCard->isSimSleepNew == False && !g_pGprs->isIndentify)
		{
			g_pSimCard->isSimSleep = g_pSimCard->isSimSleepNew;
		}
		else
		{
			Sim_SendCmdInd(CMD_SIM_CSCLK);
		}
		goto End;
	}
#endif	
End:
	return;
	Fsm_SetActiveFlag(AF_SIM, !AtCmdCtrl_IsIdle(g_pSimAtCmdCtrl));
}

void Sim_Start()
{
	Sim_PowerReset(SIM_START);
}

void Sim_Init(void)
{
	#define AT_OK_STR 		"\r\nOK\r\n"
	#define AT_ERROR_STR 	"\r\nERROR\r\n"
	
	AtCmdCtrl_Init(g_pSimAtCmdCtrl
		, Sim_TxAtCmd
		, Sim_SendCmdItem
		, g_SimReq
		, g_GprsCmdTbl
		, AT_OK_STR, AT_ERROR_STR);

	g_SendDataDelay = GPRS_SENDDATA_TIME_LONG;
	g_pSimAtCmdCtrl->Filter = Sim_Filter;
	Gps_Init();
	Gprs_Init();
	
#ifdef CFG_SIM_SLEEP
//	Mcu_RegGpioIsr(SIM_UART_RX_PORT, SIM_UART_RX_PIN, Sim_UartWakeUpIsr);
#endif
}
#endif	//#ifdef SIM_NONE

