/****************************************************************************//**
 * @file    Gprs.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 2016-5-31 $
 * @brief   BLE operation API implement file .c
 *
 * @note
 * Copyright (C) 2016 immotor Technology Corp. All rights reserved.
*****************************************************************************/

//#ifdef REMOVED_GPRS
//#include "GprsNone.c"
//#else

#include "gprs.h"
#ifdef SIM_NONE
#else
#include "fsm.h"
#include "smart_system.h"
#include "sim.h"
#include "DataRom.h"
#include "Ble.h"
#include "utp.h"
#include "Pms.h"
#include "Sms.h"
#include "Beep.h"
#ifdef SIM5320
#include "Gprs5320.h"
#endif
#ifdef SIM868
#include "Gprs868.h"
#endif
#include "gprstlv.h"
typedef struct _SysCfg
{
	uint8  version; //版本号，必须放在第一个字节,如果与保存的版本号不一致，会导致删除保存的数据而使用默认值。
	uint32 debugLevel;	
	
	uint8  timeIsValid:1;	//resetTime,值是否有效
	uint8  isLogValid:1;	//readLogStartSector，postLogInd是否有效
	uint8  IsUnPostLog:1;
	uint8  reserved1:5;
	
	uint32 resetTime;			//复位时间
	uint8  resetReason;			//复位理由
	
	uint8  readLogStartSector;	//读起始扇区偏移
	uint32 postLogInd;			//上传Log的编号
	uint32 readLogCount;		//读取记录数
	
	uint8  reserved2[8];		//保留
	uint8  SerialNums[3][6];
}SysCfg;
SysCfg	g_SysCfg = {0};
//static Utp g_GprsUtp;
SwTimer g_GprsCmdTimer;
static SwTimer g_RcvRspTimer;
static SwTimer g_GprsWdTimer;
Gprs * g_pGprs = Null;
AtCmdItem* g_pAtCmdItem;
static uint8 g_GprsBuf[256];
#ifdef CFG_LOG
static int    g_PostLogRsp = 0;
static int    g_PostLogInd = 0;
static uint8  g_LogReadSector = 0;
#endif

//#define ERR(_p2) {SetErrorCode(ERR_TYPE_GPRS, _p2, ET_ALARM);}

//int Gprs_FramePkt(uint8 cmd, const void* pData, int dataLen, void* pBuf, int bufLen);
//extern void Fsm_SetMaxGpsCount(int value);

//#ifdef CFG_LOG
//void Gprs_SetLogSec(uint8 sec, int ind)
//{
//	g_PostLogInd = ind;
//	g_LogReadSector = sec;
//}
//#endif

void Gprs_Dump()
{
	Printf("Gprs dump:\n");
	Printf("\t HbTimer(%d,%d,%d)\n", g_pGprs->m_HbTimer.m_Id, g_pGprs->m_HbTimer.m_isStart, g_pGprs->m_HbTimer.m_TimeOutTicks);
	Printf("\t WdTime(%d,%d)\n", g_pGprs->m_HbTimer.m_isStart, g_pGprs->m_HbTimer.m_TimeOutTicks);
	Printf("\t GpsisPowerOn = %d|%d\n"	, g_pGps->isPowerOn, g_pGps->isPowerOnNew);
	Printf("\t isLowPower = %d\n"	, g_pGprs->isLowPower);
	Printf("\t isAuth = %d\n"		, g_pGprs->isAuth);
	Printf("\t isSimReady = %d\n"		, g_pGprs->isSimReady);
	Printf("\t isConnected = %d\n"		, g_pGprs->isConnected);
	Printf("\t isIndentify = %d\n"	, g_pGprs->isIndentify);
	Printf("\t isSimSleep = %d|%d\n"	, g_pSimCard->isSimSleep, g_pSimCard->isSimSleepNew);
	Printf("\t heartBeatInterval = %d\n"	, g_pGprs->heartBeatInterval);
	Printf("\t IsPortChanged = %d\n"	, Gprs_IsPortChanged());
	Printf("\t isReadSms = %d\n"	, g_pGprs->isReadSms);
	Printf("\t isSendGps = %d\n"	, g_pGprs->isSendGps);
	
	
	Printf("\t CSQ = %d\n"			, g_pSimCard->csq);
	Printf("\t ICCID = %s\n"		, g_pSimCard->iccid);
	
}

Bool Gprs_IsPortChanged()
{
	if(Null == g_pGprs->GetPortStatePkt) return False;
	
//	if(g_pPms->m_portMask != g_pGprs->g_ServerPort)
//	{
//		return True;
//	}

	if(g_pGprs->g_ReportedPort != g_pGprs->g_ServerPort) 
	{
		return True;
	}

	return False;
}

//int Gprs_GetPortStateChangedPkt(void* pBuf, int bufLen)
//{
//	static uint8 pkt[64];
//	static int len = 0;
//	Gprs_SetHearbeatInterval(TIME_HEARBEAT_SHORT);
//	if(g_pGprs->GetPortStatePkt && g_pGprs->isConnected)
//	{
//		if(g_pGprs->g_ReportedPort == g_pGprs->g_ServerPort || len == 0)
//		{
//			len = g_pGprs->GetPortStatePkt(pkt, sizeof(pkt));
//			g_pGprs->g_ReportedPort = g_pPms->m_portMask;
//			//Printf("portMask=%d\n", g_pGprs->g_ReportedPort);
//			Gprs_SetHearbeatInterval(TIME_HEARBEAT_SHORT);
//		}
//	}

//	if(pBuf && len)
//	{
//		return Gprs_FramePkt(GPRS_PORT_STATE_CHANGED, pkt, len, pBuf, bufLen);
//	}
//	return 0;
//}

#ifdef CFG_SIM_SLEEP
static uint8 g_SmsCsclk = DISABLE_SLOW_CLK;
void Gprs_SetCsclk(uint8 csclk)
{
//	if(g_SmsCsclk == csclk) return;
	
	g_SmsCsclk = csclk;
	g_pSimCard->isSimSleepNew = (csclk == ENABLE_SLOW_CLK);
	g_pSimCard->isSimSleep = !g_pSimCard->isSimSleepNew;
	
}

Bool Gprs_IsSlowClk() {return g_SmsCsclk == ENABLE_SLOW_CLK;}
uint8 Gprs_GetCsclk() {return g_SmsCsclk;}
#endif

void Gprs_DelaySendCmdInd(int cmdInd, uint32 timeOutS)
{
	AtCmdItem* pCmd = Sim_GetCmdByInd(cmdInd);
	SwTimer_Start(&g_GprsCmdTimer, timeOutS * 1000, (uint32)pCmd);
}

void Gprs_UpdateFlag(uint8 flag)
{
//	g_pGprs->isIndentify = ((flag & GPRS_EVENT_DEVICE_RESET) > 0);
	if(flag & GPRS_EVENT_PORT_CHANGED)
	{
	}
	Gprs_SetHearbeatInterval(TIME_HEARBEAT_SHORT);
}

int Gprs_FramePkt(uint8 cmd, const void* pData, int dataLen, void* pBuf, int bufLen)
{
	int srcInd = 0;
	int i = 0;
	
	UtpFrame* pFrame = (UtpFrame*)g_CommonBuf;

	pFrame->cmd = cmd;
	pFrame->data[i++] = GPRS_PROTOCOL_VER;
	memcpy(&pFrame->data[i], g_Settings.mac, sizeof(g_Settings.mac));
	i += sizeof(g_Settings.mac);

	if(dataLen)
	{
		memcpy(&pFrame->data[i], pData, dataLen);
		i += dataLen;
	}

	pFrame->len = i;

	Utp_FramePkt((uint8*)pFrame, i + UTP_HEAD_LEN, &srcInd, (uint8*)pBuf, &bufLen);

	return bufLen;
}

uint8 Gprs_GetDeviceResetPkt(void* pBuf, int bufLen)
{
	int i = 0;
	VerDesc* pVerDesc = Null;
	uint8 buf[50];
	
//	buf[0] = g_pPms->m_DevicePkt.deviceType;
	memcpy(&buf[1], g_pSimCard->iccid, CCID_LEN);
	i = 1+CCID_LEN;

	pVerDesc = (VerDesc*)&buf[i];
	pVerDesc->m_HwMainVer 	= g_pDataRom->m_HwMainVer;
	pVerDesc->m_HwSubVer  	= g_pDataRom->m_HwSubVer;
	pVerDesc->m_AppMainVer 	= g_pDataRom->m_AppMainVer;
	pVerDesc->m_AppSubVer  	= g_pDataRom->m_AppSubVer;
	pVerDesc->m_AppMinorVer = g_pDataRom->m_AppMinorVer;
	pVerDesc->m_AppBuildeNum  = g_pDataRom->m_AppBuildeNum;
	i += sizeof(VerDesc);
	
	#if 0	//def TEST_SRV
	const static VerDesc g_PmsVerDesc = {1,0,1,0,0,1};
	pVerDesc->m_AppBuildeNum  = 1;//g_pDataRom->m_AppBuildeNum;
	pVerDesc = (VerDesc*)&g_PmsVerDesc;
	#else
	pVerDesc = GetPmsVerDesc();
	#endif
	
	memcpy(&buf[i], pVerDesc, sizeof(VerDesc));
	i += sizeof(VerDesc);
		
	return Gprs_FramePkt(GPRS_RESET, buf, i, pBuf, bufLen);
}
#ifdef HW_BUNKER
//#include "GprsBunker.c"
#elif HW_BIKE_SHARE
#include "GprsBike.c"
#elif HW_BIKE
#include "GprsBike.c"
#elif HW_B3DOCK
#include "GprsDock.c"
#elif HW_B1DOCK
#include "GprsDock.c"
#endif

ATCMD_RC Gprs_RspATE0(AtCmdState state, char* pRsp, AtCmdItem** pNextAtCmd)
{
	//如果连续5次启动失败，认为模组供电不足,关机
	if(g_pGprs->resetCounter++ >= 4 )//&& Mcu_Get18650V() < 350)
	{
		Printf("Low power(%d mV), shut down.\n",g_pGprs->resetCounter);//, Mcu_Get18650V());
		rt_thread_mdelay(500);//UDELAY(500);
//		DISCHARGE_18650_OFF();
	}
	return ATCRC_DONE;
}

ATCMD_RC Gprs_RspCPIN(AtCmdState state, char* pRsp, AtCmdItem** pNextAtCmd)
{
	g_pGprs->isConnected = False;
	if(state == AT_SPECIFIED_OK)
	{
		return IS_FOUND(pRsp, "+CPIN: READY") ? ATCRC_SUCCESS : ATCRC_RESEND;
	}
	return  ATCRC_SIMCARD_INVALID;
}

ATCMD_RC Gprs_RspCSQ(AtCmdState state, char* pRsp, AtCmdItem** pNextAtCmd)
{
	ATCMD_RC nRet = ATCRC_DONE;
	int value = 0;
	
	//启动10定时器, 检测连接状态
	
	if(state == AT_SPECIFIED_OK)
	{
		pRsp = strstrex(pRsp, "+CSQ: ");
		value = strtokenValueByIndex(pRsp, ",", 0);
		g_pSimCard->csq = value;
		if(value <= 0)
		{
			Gprs_DelaySendCmdInd(CMD_CSQ, 5);
			return ATCRC_DONE;
		}
	}
	else
	{
#ifdef CFG_SIM_SLEEP
		if(state == AT_TIMEOUT)
		{
			Sim_SyncSendAtCmd("AT+CSCLK=0", Null, 0);
		}
#endif		
		nRet = ATCRC_RESEND;
	}
	
	if(value && g_pGprs->isConnected)
	{
#ifdef CFG_NO_GPS
		Sim_SendCmdInd(CMD_CIPSEND);
#else
		if(!g_pGps->isLocation)
		{
			Gps_Location(True);	
			//防止GPS和GPRS两个命令重复。
			Gprs_DelaySendCmdInd(CMD_CIPSEND, 3);
		}
		else
		{
			Sim_SendCmdInd(CMD_CIPSEND);
			//*pNextAtCmd = Sim_GetCmdByInd(CMD_CIPSEND);
		}
#endif
	}
	else if(!g_pGprs->isGatt)
	{
		*pNextAtCmd = Sim_GetCmdByInd(CMD_CGATT);
	}

	return nRet;
}

ATCMD_RC Gprs_RspCGATT(AtCmdState state, char* pRsp, AtCmdItem** pNextAtCmd)
{
	ATCMD_RC nRet = ATCRC_RESEND;
	
	if(state == AT_SPECIFIED_OK)
	{
		int cgatt = 0;
		pRsp = strstrex(pRsp, "+CGATT: ");
		cgatt = pRsp[0]-'0';
		g_pGprs->isGatt = (cgatt > 0);
		if(cgatt == 0)
		{
			Gprs_DelaySendCmdInd(CMD_CSQ, 1);
			nRet = ATCRC_DONE;
		}
		else
		{
			nRet = ATCRC_SUCCESS;
			//Printf("GPRS Att Ok.\n");
			#ifdef GPRS_FW_UPGRADE
			if(GprsUpg_IsUpgradeMode())
			{
				* pNextAtCmd = Sim_GetCmdByInd(CMD_SAPBR1);
			}
			#endif
		}
	}
	
	return nRet;
}

ATCMD_RC Gprs_RspCGATT_SET(AtCmdState state, char* pRsp, AtCmdItem** pNextAtCmd)
{
	ATCMD_RC nRet = ATCRC_RESEND;
	
	if(state == AT_SPECIFIED_OK)
	{
		*pNextAtCmd = Sim_GetCmdByInd(CMD_CCID);
	}
	
	return nRet;
}


ATCMD_RC Gprs_RspCIPSEND(AtCmdState state, char* pRsp, AtCmdItem** pNextAtCmd)
{
	ATCMD_RC nRet = ATCRC_DONE;
	
	if(IS_FOUND(pRsp, ">"))
	{
		//必须清除掉所有Pending的命令
		AtCmdCtrl_ResetPendingCmd(g_pSimAtCmdCtrl);
		Sim_SendCmdInd(CMD_CIPSENDDATA);
	}	
	else if(AT_ERROR== state)
	{
		g_pGprs->tcpPktlen = 0;
		*pNextAtCmd = Sim_GetCmdByInd(CMD_IS_CIPOPEN);
	}

	return nRet;
}

ATCMD_RC Gprs_RspCIPSENDDATA(AtCmdState state, char* pRsp, AtCmdItem** pNextAtCmd)
{
	ATCMD_RC nRet = ATCRC_DONE;
	UtpFrame* pFrame = (UtpFrame*)&g_pGprs->pTcpPkt[1];
	
	
	if(state == AT_SPECIFIED_OK)
	{
		if(GPRS_HEART_BEAT == pFrame->cmd)
		{
//			g_pGprs->isSendCabinLock = False;
			Gprs_WdFeed();
			PostMsg(MSG_GPRS_UPDATE);
			SwTimer_ReStart(&g_pGprs->m_HbTimer);
			g_pGprs->m_HbCount++;

			if(g_pGprs->m_HbCount % 10 == 0)	//每发满10次(每次1分钟,大概10分钟)记录一次
			{
				LOG2(ET_GPRS_HEARBEAT, g_pGprs->m_HbCount, g_pGprs->m_GpsCount);
			}

			if(g_pGprs->isSendSms)
			{
				Sim_DelaySendCmdInd(CMD_CMGS, 5000);
			}

		}
		else if(GPRS_GPS_POINT == pFrame->cmd)
		{
			PostMsg(MSG_GPS_UPDATE);
			g_pGprs->isSendGps = False;
			g_pGprs->m_GpsCount++;
		}
		else if(GPRS_ACTIVE_DEVICE == pFrame->cmd)
		{
//			ResetDelay(MCURST_DEV_ACTIVE, 1000);
		}
#ifdef CFG_BEACON
		else if(BEACON_CHANGED == pFrame->cmd)
		{
			PostMsg(MSG_BEACON_UPDATE);
			g_pGprs->isSendBeacon = False;
		}
#endif		
		else
		{
			Gprs_WaitForRsp(3000);
		}
		
	}
	else
	{
		//Printf("Send data failed**************.\n");
		* pNextAtCmd = Sim_GetCmdByInd(CMD_NETCLOSE);
//		LOG2(ET_GPRS_SEND_FAILED, Mcu_GetRound5V(), g_pSimCard->csq);
	}
	
	return nRet;
}


void Gprs_TCP_SendData(const uint8* data, uint16 len)
{
	g_pGprs->pTcpPkt   = data;
	g_pGprs->tcpPktlen = len;
	AtCmdItem* pCmd = Sim_GetCmdByInd(CMD_CIPSEND);
	AtCmdCtrl_AnsySendFmt(g_pSimAtCmdCtrl, pCmd->atAck, 2000, (uint32)pCmd, pCmd->atCmd, len);
}

Bool Gprs_SendData(const AtCmdItem* pCmd)
{
	Bool bRet = False;
	uint16 len=0;
	uint8 pBuf[100];
	
	if(g_pGprs->isIndentify)
		g_pGprs->m_TxInitPacket = 0;
	
	if(!g_pGprs->isIndentify)
	{
		len = Gprs_GetDeviceResetPkt(g_GprsBuf, sizeof(g_GprsBuf));
	}
#ifdef CFG_BEACON
	else if(g_pGprs->isSendBeacon && !g_pGprs->isSendCabinLock)
	{
		int i, ind = 1;
		for(i = 0; i < GET_ELEMENT_COUNT(g_Beacon); i++)
		{
			if(g_Beacon[i].major == 0) break;
			memcpy(&pBuf[ind], &g_Beacon[i], sizeof(Beacon));
			ind += sizeof(Beacon);
		}
		pBuf[0] = i;
		len = Gprs_FramePkt(BEACON_CHANGED, pBuf, ind, g_GprsBuf, 100);
	}
#endif
	else if(g_pGprs->IsSendPortChanged())
	{
//		len = Gprs_GetPortStateChangedPkt(g_GprsBuf, sizeof(g_GprsBuf));
//		DUMP_BYTE(g_GprsBuf, len);
	}
	else if(len = GprsTlv_GetChangedTlv(pBuf, 100), len > 0)
	{
		len = Gprs_FramePkt(GPRS_EVENT_CHANGED, pBuf, len, g_GprsBuf, 100);
		//DUMP_BYTE(g_GprsBuf, len);
	}
	else if(g_pGprs->GetDataPkt)
	{
		len = g_pGprs->GetDataPkt(pCmd, g_GprsBuf, sizeof(g_GprsBuf));
	}

	ASRT(len <= sizeof(g_GprsBuf));

	if(len)
	{
		g_pGprs->pTcpPkt   = g_GprsBuf;
		g_pGprs->tcpPktlen = len;
		//DUMP_BYTE_EX("TX:", g_GprsBuf, len);
		bRet = AtCmdCtrl_AnsySendFmt(g_pSimAtCmdCtrl, pCmd->atAck, 2000, (uint32)pCmd, pCmd->atCmd, len);
	}

	return bRet;
}

#define SPLIT_LINE	//NOT USED

Bool Gprs_VerifyIpPkt(const uint8* pData, int datalen)
{
	//Verify frame len;
	if(datalen < 4 || (pData[2] + 4 != datalen)) goto Error;
	
	//Verify frame head and tail.
	if(pData[0] != UTP_FRAME_HEAD || pData[datalen-1] != UTP_FRAME_TAIL) goto Error;

	return True;

	Error:
	//Printf("Ip pkt error:");
	DUMP_BYTE(pData, datalen);
	return False;
}

//2,0101,\r\n
Bool Gprs_VerifyIpDataPkt(const char* pReq, int reqLen, const uint8** ppData, int* datalen)
{
	char* pData = Null;

	if(sscanf(pReq, "%d,", datalen) == 0 || *datalen < 4)
		return False;

	pData   = strstrex(pReq, ",");
	if(Null == *pData) return False;
	if(memcmp(&pReq[reqLen - 3], ",\r\n", 3) != 0)  return False;	

	* ppData = (uint8*)pData;
	return (*datalen >= (reqLen - (pData - pReq) - 3));
}

//检查req是否我一个完整的req数据包,针对命令+IPDATA,<len>,<data>,\r\n特殊处理，因为该数据包的数据部分有可能包含\r\n.
//+IPDATA,2,0101,\r\n
Bool Gprs_Filter(const char* pRcv, uint16* len)
{
	const uint8* pData = Null;
	int dataLen = 0;
	const char* pKeyWords = "+IPDATA,";
	
	//如果不是\r\n结尾，肯定不是一个完整的请求包。
	if(!Sim_Filter(pRcv, len)) return False;
	
	pKeyWords = strstrex(pRcv, pKeyWords);
	if(pKeyWords)
	{
		g_pGprs->resetCounter = 0;
		//针对命令+IPDATA,<len>,<data>,\r\n特殊处理，因为该数据包的数据部分有可能包含\r\n.
		return Gprs_VerifyIpDataPkt(pKeyWords, *len-(pKeyWords-pRcv), &pData, &dataLen);
	}
	
#ifdef CFG_SIM_SLEEP
	if(IS_FOUND(pRcv, "+CMGL: "))
	{
		AtCmdItem* pItem = Null;
		uint16 procLen = Sms_FilterCMGL(pRcv, *len);
		if(procLen && (*len) >= procLen)
		{
			*len -= procLen; //移除该段数据
			//Printf("procLen = %d, *len=%d\n", procLen, *len);
			memset((char*)(&pRcv[*len]), 0, procLen);
		}
		
		pItem = Sim_GetCmdByInd(GPS_CMD_CMGL);
		return (IS_FOUND(pRcv, pItem->atAck) != Null);
	}
#endif
#ifdef CFG_GSV
	//$GPGSV,3,1,10,20,78,331,45,01,59,235,47,22,41,069,,13,32,252,45*70 
	extern void Gsv_Parse(const char* pMsg);
	pKeyWords = strstrex(pRcv, "$GPGSV,");
	if(pKeyWords)
	{
		Gsv_Parse(pRcv);
		*len = 0;
	}
	else if(    IS_FOUND(pRcv, "$GNGGA")
		|| IS_FOUND(pRcv, "$GPGSA")
		|| IS_FOUND(pRcv, "$GLGSA")
//		|| IS_FOUND(pRcv, "$GPGSV")
		|| IS_FOUND(pRcv, "$GLGSV")
		|| IS_FOUND(pRcv, "$GNRMC")
		|| IS_FOUND(pRcv, "$GNVTG")
		)
	{
		*len = 0;
	}
#endif
	
	return True;
}

void Gprs_UtpRspProc(uint8 req, OP_CODE op, const uint8* data, int len)
{
	if(g_pGprs->rspProc && g_pGprs->rspProc(req, op, data, len))
	{
		return;
	}
	
	if(GPRS_RESET == req)
	{
		//Printf("GPRS_RESET\n");
		g_pGprs->isIndentify = True;
		g_pGprs->isSyncTime = True;
		g_pGprs->isSendSms = True;
		if(len >= 6)
		{
			S_RTC_TIME_DATA_T localDt = {0};
			localDt.u32Year   = data[0] + 2000;
			localDt.u32Month  = data[1];
			localDt.u32Day    = data[2];
			
			localDt.u32Hour   = data[3];
			localDt.u32Minute = data[4];
			localDt.u32Second = data[5];
			
			DateTime_dump(&localDt);
//			LocalTimeSync(&localDt);
			g_pGprs->m_syncSeconds = DateTime_GetSeconds(&localDt);

			Sign_TimerReset();
		}
		
		if(OP_NOT_ALLOW_DISCHARGE == op && !IsForbidDischarge())
		{
			SetForbidDischarge(True);
//			Nvds_Write_Setting();
		}
	}
	
	if(GPRS_PORT_STATE_CHANGED == req)
	{
		g_pGprs->g_ServerPort = g_pGprs->g_ReportedPort;
		if(op == SUCCESS && len >= 2)
		{
			Pms_SetBatVerify(data[0], data[1]);
		}
	}	
	if(GPRS_EVENT_CHANGED == req)
	{
		GprsTlv_UpdateSentTlv();
	}	
	#ifdef CFG_LOG
	else if(GPRS_POST_LOG == req)
	{
		if(++g_PostLogRsp == 0) g_PostLogRsp++;	//防止溢出归0
		if(op == 0)
		{
			g_SysCfg.isLogValid = 1;
			g_SysCfg.postLogInd = g_PostLogInd;
			g_SysCfg.readLogStartSector = g_LogReadSector;
			g_SysCfg.readLogCount = (g_SysCfg.readLogCount > MAX_POST_LOG_COUNT) ? (g_SysCfg.readLogCount - MAX_POST_LOG_COUNT) : 0;
//			Nvds_Write_SysCfg();
			PFL(DL_LOG, "Log Sent: Sec=%d, ind=%d, Remain=%d\n"
				, g_SysCfg.readLogStartSector, g_SysCfg.postLogInd, g_SysCfg.readLogCount);
		}
	}
	#endif
	
	//启动一个心跳定时器，定时查询电池状态
	//SwTimer_ReStart(&g_pPms->m_Timer);
}

Bool Gprs_IsAllowResend(AtCmdCtrl* pAtCmdCtrl, const AtCmdItem* pAtCmd, int reTxCount)
{
	//Printf("errCounter=%d, reTxCount=%d/%d\n", pAtCmdCtrl->m_ErrCounter, reTxCount,pAtCmdCtrl->m_MaxTxCounter);
	if(!AtCmdCtrl_IsAllowResend(pAtCmdCtrl, pAtCmd, reTxCount))
	{
		Sim_PowerReset(SIM_ATCMD_ERR);
		return False;
	}
	return True;
}

void Gprs_CmdProc(const UtpFrame* pFrame)
{
	//static Bool g_isReq = False;
	static uint8 rspRaw[32];
	//UtpFrame* pRspFrame = (UtpFrame*)rspRaw;

	//PFL(DL_GPRS, "Gprs Cmd[%d]:", pFrame->len + 2);
	//DUMP_BYTE_LEVEL(DL_GPRS, pFrame, pFrame->len + 2);
	Bool isChanged = False;
	Bool isTrue = False;
	int len = 0;
	uint8* pRsp = rspRaw;
	const uint8* pReq = pFrame->data;

	if(pFrame->cmd >= GPRS_RESET && pFrame->cmd < GPRS_REQ_MAX)
	{
		Gprs_UtpRspProc(pFrame->cmd, (OP_CODE)pFrame->data[0], &pReq[1], pFrame->len - 1);
		SwTimer_Stop(&g_RcvRspTimer);
	}
	else if(GPRS_FORBID_DISCHARGE == pFrame->cmd)
	{
		isTrue = pReq[0] & BIT_0;
		isChanged = (g_Settings.IsForbidDischarge != isTrue);
		SetForbidDischarge(isTrue);
		
		isTrue = (pReq[0] & BIT_1)>0;
		isChanged += (g_Settings.IsAlarmMode != isTrue);
		Fsm_GoAlarmMode(isTrue);

		//g_Settings.BatCutOffMode = ((pReq[0] & BIT_2) > 0);
		g_pPms->m_PowerOffAtOnce = ((pReq[0] & BIT_2) > 0);
		Pms_SendCmdDischarge();
		isChanged = True;
		
		if(isChanged)
		{
//			Nvds_Write_Setting();
		}

		isTrue = 0;
		pRsp[len++] = 0;
	}
	else if(GPRS_ACTIVE_DEVICE == pFrame->cmd)
	{
		Bool isChanged = False;
		isTrue = ((pReq[0] & BIT_0) > 0);
		if(IsActive() != isTrue)
		{
			SetActive(isTrue);
			isChanged = True;
		}

		isTrue = ((pReq[0] & BIT_1) > 0);
		if(IsSignEn() != isTrue)
		{
			SetSignEn(isTrue);
			isChanged = True;
		}

		if(isChanged)
		{
			Sign_TimerReset();
//			Nvds_Write_Setting();
		}	

		pRsp[len++] = 0;
		
	}
	else if(GPRS_DEV_RESET == pFrame->cmd && pFrame->len >= 1)
	{
		if(pReq[0] == 0)	//MCU Reset
		{
//			BootWithReason(MCURST_GPRS);
		}
		else
		{
			Sim_PowerReset(SIM_GPRS_DEV_RESET);
		}
	}
	else if(GPRS_SET_DEV_STATE == pFrame->cmd && pFrame->len >= 1)	//设置电池身份校验功能使能
	{
		if(pReq[0] < 3)
		{
			isChanged = 0;
			if(g_Settings.IsBatVerifyEn != pReq[0])
			{
				g_Settings.IsBatVerifyEn = pReq[0];
//				Nvds_Write_Setting();
			}
		}
		else
		{
			isChanged = 1;
		}
		pRsp[len++] = isChanged;
	}
	else if(GPRS_BAT_VERIFY == pFrame->cmd)
	{
		if(pFrame->len >= 2)
		{
			Pms_SetBatVerify(pReq[0], pReq[1]);
		}
		pRsp[len++] = (pFrame->len >= 2) ? 0 : 1;
	}
#ifdef CFG_LOG
	else if(GPRS_LOG_GET_INFO == pFrame->cmd)
	{
		LogItem logItem = {0};
		len = 0;
		pRsp[len++] = 0;
		memcpy(&pRsp[len], &g_LogRecord.total, 4);
		len += 4;
		if(Log_Read(&logItem, 1, 0))
		{
			memcpy(&pRsp[5], &logItem.dateTime, 4);
			len += 4;
		}
	}
	else if(GPRS_LOG_READ == pFrame->cmd)
	{
		DUMP_BYTE(pReq, pFrame->len);
		uint32 seconds = AS_UINT32(pReq[3], pReq[2], pReq[1], pReq[0]) + SECOND_OFFSET;
		g_SysCfg.postLogInd = Log_FindByTime(seconds);
		int count = AS_UINT32(pReq[7], pReq[6], pReq[5], pReq[4]);
		Log_SetPostInd(g_SysCfg.postLogInd, count);

		PFL(DL_LOG, "LogRead[%s]:Sec=%d, ind=%d, count=%d\n"
			, DateTime_ToStr(seconds), g_SysCfg.readLogStartSector, g_SysCfg.postLogInd, g_SysCfg.readLogCount);
		pRsp[len++] = SUCCESS;
	}
#endif
#ifdef CFG_WHEEL_LOCK
	else if(GPRS_SET_LOCK == pFrame->cmd)
	{
		Pms_SetLock(pReq[0]);
		pRsp[len++] = SUCCESS;
	}
#endif
#ifdef CFG_CABIN_LOCK
	else if(GPRS_SET_CABIN_LOCK == pFrame->cmd)
	{
		Pms_SetCabinLock(pReq[0]);
		pRsp[len++] = SUCCESS;
	}
#endif
#ifdef CFG_REMOTE_ACC
	else if(GPRS_SET_ACC == pFrame->cmd)
	{
		Pms_SetAcc(pReq[0]);
		pRsp[len++] = SUCCESS;
	}
#endif
#ifdef CFG_BEACON
	else if(GPRS_SET_BEACON == pFrame->cmd)
	{
		Ble_BeaconScan(pReq[0] ? 150 : 0);
		pRsp[len++] = SUCCESS;
	}
#endif		
	if(len)
	{
		len = Gprs_FramePkt(pFrame->cmd, pRsp, len, g_GprsBuf, sizeof(g_GprsBuf));
		//DUMP_BYTE(g_GprsBuf, len);
		Gprs_TCP_SendData(g_GprsBuf, len);
	}
}


//2,0101,\r\n
void Gprs_CmdIpData(const char* pReq, int reqLen)
{
	int i = 0;
	const uint8* pData = Null;
	int dataLen = 0;
	
	FRAME_STATE state = FRAME_INIT;
	static uint8 buf[UTP_REMOTE_REQ_BUF_SIZE];
	UtpFrame* pFrame = (UtpFrame*)&buf[1];
	uint16 ind = 0;

	if(!Gprs_VerifyIpDataPkt(pReq, reqLen, &pData, &dataLen) || !Gprs_VerifyIpPkt(pData, dataLen))
	{
		//Printf("IpData error[%s].\n", pReq);
		return ;
	}

	for(i = 0; i < dataLen; i++)
	{		
		if(Utp_ConvertToHost(buf, sizeof(buf), &ind, &state, pData[i])
			&& pFrame->len + 4 == ind)	//校验帧长度是否有效
		{
			Gprs_CmdProc(pFrame);
		}
	}
}

Bool Gprs_TimerProc()
{

	if(SwTimer_isTimerOut(&g_GprsCmdTimer))
	{
		return Sim_SendCmdItem((AtCmdItem*)g_GprsCmdTimer.m_Id);
	}
	else if(SwTimer_isTimerOut(&g_pGprs->m_HbTimer))
	{
		return Sim_SendCmdInd(CMD_CSQ);
	}
	
	//自动启动定时器。
	if(g_pGprs->isConnected && !g_pGprs->m_HbTimer.m_isStart && !g_pSimCard->isSimSleep)
	{
		SwTimer_Start(&g_pGprs->m_HbTimer, g_pGprs->heartBeatInterval, TIMERID_HEARBEAT);	
	}

	return False;
}

//改变心跳时间
void Gprs_SetHearbeatInterval(uint32 ms)
{
	SwTimer* pTimer = &g_pGprs->m_HbTimer;
	if(ms == g_pGprs->heartBeatInterval && pTimer->m_TimeOutTicks == ms && pTimer->m_isStart)
	{
		return ;
	}

	g_pGprs->heartBeatInterval = ms;
	
	if(g_pGprs->isConnected)
	{
		PFL(DL_GPRS, "HeartbeatTime=%d S.\n", ms/1000);
		SwTimer_Start(pTimer, g_pGprs->heartBeatInterval, TIMERID_HEARBEAT);	
	}
}

void Gprs_WaitForRsp(uint32 ms)
{
	SwTimer_Start(&g_RcvRspTimer, ms, 0);
}
Bool Gprs_IsWaitRsp()
{
	if(g_RcvRspTimer.m_isStart)
	{
		SwTimer_isTimerOut(&g_RcvRspTimer);
		return True;
	}

	return False;
}

//Gprs watch dog opertion
void Gprs_WdStart()
{
	SwTimer_Start(&g_GprsWdTimer, 300000, 0);
}
void Gprs_WdStop()
{
	SwTimer_Stop(&g_GprsWdTimer);
}
void Gprs_WdFeed()
{
	SwTimer_ReStart(&g_GprsWdTimer);
}
void Gprs_WdCheck()
{
	if(SwTimer_isTimerOut(&g_GprsWdTimer))
	{
		Printf("Gprs Wd timeout!\n");
		Sim_PowerReset(SIM_GPRS_WDT);
		return;
	}
}

void Gprs_Run()
{	
	#define SEND_LOG_INTERAL 2000
	Gprs_WdCheck();
	
	#ifdef SIM868
	if(!g_pGprs->m_isStart)
	{
		Gprs_Start();
		g_pGprs->m_isStart = True;
	}
	#endif
	
	if(!AtCmdCtrl_IsIdle(g_pSimAtCmdCtrl)) return ;

	//等待响应
	if(Gprs_IsWaitRsp())
	{
		return;
	}
	
	if(g_pGprs->isConnected)
	{
		if(g_pGprs->IsSendPortChanged() 
			|| !g_pGprs->isIndentify 
//			|| g_pGprs->isSendCabinLock
			|| GprsTlv_GetChangedTlv(g_GprsBuf, sizeof(g_GprsBuf)))
		{
			Gprs_SetHearbeatInterval(TIME_HEARBEAT_SHORT);
		}
		
#ifdef CFG_LOG
		else if(g_SysCfg.readLogCount && g_pGprs->heartBeatInterval != SEND_LOG_INTERAL)	
		{
			if(Log_GetRemain(g_SysCfg.readLogStartSector, g_SysCfg.postLogInd) >= MAX_POST_LOG_COUNT)
			{
				Gprs_SetHearbeatInterval(SEND_LOG_INTERAL);
				Fsm_SetMaxGpsCount(2);	//延迟休眠
			}
			else
			{
				Fsm_SetMaxGpsCount(1); //取消延迟休眠
			}
		}
#endif		
#ifdef CFG_BEACON
		if(g_pGprs->isSendBeacon)
		{
			Sim_SendCmdInd(CMD_CIPSEND);
			return;
		}
#endif		
		if(g_pGprs->isSendGps)
		{
			Sim_SendCmdInd(CMD_CIPSEND);
			return;
		}
		else if(g_pGprs->isReadSms)
		{
			Sim_SendCmdInd(GPS_CMD_CMGL);
			return;
		}
	}

	Gprs_TimerProc();
}

void Gprs_Reset()
{
	Gprs_Sleep();
	SwTimer_Stop(&g_pGprs->m_HbTimer);
	g_pGprs->tcpPktlen = 0;
	g_pGprs->isConnected = False;
	g_pGprs->isSimReady = SIMCARD_UNKNOWN;
	g_pGprs->isIndentify = False;
	g_pGprs->isGatt = 0;

	g_pGprs->isAuth = 0;
	g_pGprs->isLowPower = 0;
	g_pSimCard->csq = 0;
	g_pGprs->isReg = False;

	Gprs_SetHearbeatInterval(TIME_HEARBEAT_SHORT);
#ifdef SIM868
	g_pGprs->m_isStart = False;
#endif
	g_pGprs->isReadSms = False;
	g_pGprs->isSendGps = False;
}

//#ifdef SIM5320
//#include "Gprs5320.c"
//#endif

//#ifdef SIM868
////#include "Gprs868.c"
//#endif

void Gprs_Wakeup(void)
{
	g_pGprs->m_HbCount = 0;
	g_pGprs->m_GpsCount = 0;
	SwTimer_Stop(&g_pGprs->m_HbTimer);
	Gprs_WdStart();
}

void Gprs_Sleep(void)
{
	g_pGprs->isConnected = False;
	g_pGprs->isRcvSms = False;
	g_pGprs->isReadSms = False;
	SwTimer_Stop(&g_pGprs->m_HbTimer);	//停止心跳
	SwTimer_Stop(&g_GprsCmdTimer);
	Gprs_WdStop();
}

void Gprs_Start(void)
{
	PFL(DL_SIM, "%s\n", _FUNC_);
	AtCmdCtrl_Reset(g_pSimAtCmdCtrl);
	Sim_DelaySendCmdInd(CMD_ATE, 1000);
	Gps_Start();
	Gprs_WdStart();
}


void Gprs_InitBase()
{
	static Gprs __g_Gprs;

	g_pGprs = &__g_Gprs;
	memset(g_pGprs, 0, sizeof(Gprs));

	g_pGprs->heartBeatInterval = TIME_HEARBEAT_SHORT;

	SwTimer_Init(&g_pGprs->m_HbTimer, 0, 0);
	g_pGprs->IsSendPortChanged = Gprs_IsPortChanged;
	g_pSimAtCmdCtrl->Filter	= Gprs_Filter;
	g_pSimAtCmdCtrl->IsAllowResend = Gprs_IsAllowResend;

	GprsTlv_Init();
}

#endif	//#ifdef SIM_NONE
//#endif	//#ifdef REMOVED_GPRS
