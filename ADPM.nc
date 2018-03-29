/* NeW ADP-MAC Schdule */
/*Authors: Shama Siddiqui, Dr. Sayeed Ghani, Anwar Ahmed Khan*/
/*
 *
 * This module implements the basic ADP
 */

module ADPM
{
  provides {
    interface StdControl;
    interface ADPControl;
    interface MacMsg;
    interface MacActivity;
  }
  uses {
    interface StdControl as PhyControl;
    interface StdControl as TimerControl;
    interface RadioState;
    interface CarrierSense;
    interface CsThreshold;
    interface PhyPkt;
    interface PhyNotify;
    interface PhyStreamByte;
    interface Random;
	interface GetSetU32 as LocalTime;
	
	interface Timer as Sleeping_Timer;
    interface Timer as Waking_Timer;
	interface Timer as POLL_Timer;
	interface Timer as Wait_RX_Timer;
    interface Timer as BACK_OFF_Timer;
	interface Timer as Wait_EA_Timer;
    interface Timer as Wait_DATA_Timer;
    interface Timer as Wait_NEA_Timer;
    interface Timer as Wait_ACK_Timer;
    interface Timer as Wait_Timer;
  }
}

implementation
{
#include "StdReturn.h"
#include "ADPConst.h"

// MAC constants, used internally

// additional delay before timeout when waiting for a packet
#define TIMEOUT_DELAY 2

//#define TIME_WAIT_ACK (ADP_PROCESSING_DELAY + ADP_ACK_DURATION + ADP_PROCESSING_DELAY + TIMEOUT_DELAY)

  /* MAC states
  *-------------
  * POLL
  * POLLING_SENSE -- Carrier Sensing i-e POLLING
  * S_SLEEP -- SlPolling Interval
  * SLEEP -- Deep Sleep
  * WAIT_EA - just sent Rpre, and is waiting for EA
  * WAIT_DATA - just sent EA, and is waiting for DATA
  * WAIT_ACK - just sent DATA, and is waiting for ACK
  */
  enum {
    POLL,
	POLLING_SENSE,
	S_SLEEP,
	SLEEP,
	WAIT_EA,
	WAIT_DATA,
	WAIT_ACK,
  };

  // ADP packet types -- will move to ADPMsg.h
  enum { BCAST_DATA_PKT, DATA_PKT, SDATA_PKT, Rpre_PKT, EA_PKT, ACK_PKT };

  // state variables
  uint8_t state;  // MAC state
  
  //FLAGS
  uint8_t ICFLAG;
  uint8_t gotosleep;
  uint8_t TxPending;
  uint8_t forward;
  
  // Variables
  
  uint32_t TWake;
  uint32_t TSLEep;
  uint8_t Idle_Cycle;
  uint32_t tmp;
  uint8_t Idle_Poll;
  uint8_t BACK_OFF;
  uint8_t ContWinSize;
  uint32_t tTime;
  uint32_t atTime;
  uint32_t tmpTime;
  uint16_t RpreRetry;
  uint8_t No_Pkt;
  uint16_t OS;
  uint8_t NP;
  uint8_t WTrunning;
  uint16_t DDelay;
  uint32_t ORcvTime;
  uint32_t OSum;
  uint32_t OSum2;
  uint32_t OMean;
  uint32_t NSum;
  uint32_t NSum2;
  uint32_t NMean;
  uint32_t CV;
  uint32_t X;
  uint32_t CountPkt;
  uint32_t UpdatePolling;
  uint32_t TargetTime;
  uint32_t TargetPkt;
 // uint32_t W[DynWin]={0};
  //uint32_t W[5]={0,0,0,0,0};c
 // uint32_t aw;
  uint32_t LTime;
 
  
  uint8_t  ExPPolling;
  uint8_t  i;
  uint32_t POLLING_INTER;
  uint16_t PollNo;		//Max elements in Array 
  uint16_t PollArray[200]={48,363,76,72,114,89,186,375,34,67,135,90,182,144,28,18,75,127,28,32,278,1,329,26,3,90,119,51,204,174,80,115,76,315,375,247,31,22,165,9,11,55,39,18,29,56,93,322,307,122,31,121,106,60,168,54,321,23,103,143,59,48,122,76,54,14,181,74,65,18,10,665,157,137,45,5,22,50,113,91,55,106,94,613,40,8,1,148,14,36,245,313,14,5,4,211,31,3,102,69,118,210,21,30,53,56,137,280,84,61,44,86,89,302,187,25,195,10,54,12,298,78,223,11,5,21,113,133,303,12,2,103,45,362,150,33,8,58,18,8,141,347,33,311,60,292,11,66,33,201,137,140,24,11,76,52,85,76,188,125,53,22,7,54,13,126,85,76,69,66,2,14,208,152,76,214,162,76,29,37,234,98,85,142,52,10,66,10,17,137,7,143,65,257,150,78,13,3,94,11};
  
 
  
  
  
  
  
  
  
  uint16_t sendAddr;   // node that I'm sending data to
  uint32_t PN;   // Packet Number  = Seq No.
  uint8_t txPktLen;   // length of data pkt to be transmitted
  uint16_t addPreambleLen;  // additional preamble to be added
  
  
  ADPHeader* dataPkt; // pointer to tx data pkt, only access MAC header
  AppHeader* APPH;
  AppPkt*  APPPKT;
  AppPkt PACKET;
  
  AppPkt*  srcPKT;
  AppPkt*  dstPKT;
  SAppPkt*  dstSPKT;
  
  AppPkt Buffer0;
  AppPkt Buffer1;
  AppPkt Buffer2;
  AppPkt Buffer3;
  AppPkt Buffer4;
  
  SAppPkt Buffer;
  SAppPkt* SAPPPKT;
  
  uint8_t* srcPtr;
  uint8_t* dstPtr;
  uint8_t BufFul[5]={0,0,0,0,0};
  uint8_t SendTry[5]={0,0,0,0,0};
  uint16_t SendRTSTry[5]={0,0,0,0,0};
  uint8_t SPLL[5]={0,1,2,3,4};		// Super Packet Location
  
  uint8_t SUM;
  uint8_t CCR;
  uint8_t CNP;
  uint8_t inc;
  uint8_t PayLd;
  uint8_t EASUM;
  uint8_t SPS;
  uint8_t SAP;
  
  uint8_t ctrlPkt[ADP_CTRL_PKT_BUF]; // buffer for control pkts
  
  // Variables for Rx
  uint16_t recvAddr;     // node that I'm receiving data from
  
  // function prototypes
  void handleRpre(void* rpacket);
  void handleEA(void* epacket);
  void* handleDATA(void* dpacket);
  //void handleDATA(void* dpacket);
  void handleACK(void* apacket);
 // void handleErrPkt();
 
  void writefuncSS( void* src, void* dest, uint8_t offset);
  
  
  void Send_Rpre();
  void Send_EA();
  void Send_DATA();
  void Send_ACK();
  void SaveDATA(uint32_t* add, uint8_t size);
  uint16_t updcrc(void* src, uint8_t length);

  
  
  
  
  
  
  command result_t StdControl.init()
  {
    // initialize ADP and related components
    CountPkt=0;
	UpdatePolling=0;
	TargetTime=DynIntTime;
	TargetPkt=DynIntPkt;
	
	SUM=0;
	ORcvTime=0;
	ExPPolling = ExpPolling;
    state = SLEEP;
    ContWinSize = ADP_CONT_WIN;
	TWake=TWAKE;
	TSLEep=TSLEEP;
	DDelay=250;
	PollNo=0;
/*	for(i=0; i<=99; i++)
		{
		if(POLLING_INTERVAL==10)
			PollArray[i] = PollArray10[i];
		else if(POLLING_INTERVAL==20)
			PollArray[i] = PollArray20[i];
		else if(POLLING_INTERVAL==30)
			PollArray[i] = PollArray30[i];
		else if(POLLING_INTERVAL==40)
			PollArray[i] = PollArray40[i];
	//	if(POLLING_INTERVAL==50)
	//		PollArray[i] = PollArray50[i];
	//	if(POLLING_INTERVAL==60)
	//		PollArray[i] = PollArray60[i];
	//	if(POLLING_INTERVAL==70)
	//		PollArray[i] = PollArray70[i];
		}*/
		
			
	POLLING_INTER=POLLING_INTERVAL;
    // initialized lower-level components
    call Random.init();  // initialize random number generator
    call TimerControl.init();  // initialize timer
    call PhyControl.init();  //initialize physical layer
    
    return SUCCESS;
  }


command result_t StdControl.start()
  {
    // start MAC and lower-level components
    
    call TimerControl.start();  // initialize timer
    call PhyControl.start();  //initialize physical layer
	call RadioState.sleep();
	call Sleeping_Timer.start(TIMER_ONE_SHOT, 1);
	WTrunning=0;

    return SUCCESS;
  }
  
  
command result_t StdControl.stop()
  {
    // stop ADP
    
    call PhyControl.stop();  // stop physical layer
    return SUCCESS;
  }

event result_t Sleeping_Timer.fired()	//Sleeping Timer
  {
    state = POLL;
	call RadioState.idle();
	state=POLLING_SENSE;
	call CarrierSense.start(POLLING_DURATION);
	call Waking_Timer.start(TIMER_ONE_SHOT, TWake);
	WTrunning=1;
	ICFLAG=1;
	gotosleep=0;
	
	
	//////////// updated on 16-Apr-17
	
	LTime = call LocalTime.get();
	if(DynOpt !=0)
		{
		if(DynOpt==2 || DynOpt==1)
			{
			if (LTime >= TargetTime)
				{
				TargetTime=TargetTime+DynTime;
				UpdatePolling=1;
				if(TargetPkt==DynIntPkt)
					CountPkt=0;
				else
					CountPkt=TargetPkt-DynPkt;
				}
			}
		if(DynOpt==2 || DynOpt==3)
			{
			if (CountPkt >= TargetPkt)
				{
				TargetPkt=TargetPkt+DynPkt;
				UpdatePolling=1;
				if(TargetTime==DynIntTime)
					TargetTime = LTime + DynIntTime;
				else
					TargetTime = LTime + DynTime;
				}
			}
		}

	if(UpdatePolling ==1)
		{
		ORcvTime=0;
		OSum=0;
		OSum2=0;
		OMean=0;
		
		if(CV >=8)		// its basically 0.8
			ExPPolling=1;
		else
			{
			ExPPolling=0;
			POLLING_INTER=POLLING_INTERVAL;
			}
		UpdatePolling =0;
		}
	
	if(ExPPolling==1)
		{
		PollNo++;
		if(PollNo>=((sizeof(PollArray))/4))
			PollNo=0;
		POLLING_INTER=PollArray[PollNo];
		if(POLLING_INTER==0)
			{
			PollNo++;
			POLLING_INTER=PollArray[PollNo];
			}	
		}
	////////////// updated on 16-Apr-17
	return SUCCESS;
	}

event result_t Waking_Timer.fired()	//WAKING Timer
  {
    if((state == POLL) || (state == S_SLEEP))
		{
		if(ICFLAG==1)
			{
			Idle_Cycle++;
			}
		if(Idle_Cycle < (IC_LIMIT -1))
			{
			call POLL_Timer.stop();
			state=SLEEP;
			call RadioState.sleep();
			call Sleeping_Timer.start(TIMER_ONE_SHOT, TSLEep);
			WTrunning=0;
			}
		else if(Idle_Cycle ==(IC_LIMIT -1))
			{
			TWake= LongTWAKE;
			call POLL_Timer.stop();
			state=SLEEP;
			call RadioState.sleep();
			call Sleeping_Timer.start(TIMER_ONE_SHOT, TSLEep);
			WTrunning=0;
			}
		else if(Idle_Cycle >= IC_LIMIT)
			{
			Idle_Cycle =0;
			TWake=Original_TWAKE;
			TSLEep=2*TSLEEP;
			call POLL_Timer.stop();
			state=SLEEP;
			call RadioState.sleep();
			call Sleeping_Timer.start(TIMER_ONE_SHOT, TSLEep);
			WTrunning=0;
			}
		}
	else if((state == POLLING_SENSE) || (state == WAIT_EA) || (state == WAIT_ACK) || (state == WAIT_DATA))
		{
		gotosleep=1;
		WTrunning=0;
		}
		
///////////////////////////////////////////////////////////////		
	if(ExPPolling==1)
		{
		PollNo++;
		if(PollNo>=((sizeof(PollArray))/4))
			PollNo=0;
		POLLING_INTER=PollArray[PollNo];
		if(POLLING_INTER==0)
			{
			PollNo++;
			POLLING_INTER=PollArray[PollNo];
			}	
		}
		
///////////////////////////////////////  added on 8-Apr-17 ///////////////		
		
		
	return SUCCESS;
	}//END-Waking_Timer	
event result_t POLL_Timer.fired()		//POLL_Interval Timer
  {
    call RadioState.idle();
	if(WTrunning==1)
		tmp = (call Waking_Timer.getRemainingTime());
	else			// No-case
		tmp=2;
	
	LTime = call LocalTime.get();
	if(DynOpt !=0)
		{
		if(DynOpt==2 || DynOpt==1)
			{
			if (LTime >= TargetTime)
				{
				TargetTime=TargetTime+DynTime;
				UpdatePolling=1;
				if(TargetPkt==DynIntPkt)
					CountPkt=0;
				else
					CountPkt=TargetPkt-DynPkt;
				}
			}
		if(DynOpt==2 || DynOpt==3)
			{
			if (CountPkt >= TargetPkt)
				{
				TargetPkt=TargetPkt+DynPkt;
				UpdatePolling=1;
				if(TargetTime==DynIntTime)
					TargetTime = LTime + DynIntTime;
				else
					TargetTime = LTime + DynTime;
				}
			}
		}

	if(UpdatePolling ==1)
		{
		ORcvTime=0;
		OSum=0;
		OSum2=0;
		OMean=0;
		
		if(CV >=8)		// its basically 0.8
			ExPPolling=1;
		else
			{
			ExPPolling=0;
			POLLING_INTER=POLLING_INTERVAL;
			}
		UpdatePolling =0;
		}
	
	if(ExPPolling==1)
		{
		PollNo++;
		if(PollNo>=((sizeof(PollArray))/4))
			PollNo=0;
		POLLING_INTER=PollArray[PollNo];
		if(POLLING_INTER==0)
			{
			PollNo++;
			POLLING_INTER=PollArray[PollNo];
			}	
		}
	state=POLLING_SENSE;

	if(tmp < POLLING_DURATION)
		{
		call Waking_Timer.setRemainingTime(POLLING_DURATION + EXTRA);
		}
	call CarrierSense.start(POLLING_DURATION);
	return SUCCESS;
	}
event result_t Wait_RX_Timer.fired()		//Wait for RX Timer
  {
    call Waking_Timer.start(TIMER_ONE_SHOT, tmpTime);
	WTrunning=1;
	call POLL_Timer.start(TIMER_ONE_SHOT, POLLING_INTER);
	state=S_SLEEP;
	call RadioState.sleep();
	return SUCCESS;
	}
event result_t Wait_EA_Timer.fired()		//Wait for EA Timer
  {
    if(gotosleep==1)
		{
		state=SLEEP;
		call RadioState.sleep();
		BACK_OFF=(call Random.rand() % ContWinSize);
		call Sleeping_Timer.start(TIMER_ONE_SHOT, (TSLEep + BACK_OFF));
		call Waking_Timer.stop();
		WTrunning=0;
		}
	else
		{
		Send_Rpre();
		}
	return SUCCESS;
	}
event result_t Wait_DATA_Timer.fired()		//Wait for Data Timer
  {
    call Waking_Timer.start(TIMER_ONE_SHOT, tmpTime);
	WTrunning=1;
	call POLL_Timer.start(TIMER_ONE_SHOT, POLLING_INTER);
	state=S_SLEEP;
	call RadioState.sleep();
	return SUCCESS;
	}
	
event result_t Wait_NEA_Timer.fired()		//Wait for Neighbour_EA
  {
    if(TxPending!=1)
		{
		state=SLEEP;
		call RadioState.sleep();
		call Sleeping_Timer.start(TIMER_ONE_SHOT, TSLEep);
		call Waking_Timer.stop();
		WTrunning=0;
		}
	else
		{
		Send_Rpre();
		}
	
	return SUCCESS;
	}
	
event result_t Wait_ACK_Timer.fired()		//Wait for ACK
  {
	call Waking_Timer.start(TIMER_ONE_SHOT, tmpTime);
	WTrunning=1;
	call POLL_Timer.start(TIMER_ONE_SHOT, POLLING_INTER);
	state=S_SLEEP;
	call RadioState.sleep();
	return SUCCESS;
	}
	
event result_t BACK_OFF_Timer.fired()		//Back-off Timer
  {
    call Waking_Timer.start(TIMER_ONE_SHOT, tmpTime);
	WTrunning=1;
	call CarrierSense.start(POLLING_DURATION);
	
	return SUCCESS;
	}//END-BACK_OFF
event result_t Wait_Timer.fired()		//Wait for Waiting :)
  {
    state=SLEEP;
	call RadioState.sleep();
	call Sleeping_Timer.start(TIMER_ONE_SHOT, TSLEep);
	call Waking_Timer.stop();
	WTrunning=0;
	
	return SUCCESS;
	}
	
async event result_t CarrierSense.channelIdle()		// physical carrier sense indicate channel idle
  {
	if(state==POLLING_SENSE)
		{
		Idle_Poll++;
		if((TxPending==0) && (gotosleep==0))
			{
			state=S_SLEEP;
			call RadioState.sleep();
			call POLL_Timer.start(TIMER_ONE_SHOT, POLLING_INTER);
			}
		else if(TxPending==1)
			{
			Send_Rpre();
			}
		else if(gotosleep==1)
			{
			//state=SLEEP;
			if(ICFLAG==1)
				{
				Idle_Cycle++;
				}
			if(Idle_Cycle < (IC_LIMIT -1))
				{
				call POLL_Timer.stop();
				state=SLEEP;
				call RadioState.sleep();
				call Sleeping_Timer.start(TIMER_ONE_SHOT, TSLEep);
				WTrunning=0;
				}
			else if(Idle_Cycle ==(IC_LIMIT -1))
				{
				TWake=LongTWAKE;
				call POLL_Timer.stop();
				state=SLEEP;
				call RadioState.sleep();
				call Sleeping_Timer.start(TIMER_ONE_SHOT, TSLEep);
				WTrunning=0;
				}
			else if(Idle_Cycle >= IC_LIMIT)
				{
				Idle_Cycle =0;
				TWake=Original_TWAKE;
				TSLEep=TSLEEP + TSLEEP;
				call POLL_Timer.stop();
				state=SLEEP;
				call RadioState.sleep();
				call Sleeping_Timer.start(TIMER_ONE_SHOT, TSLEep);
				WTrunning=0;
				}
			}
		}
	return SUCCESS;
	}

async event result_t CarrierSense.channelBusy()// physical carrier sense indicate channel busy
  {
	if(state == POLLING_SENSE)
		{
		if(WTrunning==1)
			{
			tmpTime= call Waking_Timer.getRemainingTime();
			call Waking_Timer.stop();
			WTrunning=0;
			}
		ICFLAG=0;
		TSLEep=Original_TSLEEP;
		Idle_Cycle =0;
		if(gotosleep==1)
			{
			call Wait_Timer.start(TIMER_ONE_SHOT, POLLING_DURATION);
			}
		else if(gotosleep!=1)
			{
			if(TxPending!=1)
				{
				call Wait_RX_Timer.start(TIMER_ONE_SHOT, POLLING_DURATION);
				}
			else if(TxPending==1)
				{
				BACK_OFF=(call Random.rand() % ContWinSize);
				call BACK_OFF_Timer.start(TIMER_ONE_SHOT, (POLLING_DURATION + BACK_OFF));
				}
			}
		}
    return SUCCESS;
  }
 

event void PhyStreamByte.rxDone(uint8_t* buffer, uint8_t byteIdx)// PHY streams each byte before the whole packet is received
  {   

#ifndef ADP_ENABLE_OVERHEARING    // overhearing avoidance after receiving headers of unicast pkt
    uint8_t tmpe;
    ADPHeader* recvHdr;
	
	//call POLL_Timer.stop();
	//call Wait_EA_Timer.stop();
	//call Wait_DATA_Timer.stop();
	//call Wait_NEA_Timer.stop();
	//call Wait_ACK_Timer.stop();
	call Wait_RX_Timer.stop();
	call BACK_OFF_Timer.stop();
	call Wait_Timer.stop();
	if(WTrunning==1)
		{
		tmpTime= call Waking_Timer.getRemainingTime();
		call Waking_Timer.stop();
		WTrunning=0;
		}
	
	
	
    if(byteIdx == sizeof(ADPHeader) - 1) {      // the whole header is received
		recvHdr = (ADPHeader*)buffer;
		tmpe = (*(buffer + sizeof(PhyHeader))) >> 4;  // pkt type
		
		//call Wait_Timer.stop();
		//call Wait_RX_Timer.stop();
		//call BACK_OFF_Timer.stop();
		//if(WTrunning==1)
		//	{
		//	tmpTime= call Waking_Timer.getRemainingTime();
		//	call Waking_Timer.stop();
		//	WTrunning=0;
		//	}
		if (((tmpe == DATA_PKT) || (tmpe == SDATA_PKT)) && (recvHdr->toAddr != TOS_LOCAL_ADDRESS)) // unicast destined to others
			{
			tmpe = recvHdr->phyHdr.length;  // pkt length
			tTime = ((((uint32_t)(tmpe - sizeof(ADPHeader) + (PHY_BASE_PRE_BYTES) + sizeof(ACK)) + SIFS) * PHY_TX_BYTE_TIME )/ 1000) + EXTRA;
				
			if(state==POLLING_SENSE)
				{
				state=SLEEP;
				call Waking_Timer.stop();
				call RadioState.sleep();
				call Sleeping_Timer.start(TIMER_ONE_SHOT, tTime);
				WTrunning=0;
				}
			else if(state==WAIT_EA)
				{
				call Wait_EA_Timer.stop();
				call Waking_Timer.stop();
				WTrunning=0;
				state=SLEEP;
				call RadioState.sleep();
				call Sleeping_Timer.start(TIMER_ONE_SHOT, tTime);
				}
			else if(state==WAIT_DATA)
				{
				call Wait_DATA_Timer.stop();
				call Waking_Timer.stop();
				WTrunning=0;
				state=SLEEP;
				call RadioState.sleep();
				call Sleeping_Timer.start(TIMER_ONE_SHOT, tTime);
				}
			else if(state==WAIT_ACK)
				{
				call Wait_ACK_Timer.stop();
				call Waking_Timer.stop();
				WTrunning=0;
				state=SLEEP;
				call RadioState.sleep();
				call Sleeping_Timer.start(TIMER_ONE_SHOT, tTime);
				}
		}
	}
#endif
  }



event void* PhyPkt.receiveDone(void* vpacket, uint8_t error)
  {
    uint8_t vpktType;
    //if(vpacket == NULL || (error==1)) 		return vpacket;	
		

	
	if(0) 	return vpacket;
	else
		{
		vpktType = (*((uint8_t*)vpacket + sizeof(PhyHeader))) >> 4;     // dispatch to actual packet handlers
		
		if((vpktType == Rpre_PKT) || (vpktType == EA_PKT) || (vpktType == DATA_PKT) || (vpktType == SDATA_PKT) || (vpktType == ACK_PKT))
			{
			call Wait_Timer.stop();
			call Wait_RX_Timer.stop();
			call BACK_OFF_Timer.stop();
		//	if(WTrunning==1)
		//	{
		//	tmpTime= call Waking_Timer.getRemainingTime();
		//	call Waking_Timer.stop();
		//	WTrunning=0;
		//	}
			
			//if(vpktType == Rpre_PKT) call ADPControl.handleRpre(vpacket);
			//else if(vpktType == EA_PKT) call ADPControl.handleEA(vpacket);
			////else if(vpktType == DATA_PKT) return call ADPControl.handleDATA(vpacket);
			//else if(vpktType == DATA_PKT) call ADPControl.handleDATA(vpacket);
			//else if(vpktType == ACK_PKT) call ADPControl.handleACK(vpacket);
			
			if(vpktType == Rpre_PKT) handleRpre(vpacket);
			else if(vpktType == EA_PKT)	handleEA(vpacket);
			else if(vpktType == DATA_PKT) return handleDATA(vpacket);
			else if(vpktType == ACK_PKT) handleACK(vpacket);
			else if(vpktType == SDATA_PKT)	return handleDATA(vpacket);
			
			error=0;
			
			}
			//else
			//	handleErrPkt();
		return vpacket;
		}
  }
  
void handleRpre(void* pkt)    // internal handler for RTS
//command ADPControl.handleRpre(void* pkt)    // internal handler for RTS
  {
	Rpre* rpacket;
    rpacket = (Rpre*)pkt;
	No_Pkt=rpacket->No_Pkt;
	recvAddr=rpacket->fromAddr;
	
    if(state==POLLING_SENSE)
		{
		if(rpacket->toAddr != TOS_LOCAL_ADDRESS)
			{
			if(TxPending!=1)
				{
				call Wait_NEA_Timer.start(TIMER_ONE_SHOT, POLLING_DURATION);
				}
			else if(TxPending==1)
				{
				call Wait_NEA_Timer.start(TIMER_ONE_SHOT, POLLING_DURATION);
				}
			}
		else if(rpacket->toAddr == TOS_LOCAL_ADDRESS)
			{
			if(TxPending!=1)
				{
				Send_EA();
				}
			else if(TxPending==1)
				{
				TxPending=0;
				Send_EA();
				}
			}
		}
    else if(state==WAIT_EA)// do-nothing 
		{
		if(rpacket->toAddr != TOS_LOCAL_ADDRESS) // do-nothing or No-case
			{
			if(TxPending!=1){}//No-case
			else if(TxPending==1){}
			}
		else if(rpacket->toAddr == TOS_LOCAL_ADDRESS) // do-nothing or No-case
			{
			if(TxPending!=1){}//No-case
			else if(TxPending==1){}
			}
		}
    else if(state==WAIT_DATA)
		{
		if(rpacket->toAddr != TOS_LOCAL_ADDRESS) // do-nothing or No-case
			{
			if(TxPending!=1){}
			else if(TxPending==1){}
			}
		else if(rpacket->toAddr == TOS_LOCAL_ADDRESS)
			{
			call Wait_DATA_Timer.stop();
			if(TxPending!=1)
				{
				Send_EA();
				}
			else if(TxPending==1) // isnt it NO-Case?
				{
				TxPending=0;
				Send_EA();
				}
			}
		}
    else if(state==WAIT_ACK) // do-nothing or No-case
		{
		if(rpacket->toAddr != TOS_LOCAL_ADDRESS) // do-nothing or No-case
			{
			if(TxPending!=1){}//No-case
			else if(TxPending==1){}
			}
		else if(rpacket->toAddr == TOS_LOCAL_ADDRESS) // do-nothing or No-case
			{
			if(TxPending!=1){}//No-case
			else if(TxPending==1){}
			}
		}
  }  //END-handleRpre;

void handleEA(void* pkt)    // internal handler for CTS
//command ADPControl.handleEA(void* pkt)    // internal handler for CTS
  {
    EA* epacket;
    epacket = (EA*)pkt;
	No_Pkt=epacket->No_Pkt;
	tmp=50 + ( (No_Pkt-1) * 32);
	
    if(state==POLLING_SENSE)
		{
		if(epacket->toAddr != TOS_LOCAL_ADDRESS)
			{
			if ((TxPending!=1) || (TxPending==1))
				{
				tTime = ((((uint32_t)((PHY_BASE_PRE_BYTES) + tmp + (2 * SIFS) + (PHY_BASE_PRE_BYTES) + sizeof(ACK))) * PHY_TX_BYTE_TIME )/ 1000) + EXTRA;
				state=SLEEP;
				call RadioState.sleep();
				call Sleeping_Timer.start(TIMER_ONE_SHOT, tTime);
				WTrunning=0;
				}
			}
		else if(epacket->toAddr == TOS_LOCAL_ADDRESS) //do nothing
			{
			if(TxPending!=1){}
			else if(TxPending==1){}
			}
		}
    else if(state==WAIT_EA)
		{
		if(epacket->toAddr != TOS_LOCAL_ADDRESS)
			{
			call Waking_Timer.stop();
			WTrunning=0;
			call Wait_EA_Timer.stop();
			tTime = ((((uint32_t)((PHY_BASE_PRE_BYTES) + tmp + SIFS + (PHY_BASE_PRE_BYTES) + sizeof(ACK))) * PHY_TX_BYTE_TIME )/ 1000) + EXTRA;
			state=SLEEP;
			call RadioState.sleep();
			call Sleeping_Timer.start(TIMER_ONE_SHOT, tTime);
			}
		else if(epacket->toAddr == TOS_LOCAL_ADDRESS)
			{
			call Wait_EA_Timer.stop();
			if(WTrunning==1)
				{
				tmpTime= call Waking_Timer.getRemainingTime();
				call Waking_Timer.stop();
				WTrunning=0;
				}
			EASUM=No_Pkt;
			Send_DATA();
			}
		}    
    else if(state==WAIT_DATA) //do nothing
		{
		if(epacket->toAddr != TOS_LOCAL_ADDRESS)
			{
			if(TxPending!=1){}
			else if(TxPending==1){}
			}
		else if(epacket->toAddr == TOS_LOCAL_ADDRESS)
			{
			if(TxPending!=1){}
			else if(TxPending==1){}
			}
		}
    else if(state==WAIT_ACK) //do nothing
		{
		if(epacket->toAddr != TOS_LOCAL_ADDRESS)
			{
			if(TxPending!=1){}
			else if(TxPending==1){}
			}
		else if(epacket->toAddr == TOS_LOCAL_ADDRESS)
			{
			if(TxPending!=1){}
			else if(TxPending==1){}
			}
		}
	
		
		
  }//END-handleEA;
 void* handleDATA(void* pkt)	// internal handler for unicast data packet
 //void handleDATA(void* pkt)	// internal handler for unicast data packet
//command ADPControl.handleDATA(void* pkt)	// internal handler for unicast data packet
  {
	uint8_t n_pkts;
	uint8_t leng;
	//uint8_t inc;
	
	ADPHeader* dpacket = (ADPHeader*)pkt;
	PhyHeader* PhyH = (PhyHeader*)pkt;
	
	SAPPPKT = (SAppPkt*)pkt;
	APPPKT = (AppPkt*)pkt;
	APPH = (AppHeader*)pkt;
	
	
	PN=APPH->seqNo;
	OS = APPH->Ori_Source;
	
    if(state==POLLING_SENSE)
		{
		if(dpacket->toAddr != TOS_LOCAL_ADDRESS){} //handled in Phy-stream Byte
		else if(dpacket->toAddr == TOS_LOCAL_ADDRESS){state=WAIT_DATA;}
		}
    else if(state==WAIT_EA)
		{
		call Wait_EA_Timer.stop();
		if(WTrunning==1)
			{
			tmpTime= call Waking_Timer.getRemainingTime();
			call Waking_Timer.stop();
			WTrunning=0;
			}
		if(dpacket->toAddr != TOS_LOCAL_ADDRESS){} //handled in Phy-stream Byte
		else if(dpacket->toAddr == TOS_LOCAL_ADDRESS){} //No-case
		}    
    if(state==WAIT_DATA)
		{
		if(dpacket->toAddr != TOS_LOCAL_ADDRESS){} //handled in Phy-stream Byte
		else if(dpacket->toAddr == TOS_LOCAL_ADDRESS)
			{
			call Wait_DATA_Timer.stop();
			
			CountPkt++;
			X=APPH->TxTime;
			X = X - ORcvTime;
			if(ORcvTime==0)
				{
				OSum=X;
				OSum2=X*X;
				OMean=X;
				
				NSum=X;
				NSum2=X*X;
				NMean=X;
				}
			else if(ORcvTime!=0)
				{
				//if(CountPkt<=DynWin)
				//	{
					NSum = OSum + X;
					NSum2= OSum2 + (X * X);
					NMean = (OMean * NSum) / (OSum + OMean);
				//	}
				//else
				//	{
				//	NSum = OSum + X - W[1];
				//	NSum2 = OSum2 + (X*X) - (W[1] * W[1]);
				//	NMean = NSum / DynWin;
				//	}
				}
			CV = (sqrt( (NSum2 - (NSum * NMean))/(NMean * (NSum - NMean)) ))*10;
			OSum = NSum;
			OSum2 = NSum2;
			OMean = NMean;
			
			//for(aw=1;aw<=(DynWin - 1);aw++)
			//	{
			//	W[aw]=W[aw+1];
			//	}
			//W[DynWin]=X;
			//
			
			ORcvTime = APPH->TxTime;
			
			APPH->TxTime = call LocalTime.get();
			
			if(Con==1)
				{
				leng=PhyH->length;
				n_pkts=(leng-50)/32 +1;
				SAP=n_pkts<<5;
				inc=0;
				
				if(BufFul[0]==0)
					{
					writefuncSS((SAppPkt*)pkt, &Buffer0, inc);
					CCR=updcrc(&Buffer0,(sizeof(AppPkt)-sizeof(SuperAppHeader)-6));	////////////////////////////change Buffer name to 1,2,3,4
					Buffer0.crcc=CCR;
					if((Buffer0.crcc)==CCR)
						{
						BufFul[0]=1;
						SendTry[0]=0;
						SendRTSTry[0]=0;
						SAP=SAP+1;
						}
					inc=inc+1;
					}
				if((BufFul[1]==0) && (inc < n_pkts))
					{
					writefuncSS((SAppPkt*)pkt, &Buffer1, inc);
					CCR=updcrc(&Buffer1,(sizeof(AppPkt)-sizeof(SuperAppHeader)-6));	////////////////////////////change Buffer name to 1,2,3,4
					Buffer1.crcc=CCR;
					if((Buffer1.crcc)==CCR)
						{
						BufFul[1]=1;
						SendTry[1]=0;
						SendRTSTry[1]=0;
						SAP=SAP+2;
						}
					inc=inc+1;
					}
				if((BufFul[2]==0) && (inc < n_pkts))
					{
					writefuncSS((SAppPkt*)pkt, &Buffer2, inc);
					CCR=updcrc(&Buffer2,(sizeof(AppPkt)-sizeof(SuperAppHeader)-6));	////////////////////////////change Buffer name to 1,2,3,4
					Buffer2.crcc=CCR;
					if((Buffer2.crcc)==CCR)
						{
						BufFul[2]=1;
						SendTry[2]=0;
						SendRTSTry[2]=0;
						SAP=SAP+4;
						}
					inc=inc+1;
					}
				if((BufFul[3]==0) && (inc < n_pkts))
					{
					writefuncSS((SAppPkt*)pkt, &Buffer3, inc);
					CCR=updcrc(&Buffer3,(sizeof(AppPkt)-sizeof(SuperAppHeader)-6));	////////////////////////////change Buffer name to 1,2,3,4
					Buffer3.crcc=CCR;
					if((Buffer3.crcc)==CCR)
						{
						BufFul[3]=1;
						SendTry[3]=0;
						SendRTSTry[3]=0;
						SAP=SAP+8;
						}
					inc=inc+1;
					}
				if((BufFul[4]==0) && (inc < n_pkts))
					{
					writefuncSS((SAppPkt*)pkt, &Buffer4, inc);
					CCR=updcrc(&Buffer4,(sizeof(AppPkt)-sizeof(SuperAppHeader)-6));	////////////////////////////change Buffer name to 1,2,3,4
					Buffer4.crcc=CCR;
					if((Buffer4.crcc)==CCR)
						{
						BufFul[4]=1;
						SendTry[4]=0;
						SendRTSTry[4]=0;
						SAP=SAP+16;
						}
					inc=inc+1;
					}
		

				}
			Send_ACK();
			}
		}
    else if(state==WAIT_ACK)
		{
		if(dpacket->toAddr != TOS_LOCAL_ADDRESS){} //handled in Phy-stream Byte
		else if(dpacket->toAddr == TOS_LOCAL_ADDRESS){} //No-case
		}
    return pkt;


  }  //END-handleDATA


void handleACK(void* pkt)    // internal handler for ACK packet
    { 
	uint8_t NNP;
	ACK* apacket;
    apacket = (ACK*)pkt;
	NNP=apacket->NP;
    
	if(state==POLLING_SENSE)
		{
		if(apacket->toAddr != TOS_LOCAL_ADDRESS)
			{
			if(TxPending!=1)
				{
				call Wait_RX_Timer.start(TIMER_ONE_SHOT, 1);
				}
			else if(TxPending==1)
				{
				BACK_OFF=((call Random.rand() % ContWinSize) +SIFS);
				call BACK_OFF_Timer.start(TIMER_ONE_SHOT, BACK_OFF);
				}
			}
		else if(apacket->toAddr == TOS_LOCAL_ADDRESS) {}
		}
    else if(state==WAIT_EA) //do nothing
		{
		if(apacket->toAddr != TOS_LOCAL_ADDRESS) {}
		else if(apacket->toAddr == TOS_LOCAL_ADDRESS) {}
		}    
    else if(state==WAIT_DATA) //do nothing
		{
		if(apacket->toAddr != TOS_LOCAL_ADDRESS){}
		else if(apacket->toAddr == TOS_LOCAL_ADDRESS){}
		}
    else if(state==WAIT_ACK)
		{
		if(apacket->toAddr != TOS_LOCAL_ADDRESS) // do nothing
			{
			if(TxPending!=1){}
			else if(TxPending==1){}
			}
		else if(apacket->toAddr == TOS_LOCAL_ADDRESS)
			{
			call Wait_ACK_Timer.stop();
			if(Con==0)
				{
				TxPending=0;
				if(forward==0)
					signal MacMsg.sendDone(APPPKT, SUCCESS);
				forward=0;
				state=S_SLEEP;
				call RadioState.sleep();
				call POLL_Timer.start(TIMER_ONE_SHOT, POLLING_INTER);
				call Waking_Timer.start(TIMER_ONE_SHOT, tmpTime);
				WTrunning=1;
				}
			if(Con==1)
				{
				SAP=(NNP>>5) &&7;
				if((SAP>5) || (SAP==0)) //NotOK{sendRTS}
					{
					state=POLLING_SENSE;
					Send_Rpre();
					}
				else  // maybe OK , so start checking
					{
					if((NNP && 1) > 0)	// means 1st locaton is ok
						{
						BufFul[SPLL[0]]=0;
						}
					else
						{
						if(SendTry[SPLL[0]]>PRtryLim)
							{
							BufFul[SPLL[0]]=0;
							SendTry[SPLL[0]]=0;
							SendRTSTry[SPLL[0]]=0;
							}
						}
					if(((NNP && 2) > 0) && (SAP >=2))	// means 2nd locaton is ok
						{
						BufFul[SPLL[1]]=0;
						}
					else
						{
						if(SendTry[SPLL[1]]>PRtryLim)
							{
							BufFul[SPLL[1]]=0;
							SendTry[SPLL[1]]=0;
							SendRTSTry[SPLL[1]]=0;
							}
						}
					if(((NNP && 4) > 0) && (SAP >=3))	// means 3rd locaton is ok
						{
						BufFul[SPLL[2]]=0;
						}
					else
						{
						if(SendTry[SPLL[2]]>PRtryLim)
							{
							BufFul[SPLL[2]]=0;
							SendTry[SPLL[2]]=0;
							SendRTSTry[SPLL[2]]=0;
							}
						}
					if(((NNP && 8) > 0) && (SAP >=4))	// means 4th locaton is ok
						{
						BufFul[SPLL[3]]=0;
						}
					else
						{
						if(SendTry[SPLL[3]]>PRtryLim)
							{
							BufFul[SPLL[3]]=0;
							SendTry[SPLL[3]]=0;
							SendRTSTry[SPLL[3]]=0;
							}
						}
					if(((NNP && 16) > 0) && (SAP ==5))	// means 5th locaton is ok
						{
						BufFul[SPLL[4]]=0;
						}
					else
						{
						if(SendTry[SPLL[4]]>PRtryLim)
							{
							BufFul[SPLL[4]]=0;
							SendTry[SPLL[4]]=0;
							SendRTSTry[SPLL[4]]=0;
							}
						}
					
					
					SUM = BufFul[0] + BufFul[1] + BufFul[2] + BufFul[3] + BufFul[4];
					if(SUM==0)	// OK{S_sleep}
						{
						TxPending=0;
						if(forward==0)
							signal MacMsg.sendDone(SAPPPKT, SUCCESS);
						forward=0;
						state=S_SLEEP;
						call RadioState.sleep();
						call POLL_Timer.start(TIMER_ONE_SHOT, POLLING_INTER);
						call Waking_Timer.start(TIMER_ONE_SHOT, tmpTime);
						WTrunning=1;
						}
						else
							{
							state=POLLING_SENSE;
							Send_Rpre();
							}
				} // END // maybe OK ,  checking end
				
				}// Con=1 //End
			} // apacket->toAddr == TOS_LOCAL_ADDRESS
		}// state==WAIT_ACK

}//END-handleACK



void SaveDATA(uint32_t* add, uint8_t size)
{

uint8_t a;
uint8_t da;
uint32_t ada;


for(a=0;a<=size;a++)
	{
	ada=(uint32_t)(&(PACKET)) + a;
	da=*(add+a);
	ada=da;
	}


}

void writefunc( void* src, void* dest, uint8_t size) 					// App -> Buffer0,1,2,3,4
{
uint8_t a;

AppPkt *srcApp=(AppPkt*)src;

char *srcPKTch = (char*)srcApp;
char *dstPKTch = (char*)dest;

for(a=0;a<=size;a++)
	{
	*(dstPKTch+a)=*(srcPKTch+a);
	}
	dest=(AppPkt*)dstPKTch;
}
void writefuncS(void* src, void* dest, uint8_t offset, uint8_t size)	// Buffer0,1,2,3,4 -> SuperPkt
{

uint8_t a;
uint8_t x;
uint8_t y;

char *srcPKTch = (char*)src;
char *dstSPKTch = (char*)dest;


x=sizeof(SuperAppHeader);
y=offset*size;

for(a=x;a<=(x+size);a++)
	{
	*(dstSPKTch+a+y)=*(srcPKTch+a);
	}
	dest=(SAppPkt*)dstSPKTch;
}


void writefuncSS( void* src, void* dest, uint8_t offset) 				// SuperPkt -> Buffer0,1,2,3,4
{

uint8_t a;
uint8_t x;
uint8_t y;

uint8_t z;

SAppPkt *srcApp=(SAppPkt*)src;


char *srcSPKTch = (char*)srcApp;
char *dstPKTch = (char*)dest;

x=sizeof(SuperAppHeader);
z=sizeof(AppPkt)-x-4;		// SeqNo. + GenTime + pay load + CRC == 32
y=offset*z;


for(a=x;a<=(x+z);a++)
	{
	*(dstPKTch+a)=*(srcSPKTch+a+y);
	}
	dest=(AppPkt*)dstPKTch;
}



uint16_t updcrc(void* src, uint8_t length)
{
uint16_t CalCRC;
uint8_t a;
uint8_t ii;
uint16_t tmpi;
uint16_t crc;
uint8_t x;

char *srcPKTch = (char*)src;

x=sizeof(SuperAppHeader);

///////////////////////
crc=0;
for(a=x;a<=(x+length);a++)
	{
	tmpi =(uint16_t)*(srcPKTch+a);
	crc = crc ^ (tmpi << 8);
    for (ii = 0; ii < 8; ii++)
		{
        if (crc & 0x8000)
            crc = crc << 1 ^ 0x1021;  // << is done before ^
        else
            crc = crc << 1;
        }
	}
	
////////////////////////////

CalCRC = crc;

CalCRC=0xEEEE;		// CRC 
return CalCRC;
}



  
command result_t MacMsg.send(void* msg, uint8_t length, uint16_t toAddr, uint8_t forwardd)    // send a message
  {
	
	sendAddr = toAddr;
	txPktLen = length;
	Idle_Cycle=0;
	TWake= Original_TWAKE;
	TSLEep=Original_TSLEEP;
	
	if(Con==0)	
	{ // sanity check
    if(msg == 0 || length == 0 ) return FAIL;//|| length > PHY_MAX_PKT_LEN) return FAIL;
	
	dataPkt = (ADPHeader*)msg;
	APPH = (AppHeader*)msg;
	APPPKT = (AppPkt*)msg;
   

	forward=forwardd;
	PN=APPH->seqNo;
	SaveDATA(msg, txPktLen);
	
	if(forward==0)
		{
		if(TxPending==1) return FAIL;
		else if(TxPending!=1) 
			{
			if(state==WAIT_DATA) return FAIL;
			TxPending=1;
			RpreRetry=0;
			//if(length<=50)
			dataPkt->type = (DATA_PKT << 4) + (dataPkt->type & 0x0f);  // higher 4 bits
			dataPkt->fromAddr = TOS_LOCAL_ADDRESS;
			dataPkt->toAddr = sendAddr;
			No_Pkt=(((length-50)/32)+1);
//			if((state == WAIT_EA)|| (state==WAIT_ACK){}//No-case
			if((state == SLEEP) || (state==S_SLEEP) || (state==POLLING_SENSE))    return SUCCESS;
			
			Send_Rpre();			
			}
		}
	else if(forward==1)
		{
		TxPending=1;
		RpreRetry=0;
		//if(length<=50)
			dataPkt->type = (DATA_PKT << 4) + (dataPkt->type & 0x0f);  // higher 4 bits
		dataPkt->fromAddr = TOS_LOCAL_ADDRESS;
		dataPkt->toAddr = sendAddr;
		No_Pkt=(((length-50)/32)+1);
		Send_Rpre();
		}
		
		
    return SUCCESS;
	}//No-Concate
	
	
	
	
	if(Con==1)
		{
		
		
		SUM = BufFul[0] + BufFul[1] + BufFul[2] + BufFul[3] + BufFul[4];
		if(SUM==5) return SUCCESS;
		
		if(BufFul[0]==0)
			{
			SendTry[0]=0;
			SendRTSTry[0]=0;
			BufFul[0]=1;
			writefunc((AppPkt*)msg, &Buffer0, length);											///////////////////////////change Buffer name to 1,2,3,4
			CCR=updcrc(&Buffer0,(length-sizeof(SuperAppHeader)-6));	////////////////////////////change Buffer name to 1,2,3,4
			Buffer0.crcc = CCR;													////////////////////////////change Buffer name to 1,2,3,4
			}
		else if(BufFul[1]==0)
			{
			SendTry[1]=0;
			SendRTSTry[1]=0;
			BufFul[1]=1;
			writefunc((AppPkt*)msg, &Buffer1, length);											///////////////////////////change Buffer name to 1,2,3,4
			CCR=updcrc(&Buffer1,(length-sizeof(SuperAppHeader)-6));	////////////////////////////change Buffer name to 1,2,3,4
			Buffer1.crcc = CCR;													////////////////////////////change Buffer name to 1,2,3,4
			}		
		else if(BufFul[2]==0)
			{
			SendTry[2]=0;
			SendRTSTry[2]=0;
			BufFul[2]=1;
			writefunc((AppPkt*)msg, &Buffer2, length);											///////////////////////////change Buffer name to 1,2,3,4
			CCR=updcrc(&Buffer2,(length-sizeof(SuperAppHeader)-6));	////////////////////////////change Buffer name to 1,2,3,4
			Buffer2.crcc = CCR;													////////////////////////////change Buffer name to 1,2,3,4
			}		
		else if(BufFul[3]==0)
			{
			SendTry[3]=0;
			SendRTSTry[3]=0;
			BufFul[3]=1;
			writefunc((AppPkt*)msg, &Buffer3, length);											///////////////////////////change Buffer name to 1,2,3,4
			CCR=updcrc(&Buffer3,(length-sizeof(SuperAppHeader)-6));	////////////////////////////change Buffer name to 1,2,3,4
			Buffer3.crcc = CCR;													////////////////////////////change Buffer name to 1,2,3,4
			}
		else if(BufFul[4]==0)
			{
			SendTry[4]=0;
			SendRTSTry[4]=0;
			BufFul[4]=1;
			writefunc((AppPkt*)msg, &Buffer4, length);											///////////////////////////change Buffer name to 1,2,3,4
			CCR=updcrc(&Buffer4,(length-sizeof(SuperAppHeader)-6));	////////////////////////////change Buffer name to 1,2,3,4
			Buffer4.crcc = CCR;													////////////////////////////change Buffer name to 1,2,3,4
			}  
		TxPending=1;
		SUM = BufFul[0] + BufFul[1] + BufFul[2] + BufFul[3] + BufFul[4];
		return SUCCESS;
		}//YES-Concate
	
	}//END-Mac Msg_Send
  
void Send_Rpre()    // construct and send Rpre packet
  {
    Rpre *rtsPkt = (Rpre *)ctrlPkt;
    rtsPkt->type = Rpre_PKT << 4;
    rtsPkt->fromAddr = TOS_LOCAL_ADDRESS;
    rtsPkt->toAddr = sendAddr;
	
	if(Con==1)
	{
	if(WTrunning==1)
			{
			tmpTime= call Waking_Timer.getRemainingTime();
			call Waking_Timer.stop();
			WTrunning=0;
			}
		

	if(BufFul[0]==1)
		{
		SendRTSTry[0]=SendRTSTry[0]+1;
		if(SendRTSTry[0]>R_LIMIT)
			{
			BufFul[0]=0;
			SendTry[0]=0;
			SendRTSTry[0]=0;
			}
		}
	if(BufFul[1]==1)
		{
		SendRTSTry[1]=SendRTSTry[1]+1;
		if(SendRTSTry[1]>R_LIMIT)
			{
			BufFul[1]=0;
			SendTry[1]=0;
			SendRTSTry[1]=0;
			}
		}
	if(BufFul[2]==1)
		{
		SendRTSTry[2]=SendRTSTry[2]+1;
		if(SendRTSTry[2]>R_LIMIT)
			{
			BufFul[2]=0;
			SendTry[2]=0;
			SendRTSTry[2]=0;
			}
		}
	if(BufFul[3]==1)
		{
		SendRTSTry[3]=SendRTSTry[3]+1;
		if(SendRTSTry[3]>R_LIMIT)
			{
			BufFul[3]=0;
			SendTry[3]=0;
			SendRTSTry[3]=0;
			}
		}
	if(BufFul[4]==1)
		{
		SendRTSTry[4]=SendRTSTry[4]+1;
		if(SendRTSTry[4]>R_LIMIT)
			{
			BufFul[4]=0;
			SendTry[4]=0;
			SendRTSTry[4]=0;
			}
		}
	SUM = BufFul[0] + BufFul[1] + BufFul[2] + BufFul[3] + BufFul[4];
	rtsPkt->No_Pkt = SUM;
	if(SUM>0)
		{
		state=WAIT_EA;
		call PhyPkt.send(rtsPkt, sizeof(Rpre), 0);
		tTime = ((((uint32_t)(DIFS + (PHY_BASE_PRE_BYTES) + sizeof(Rpre) + SIFS + (PHY_BASE_PRE_BYTES) + sizeof(EA))) * PHY_TX_BYTE_TIME )/ 1000) + EXTRA;
		call Wait_EA_Timer.start(TIMER_ONE_SHOT, tTime);
		call Waking_Timer.start(TIMER_ONE_SHOT, tmpTime);
		WTrunning=1;
		}
	else
		{
		TxPending=0;
		forward=0;
		signal MacMsg.sendDone(APPPKT, FAIL);
		state=S_SLEEP;
		call RadioState.sleep();
		call POLL_Timer.start(TIMER_ONE_SHOT,POLLING_INTER);
		}
	
	
	}//YES-Concate
	
	
	
	
	
	if(Con==0)
	{
	rtsPkt->Pkt_No = PN;
	
	if(RpreRetry<R_LIMIT)
		{
		if(WTrunning==1)
			{
			tmpTime= call Waking_Timer.getRemainingTime();
			call Waking_Timer.stop();
			WTrunning=0;
			}
		RpreRetry++;
		rtsPkt->No_Pkt=No_Pkt;
		
		state=WAIT_EA;
		call PhyPkt.send(rtsPkt, sizeof(Rpre), 0);
		tTime = ((((uint32_t)(DIFS + (PHY_BASE_PRE_BYTES) + sizeof(Rpre) + SIFS + (PHY_BASE_PRE_BYTES) + sizeof(EA))) * PHY_TX_BYTE_TIME )/ 1000) + EXTRA;
		call Wait_EA_Timer.start(TIMER_ONE_SHOT, tTime);
		call Waking_Timer.start(TIMER_ONE_SHOT, tmpTime);
		WTrunning=1;
		}
	else if(RpreRetry>=R_LIMIT)
		{
		TxPending=0;
		forward=0;
		signal MacMsg.sendDone(APPPKT, FAIL);
		state=S_SLEEP;
		call RadioState.sleep();
		call POLL_Timer.start(TIMER_ONE_SHOT,POLLING_INTER);
		}
	}//No-ConCate
  }  //END-Send_Rpre
		
void Send_EA()	    // construct and send EA
  {
    EA *ctsPkt = (EA *)ctrlPkt;
    ctsPkt->type = EA_PKT << 4;
    ctsPkt->toAddr = recvAddr;
	state=WAIT_DATA;
	
	if(Con==1)
	{
	SUM = BufFul[0] + BufFul[1] + BufFul[2] + BufFul[3] + BufFul[4];
	CNP=5-SUM;
	
	if(CNP<No_Pkt)
		{
		ctsPkt->No_Pkt=CNP;
		}
	else
		{
		ctsPkt->No_Pkt=No_Pkt;
		CNP=No_Pkt;
		}
	tTime = ((((uint32_t)((PHY_BASE_PRE_BYTES) + sizeof(EA) + (PHY_BASE_PRE_BYTES) + (50+(CNP-1)*32) + (2 * SIFS) )) * PHY_TX_BYTE_TIME )/ 1000) + EXTRA;
	
	}//Yes-Concate
	
	if(Con==0)
	{
	ctsPkt->No_Pkt=No_Pkt;
	tTime = ((((uint32_t)((PHY_BASE_PRE_BYTES) + sizeof(EA) + (PHY_BASE_PRE_BYTES) + sizeof(AppPkt) + (2 * SIFS) )) * PHY_TX_BYTE_TIME )/ 1000) + EXTRA;
	}//No-Concate
	
	call Wait_DATA_Timer.start(TIMER_ONE_SHOT, tTime);
    call PhyPkt.send(ctsPkt, sizeof(EA), 0);
	
 } //END-Send-EA
	
void Send_DATA()	    // construct and send DATA
  {
	
	if(Con==1)
		{
		if(EASUM>0)
		{
		inc=0;
		PayLd=(50-sizeof(SuperAppHeader)-4);
		if(BufFul[0]==1)
			{
			SendTry[0]=SendTry[0]+1;
			SPLL[inc]=0;
			writefuncS(&Buffer0, &Buffer, inc, PayLd);											///////////////////////////change Buffer name to 1,2,3,4
			inc=inc+1;
			}
		if((BufFul[1]==1) && (inc < EASUM))
			{
			SendTry[1]=SendTry[1]+1;
			SPLL[inc]=1;
			writefuncS(&Buffer1, &Buffer, inc, PayLd);											///////////////////////////change Buffer name to 1,2,3,4
			inc=inc+1;
			}
		if((BufFul[2]==1) && (inc < EASUM))
			{
			SendTry[2]=SendTry[2]+1;
			SPLL[inc]=2;
			writefuncS(&Buffer2, &Buffer, inc, PayLd);											///////////////////////////change Buffer name to 1,2,3,4
			inc=inc+1;
			}
		if((BufFul[3]==1) && (inc < EASUM))
			{
			SendTry[3]=SendTry[3]+1;
			SPLL[inc]=3;
			writefuncS(&Buffer3, &Buffer, inc, PayLd);											///////////////////////////change Buffer name to 1,2,3,4
			inc=inc+1;
			}
		if((BufFul[4]==1) && (inc < EASUM))
			{
			SendTry[4]=SendTry[4]+1;
			SPLL[inc]=4;
			writefuncS(&Buffer4, &Buffer, inc, PayLd);											///////////////////////////change Buffer name to 1,2,3,4
			inc=inc+1;
			}
		SendRTSTry[0]=0;
		SendRTSTry[1]=0;
		SendRTSTry[2]=0;
		SendRTSTry[3]=0;
		SendRTSTry[4]=0;
		SPS=50+(inc-1)*32;
	
		Buffer.hdr.hdr.type=SDATA_PKT << 4;
		Buffer.hdr.hdr.fromAddr=TOS_LOCAL_ADDRESS;
		Buffer.hdr.hdr.toAddr=sendAddr;

		if(MULTISENDER==0)
			Buffer.hdr.Ori_Source=TST_MIN_NODE_ID;
		else
			Buffer.hdr.Ori_Source=TOS_LOCAL_ADDRESS;
		Buffer.hdr.Final_Dst=TST_MAX_NODE_ID;
		
		Buffer.hdr.TxTime=call LocalTime.get();
	
		state=WAIT_ACK;
		atTime = ((((uint32_t)((PHY_BASE_PRE_BYTES) + SPS + DDelay +(2 *SIFS) + (PHY_BASE_PRE_BYTES) + sizeof(ACK))) * PHY_TX_BYTE_TIME )/ 1000) + EXTRA;
		call Wait_ACK_Timer.start(TIMER_ONE_SHOT, atTime);
	
		call PhyPkt.send(&Buffer, SPS, 0);
		}
		else
		{
		state=S_SLEEP;
		call RadioState.sleep();
		call POLL_Timer.start(TIMER_ONE_SHOT,POLLING_INTER);
		}
		
		}//Yes-Concate
	
	if(Con==0)
	{
	state=WAIT_ACK;
	atTime = ((((uint32_t)((PHY_BASE_PRE_BYTES) + txPktLen + DDelay +(2 *SIFS) + (PHY_BASE_PRE_BYTES) + sizeof(ACK))) * PHY_TX_BYTE_TIME )/ 1000) + EXTRA;
	call Wait_ACK_Timer.start(TIMER_ONE_SHOT, atTime);
	
    call PhyPkt.send(APPPKT, txPktLen, 0);
	
	}//No-Concate
	} //END-Send-DATA
	
	
	
void Send_ACK()    // construct and send ACK
  {
    ACK *ackPkt = (ACK *)ctrlPkt;
    ackPkt->type = ACK_PKT << 4;
    ackPkt->toAddr = recvAddr;
	ackPkt->NP=No_Pkt;
	ackPkt->Pkt_No = PN;
	ackPkt-> Ori_Source = OS;
	ackPkt->CVV=CV;
	
	if(Con==1)
		ackPkt->NP=SAP;
	
    call PhyPkt.send(ackPkt, sizeof(ACK), 0);
	
	if(APPH->Final_Dst == TOS_LOCAL_ADDRESS)
		{		//goto sleep when ACK Sen-Done		
		}
	else
		{	
		//forward=1;
		//TxPending=1;
		// send a message when ACK Send-DONE
		}
  }  //END-Send_ACK

  
  
  
  
  async event result_t PhyNotify.startSymSent(void* pkt)
  {
    // can be used to put timestamp on outgoing pkt
    return SUCCESS;
  }

async event result_t PhyNotify.startSymDetected(void* pkt, uint8_t bitOffset)    // start symbol is detected
  {
   // this event is signalled asynchronously within interrupt handler
 /*  
    if(state == BACKOFF) {
      call BackoffTimer.stop();  // stop backoff timer
    }
    if(state == IDLE || state == CARR_SENSE || state == BACKOFF) {       
      state = RECEIVE;
    }
*/
    return SUCCESS;
  }


command void ADPControl.addPreamble(uint16_t length)
  {
    // set the length of additional preamble
    addPreambleLen = length;
  }
command void ADPControl.setContWin(uint8_t numSlots)
  {
    // set contention window size in terms of number of slots
    
    ContWinSize = numSlots;
  }
    
  command void ADPControl.setBackoffTime(uint32_t time)
  {
    // set backoff time when carrier sense fails
    // should be longer than start symbol transmission time
    
 //   backoffTime = time;
  }
  
  
  command void ADPControl.disableAutoReTx()
  {
    // enable automatic re-send after previous attempt fails
    
 //   autoReTx = FALSE;
  }
  
  
  
  
  
  
  
  
  
  
  
  
  
async event result_t RadioState.wakeupDone()    // radio wakeup is done -- it becomes stable now
  {
//      signal MacActivity.virtualCSIdle();
    return SUCCESS;
  }
void txMsgDone(result_t result)
  {
//  signal MacMsg.sendDone(dataPkt, result);
  }
event result_t PhyPkt.sendDone(void* spacket)    // transmit packet is done by physical layer
  {

    char spktType;
	uint8_t d;
    
	spktType = (*((char*)spacket + sizeof(PhyHeader))) >> 4;
    

	switch (spktType) {  // the type field
    case BCAST_DATA_PKT:  // just sent a broadcast data
      break;
    case Rpre_PKT:  // just sent RTS
//		state = WAIT_EA;
      break;
    case EA_PKT:  // just sent CTS
//		state = WAIT_DATA;
      break;
    case DATA_PKT:  // just sent unicast data
//		state = WAIT_ACK;  // waiting for ACK
//		call Wait_ACK_Timer.start(TIMER_ONE_SHOT, atTime);
      break;
    case ACK_PKT:
		if(APPH->Final_Dst == TOS_LOCAL_ADDRESS)
	//	if(TOS_LOCAL_ADDRESS == TST_MAX_NODE_ID)
			{
			signal MacMsg.receiveDone(APPPKT);
			if(Con==1) //empty the Buffer
				{
				BufFul[0]=0;
				BufFul[1]=0;
				BufFul[2]=0;
				BufFul[3]=0;
				BufFul[4]=0;
				SendTry[0]=0;
				SendTry[1]=0;
				SendTry[2]=0;
				SendTry[3]=0;
				SendTry[4]=0;
				SendRTSTry[0]=0;
				SendRTSTry[1]=0;
				SendRTSTry[2]=0;
				SendRTSTry[3]=0;
				SendRTSTry[4]=0;
				}
			
				//state=S_SLEEP;
				//call RadioState.sleep();
				//call POLL_Timer.start(TIMER_ONE_SHOT, POLLING_INTER);
			call Wait_Timer.start(TIMER_ONE_SHOT, 102);// approx 100msecs // 
			state=POLLING_SENSE;				
			}
		else
			{
			
			forward=1;
			for(d=0;d<DDelay;d++)
				{}
			call MacMsg.send(APPPKT, sizeof(AppPkt), TOS_LOCAL_ADDRESS + 1, forward);    // send a message
				
			}
      break;
    }

    return SUCCESS;
  }
  
  
  
  
  
  
  
  void handleErrPkt()    // an erronous packet is received
  {
/* 
    if(state == RECEIVE) {
      state = IDLE;
      checkActivity(FAIL);  // check if upper layer want me to send now  
    }
*/

	//if(WTrunning==0)
	//			{call Waking_Timer.start(TIMER_ONE_SHOT, tmpTime);
	//			WTrunning=1;
	//			}
	//		call POLL_Timer.start(TIMER_ONE_SHOT, POLLING_INTER);
	//		state=S_SLEEP;
	//		call RadioState.sleep();
////XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX			
//			
//	if(state==POLLING_SENSE)
//		{
//		if(TxPending!=1){}
//		else if(TxPending==1){}
//		}
//    else if(state==WAIT_EA)
//		{
//		if(TxPending!=1){}
//		else if(TxPending==1){}
//		}    
//    else if(state==WAIT_DATA)
//		{
//		if(TxPending!=1){}
//		else if(TxPending==1){}
//		}
//    else if(state==WAIT_ACK)
//		{
//		if(TxPending!=1){}
//		else if(TxPending==1){}
//		}
//			
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXx			
			
			
  }
  command result_t MacActivity.reSend()    // resend a previously buffered packet
  {
/*   
    if(txPktState != BUFFERED) return FAIL;
    return tryToSend();
*/
	return SUCCESS;
  }
  default event void MacActivity.radioDone(result_t result)
  {
    // default do-nothing handler
  }
  
  
  default event void MacActivity.virtualCSBusy()
  {
    // default do-nothing handler
  }
  
  
  default event void MacActivity.virtualCSIdle()
  {
    // default do-nothing handler
  }
command result_t MacMsg.sendCancel(void* msg) // cancel a message to be sent (i.e., previously called MacMsg.send)
  {
   
    return  SUCCESS;
  }
  
  
  
}  // end of implementation
