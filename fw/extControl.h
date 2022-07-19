// Copyright 2022 Josh Pieper, jjp@pobox.com.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "fw/aux_common.h"
#include "fw/millisecond_timer.h"
#include "fw/stm32g4_dma_uart.h"

//#define YOKE
#define THRUST_RIGHT
//#define THRUST_LEFT
namespace moteus {


  
class ExtControl {
 public:
   enum
  {
	UART_SB1=0xAA,
	UART_SB2=0x55,
  };
  
  enum
  {
	UART_TX_SEL_POS=0,
	UART_TX_SEL_SPEED=1,
	UART_TX_SEL_TORQUE=2,
	
	
	UART_TX_SEL_MAX=3,
  };
  enum
  {
	  EXTC_CMD_INIT=1,
	  EXTC_CMD_NORMAL_MODE=2,
	  EXTC_CMD_STOP=3,
  };
  struct UART_PACKET
  {
	  uint8_t sb1;
	  uint8_t sb2;
	  uint8_t msgT;
	  uint8_t data[4];
	  uint8_t ck;
  };
  //the G4 uart has an 8-byte fifo...if we don't exceed this we can just push 8 bytes without waiting...

  ExtControl(Stm32G4DmaUart* uart,
         MillisecondTimer* timer)
      : uart_(uart),
        timer_(timer) {}

  void handleRxByte(uint8_t byte)
  {
		switch(rxState_)
		{
			case 0:
				if (byte==0xAA)
					rxState_++;
			break;
			case 1:
				if (rxState_==0x55)
					rxState_++;
			break;
			case 2:
				rxCmd_=byte;
				rxChk_=0x33;
				rxChk_+=byte;
				rxCnt_=0;
			break;
			case 3:
				rxChk_+=byte;
				rxData_[rxCnt_]=byte;
				rxCnt_++;
				if (rxCnt_>=4)
					rxState_++;
			break;
			case 4:
				if (rxChk_==byte)
				{
					actCmd_=rxCmd_;
					initState_=0;
					memcpy(&rxNewVal_,&rxData_[0],sizeof(rxNewVal_));
				}
					
		}
  }

  void PollMillisecond()
  {
	  if (msTimer_<10000)
	  {
		msTimer_++;
		
		if ((msTimer_%10)==0)
		{
			struct UART_PACKET pkt;
			pkt.sb1=UART_SB1;
			pkt.sb2=UART_SB2;
			pkt.msgT=uartTxSelect_;
			
			switch(uartTxSelect_)
			{
				case UART_TX_SEL_POS:
					memcpy(&pkt.data[0],&pos_,sizeof(pos_));
				break;
				case UART_TX_SEL_SPEED:
					memcpy(&pkt.data[0],&speed_,sizeof(speed_));
				break;
				case UART_TX_SEL_TORQUE:
					memcpy(&pkt.data[0],&torque_,sizeof(torque_));
				break;
			}
			
			pkt.ck=0xAA;
			for (uint8_t i=2;i<sizeof(pkt)-1;i++)
				pkt.ck+=((uint8_t*)&pkt)[i];
			
			for (uint8_t i=0;i<sizeof(pkt);i++)
				uart_->write_char( ((uint8_t*)&pkt)[i]);
			
			uartTxSelect_++;
			uartTxSelect_%=UART_TX_SEL_MAX;
			
		}
	  }
	
	
	if (!query_outstanding_)
	{
		uartRdPtr_=0;
		lastUartBytesRem_=sizeof(buffer_);
		StartRead();
		
		query_outstanding_=true;
	}else
	{
		if (lastUartBytesRem_!=uart_->bytes_remaining())
		{
			//we received data -> process it
			
			uint8_t bytesInBuffer=sizeof(buffer_)-uartRdPtr_;
			for (uint8_t i=uartRdPtr_;i<bytesInBuffer;i++)
			{
				uartRxByteCnt_++;
				handleRxByte(buffer_[i]);
			}
			uartRdPtr_=bytesInBuffer;
			
			lastUartBytesRem_=uart_->bytes_remaining();
		}else
		{
			//nothing happened in the last cycle. If neccessary, reset uart dma
			if (uart_->bytes_remaining()!=sizeof(buffer_))
			{
				uart_->finish_dma_read();
				uartRdPtr_=0;
				lastUartBytesRem_=sizeof(buffer_);
				StartRead();
			}
			
		}
		
	}
	  
#ifdef THRUST_RIGHT
	  if (actCmd_==EXTC_CMD_INIT)
	  {
		  if (initState_==0)
		  {
			initState_++;
		  }else
		  if (initState_==1)
		  {
			
			zeroPos_=0.0f;
			cmdPos_=std::numeric_limits<float>::quiet_NaN();
			cmdSpeed_=+0.5f;
			cmdKp_=0.0f;
			cmdKd_=1.0f;
			cmdTorque_=0.2f;
			controlActive_=true;
			
			if (fabs(speed_)<0.25f)
			{
				_limitDetectedCnt++;
				
				if (_limitDetectedCnt>500)
				{
					zeroPos_=pos_-1.72f;
					initState_=2;
				}
			}else
				_limitDetectedCnt=0;
			
			lastPos_=pos_;
		  }else if (initState_==2)
		  {
			if (pos_>0.0f)
			{
				cmdPos_=std::numeric_limits<float>::quiet_NaN();
				cmdSpeed_=-0.4f;
				cmdKp_=1.0f;
				cmdKd_=1.0f;
				cmdTorque_=0.1f;
				controlActive_=true;
			}else
			{
				initState_=0;
				actCmd_=EXTC_CMD_NORMAL_MODE;
			}
		  }
	  }else
	  if (actCmd_==EXTC_CMD_NORMAL_MODE)
	  {
		  #define RANGE_LIMIT 1.0f
		  if ((pos_<0.1f) && (pos_>-0.1f))
		  {
				cmdPos_=std::numeric_limits<float>::quiet_NaN();
				cmdSpeed_=0.0f;
				cmdKp_=0.0f;
				cmdKd_=3.0f-fabs(pos_)*20.0f;
				cmdTorque_=0.6f;
				controlActive_=true;
		  }
		  else if (pos_>RANGE_LIMIT)
		  {
				cmdPos_=RANGE_LIMIT;
				cmdSpeed_=0.0f;
				cmdKp_=0.1f+(pos_-RANGE_LIMIT)*5.0f;
				if (cmdKp_<0.1f)
					cmdKp_=0.1f;
				if (cmdKp_>1.0f)
					cmdKp_=1.0f;
				cmdKd_=1.0f;
				cmdTorque_=0.8f;
				stopAct_=1;	
				controlActive_=true;
		  }else if (pos_<-RANGE_LIMIT)
		  {
				cmdPos_=-RANGE_LIMIT;
				cmdSpeed_=0.0f;
				cmdKp_=0.1f-(pos_+RANGE_LIMIT)*5.0f;
				if (cmdKp_<0.1f)
					cmdKp_=0.1f;
				if (cmdKp_>1.0f)
					cmdKp_=1.0f;
				cmdKd_=1.0f;
				cmdTorque_=0.8f;
				stopAct_=1;	
				controlActive_=true;
		  }
		  else
		  {
				cmdPos_=std::numeric_limits<float>::quiet_NaN();
				cmdSpeed_=0.0f;
				cmdKp_=0.0f;
				cmdKd_=1.0f;
				cmdTorque_=0.2f;
				controlActive_=true;
		  }
	  }else
	  if (actCmd_==EXTC_CMD_STOP)
	  {
		  controlActive_=false;
	  }
#endif


#ifdef YOKE	  
	  if (actCmd_==EXTC_CMD_INIT)
	  {
		  if (initState_==0)
		  {
			zeroPos_=posRaw_;
			pos_=posRaw_-zeroPos_;
			initState_++;
		  }else
		  if (initState_==1)
		  {
			
			if (pos_>-0.8f)
			{
				cmdPos_=std::numeric_limits<float>::quiet_NaN();
				cmdSpeed_=-0.4f;
				cmdKp_=1.0f;
				cmdKd_=1.0f;
				cmdTorque_=0.1f;
				controlActive_=true;
			}else
				initState_++;
		  }else if (initState_==2)
		  {
			if (pos_<0.8f)
			{
				cmdPos_=std::numeric_limits<float>::quiet_NaN();
				cmdSpeed_=0.4f;
				cmdKp_=1.0f;
				cmdKd_=1.0f;
				cmdTorque_=0.1f;
				controlActive_=true;
			}else
				initState_++;
		  }else if (initState_==3)
		  {
			if (pos_>0.0f)
			{
				cmdPos_=std::numeric_limits<float>::quiet_NaN();
				cmdSpeed_=-0.4f;
				cmdKp_=1.0f;
				cmdKd_=1.0f;
				cmdTorque_=0.1f;
				controlActive_=true;
			}else
			{
				initState_=0;
				actCmd_=EXTC_CMD_NORMAL_MODE;
			}
		  }
	  }else
	  if (actCmd_==EXTC_CMD_NORMAL_MODE)
	  {
		  if ((pos_<0.1f) && (pos_>-0.1f))
		  {
				cmdPos_=std::numeric_limits<float>::quiet_NaN();
				cmdSpeed_=0.0f;
				cmdKp_=0.0f;
				cmdKd_=4.0f-fabs(pos_)*30.0f;
				cmdTorque_=0.6f;
				controlActive_=true;
		  }else if (pos_>0.8f)
		  {
				cmdPos_=0.8f;
				cmdSpeed_=0.0f;
				cmdKp_=0.1f+(pos_-0.8f)*5.0f;
				if (cmdKp_<0.1f)
					cmdKp_=0.1f;
				if (cmdKp_>1.0f)
					cmdKp_=1.0f;
				cmdKd_=1.0f;
				cmdTorque_=0.8f;
				stopAct_=1;	
				controlActive_=true;
		  }else if (pos_<-0.8f)
		  {
				cmdPos_=-0.8f;
				cmdSpeed_=0.0f;
				cmdKp_=0.1f-(pos_+0.8f)*5.0f;
				if (cmdKp_<0.1f)
					cmdKp_=0.1f;
				if (cmdKp_>1.0f)
					cmdKp_=1.0f;
				cmdKd_=1.0f;
				cmdTorque_=0.8f;
				stopAct_=1;	
				controlActive_=true;
		  }else
		  {
				cmdPos_=std::numeric_limits<float>::quiet_NaN();
				cmdSpeed_=0.0f;
				cmdKp_=0.0f;
				cmdKd_=1.0f;
				cmdTorque_=0.2f;
				controlActive_=true;
		  }
	  }else
	  if (actCmd_==EXTC_CMD_STOP)
	  {
		  controlActive_=false;
	  }
#endif


  }
  
  
  void resetTimer()
  {
	  msTimer_=0;
  }
  
  
  uint8_t active()
  {
	return controlActive_;
  }
  
  void updateState(float pos, float speed, float torque)
  {
	  posRaw_=pos;
	  pos_=posRaw_-zeroPos_;
	  speed_=speed;
	  torque_=torque;
  }
  void getCommand(float* desiredPos, float* speed, float* kp, float* kd, float* torque)
  {
	  *desiredPos=cmdPos_;
	  *speed=cmdSpeed_;
	  *kp=cmdKp_;
	  *kd=cmdKd_;
	  *torque=cmdTorque_;
  }
  
  void ISR_Update(aux::UartEncoder::Status* status) MOTEUS_CCM_ATTRIBUTE {
	  
	status->nonce++;
    status->active = true;
	status->value = uartRxByteCnt_;
	
  }


 private:
  void StartRead() MOTEUS_CCM_ATTRIBUTE {
    uart_->start_dma_read(
        mjlib::base::string_span(reinterpret_cast<char*>(&buffer_[0]),
                                 sizeof(buffer_)));
  }

  Stm32G4DmaUart* const uart_;
  MillisecondTimer* const timer_;

  bool query_outstanding_ = false;

  uint32_t last_query_start_us_ = 0;
  bool controlActive_=false;
  
  uint32_t msTimer_=0;
  
  static constexpr int kResyncBytes = 3;
  static constexpr int kMaxCount = 50;


  float cmdPos_=0.0f; 
  float cmdSpeed_=0.0f;
  float cmdKp_=0.0f;
  float cmdKd_=0.0f;
  float cmdTorque_=0.0f;
  
  float pos_=0.0f;
  float posRaw_=0.0f;
  float speed_=0.0f;
  float torque_=0.0f;
  float lastPos_=0.0f;
  uint8_t uartTxSelect_=0;
  // The "detailed" reply has a header byte, 3 bytes of position, and
  // 2 bytes of status.
  //
  // We have 3 extra bytes so that we could eventually re-synchronize.
  uint8_t buffer_[16] = {};
  uint8_t rxState_=0;
  

  uint8_t rxCmd_=0;
  uint8_t actCmd_=EXTC_CMD_INIT;
  uint8_t rxChk_;
  uint8_t rxCnt_;
  uint8_t stopAct_=0;
  uint8_t rxData_[4];
  uint8_t initState_=0;
  uint16_t _limitDetectedCnt=0;
  float rxNewVal_;
  float zeroPos_=0.0f;
  uint8_t uartRdPtr_=0;
  uint8_t lastUartBytesRem_=0;
  uint32_t uartRxByteCnt_=0;
};

}
