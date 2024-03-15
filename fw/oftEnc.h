// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
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

namespace moteus {

class OftEnc {

      typedef struct
      {
        uint8_t start;
        uint8_t flags;
        uint16_t val;
        uint16_t chk;
      } AS5x47U_UART_PKT;


 public:
  OftEnc(const aux::UartEncoder::Config& config,
           Stm32G4DmaUart* uart,
           MillisecondTimer* timer)
      : config_(config),
        uart_(uart),
        timer_(timer) {}


  void ISR_Update(aux::UartEncoder::Status* status) MOTEUS_CCM_ATTRIBUTE {

    if (!dmaInitialized_) {
        StartRead();
        dmaInitialized_ = true;
    }

    ProcessQuery(status);
   
    
  }

 private:

  unsigned short as_crc_update (unsigned short crc, unsigned char data)
  {
    data ^= (crc & 0xff);
    data ^= data << 4;

    return ((((unsigned short )data << 8) | ((crc>>8)&0xff)) ^ (unsigned char )(data >> 4)
        ^ ((unsigned short )data << 3));
  }

  unsigned short as_crc16(void* data, unsigned short cnt)
  {
    unsigned short crc=0xff;
    unsigned char * ptr=(unsigned char *) data;
    int i;

    for (i=0;i<cnt;i++)
    {
      crc=as_crc_update(crc,*ptr);
      ptr++;
    }
    return crc;
  }

  void ProcessQuery(aux::UartEncoder::Status* status) MOTEUS_CCM_ATTRIBUTE {

    uint8_t dmaReadPos=sizeof(buffer_)-uart_->read_bytes_remaining();
   
    if (dmaReadPos==bufferReadPos_)
    {
      return;
    }

    int8_t dmaSize=(int8_t)dmaReadPos-(int8_t)bufferReadPos_;
    if (dmaSize<0)
    {
      dmaSize+=sizeof(buffer_);
    }
    
    
    for (uint32_t i=0;i<(uint32_t)dmaSize;i++)
    {
        uint8_t res=buffer_[(bufferReadPos_+i)%sizeof(buffer_)];
        switch(uartState_)
        {
          case 0:
            if (res==0xAA)
            {
              uartState_++;
              uartPkt_.start=res;
            }
          break;
          case 1:
            uartPkt_.flags=res;
            uartState_++;
          break;
          case 2:
            uartPkt_.val=res;
            uartState_++;
          break;
          case 3:
            uartPkt_.val|=res<<8;
            uartState_++;
          break;
          case 4:
            uartPkt_.chk=res;
            uartState_++;
          break;
          case 5:
            uartPkt_.chk|=res<<8;

            if (as_crc16(&uartPkt_.start,sizeof(AS5x47U_UART_PKT)-2)==uartPkt_.chk)
            {
              if (uartPkt_.flags==0x01)
              {

                status->value = uartPkt_.val & 0x3fff;
                status->nonce++;
                status->active = true; 
               
              }
            }else
            {
               status->checksum_errors++;
   
            }
    

            uartState_=0;
          break;
          default:
            uartState_=0;
          break;


        }


    }
    
    bufferReadPos_+=dmaSize;
    bufferReadPos_%=sizeof(buffer_);
  }

  void StartRead() MOTEUS_CCM_ATTRIBUTE {
    uart_->start_dma_read_circular(
        mjlib::base::string_span(reinterpret_cast<char*>(&buffer_[0]),
                                 sizeof(buffer_)));
  }

  const aux::UartEncoder::Config config_;
  Stm32G4DmaUart* const uart_;
  MillisecondTimer* const timer_;

  bool dmaInitialized_ = false;

  uint32_t last_query_start_us_ = 0;

  uint8_t buffer_[64] = {};
  AS5x47U_UART_PKT uartPkt_;
  uint8_t uartState_=0; 
  uint8_t bufferReadPos_=0;
};

}
