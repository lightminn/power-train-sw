"""Compile unchanged MKS write/HAL functions with a deterministic preemption stub.
This is a source-level counterexample, not execution on an STM32 or proof of
which HAL error the real board held. Input: extracted MKS V3.6 firmware source.
"""
from pathlib import Path
import subprocess,sys,tempfile,zipfile
source=Path(sys.argv[1])
def read_source(path):
 if source.is_dir():return (source/path).read_text()
 with zipfile.ZipFile(source) as archive:
  matches=[n for n in archive.namelist() if n.endswith('/'+path) and '/Firmware/' in n]
  assert len(matches)==1,matches
  return archive.read(matches[0]).decode()
def function(path,signature):
 text=read_source(path);start=text.index(signature);opening=text.index('{',start);depth=1;end=opening+1
 while depth:
  if text[end]=='{':depth+=1
  elif text[end]=='}':depth-=1
  end+=1
 return text[start:end]
write=function('interface_can.cpp','uint32_t ODriveCAN::write(')
add=function('stm32f4xx_hal_can.c','HAL_StatusTypeDef HAL_CAN_AddTxMessage(')
prelude=r'''
#include <cstdint>
#include <iostream>
#include <cassert>
using HAL_StatusTypeDef=int;
constexpr int HAL_OK=0,HAL_ERROR=1,HAL_CAN_STATE_READY=1,HAL_CAN_STATE_LISTENING=2;
constexpr uint32_t HAL_CAN_ERROR_NONE=0,HAL_CAN_ERROR_PARAM=0x00200000,HAL_CAN_ERROR_NOT_INITIALIZED=0x00040000;
constexpr uint32_t CAN_ID_STD=0,CAN_ID_EXT=4,CAN_RTR_DATA=0,RESET=0;
constexpr uint32_t CAN_TSR_TME0=1u<<26,CAN_TSR_TME1=1u<<27,CAN_TSR_TME2=1u<<28,CAN_TSR_CODE=3u<<24,CAN_TSR_CODE_Pos=24;
constexpr uint32_t CAN_TI0R_STID_Pos=21,CAN_TI0R_EXID_Pos=3,CAN_TI0R_TXRQ=1,CAN_TDT0R_TGT=8;
constexpr int CAN_TDH0R_DATA7_Pos=24,CAN_TDH0R_DATA6_Pos=16,CAN_TDH0R_DATA5_Pos=8,CAN_TDH0R_DATA4_Pos=0;
constexpr int CAN_TDL0R_DATA3_Pos=24,CAN_TDL0R_DATA2_Pos=16,CAN_TDL0R_DATA1_Pos=8,CAN_TDL0R_DATA0_Pos=0;
enum FunctionalState {DISABLE=0,ENABLE=1};
struct Mailbox {uint32_t TIR=0,TDTR=0,TDHR=0,TDLR=0;};
struct Registers {uint32_t TSR=CAN_TSR_TME0;Mailbox sTxMailBox[3];} regs;
struct CAN_HandleTypeDef {int State=HAL_CAN_STATE_LISTENING;uint32_t ErrorCode=0;Registers* Instance=&regs;};
struct CAN_TxHeaderTypeDef {uint32_t StdId,ExtId,IDE,RTR,DLC;FunctionalState TransmitGlobalTime;};
struct can_Message_t {uint32_t id=0x1a9;bool isExt=false;uint32_t len=8;uint8_t buf[8]={};};
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef* h) {return h->ErrorCode;}
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef* h);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef*,CAN_TxHeaderTypeDef*,uint8_t[],uint32_t*);
class ODriveCAN {public:CAN_HandleTypeDef* handle_;uint32_t error_=0;uint32_t write(can_Message_t&);};
void set_bit(uint32_t& reg,uint32_t value){reg|=value;if(value==CAN_TI0R_TXRQ)for(int i=0;i<3;++i)if(&reg==&regs.sTxMailBox[i].TIR)regs.TSR&=~(CAN_TSR_TME0<<i);}
#define assert_param(x) ((void)0)
#define SET_BIT(r,v) set_bit(r,v)
#define WRITE_REG(r,v) ((r)=(v))
'''
postlude=r'''
ODriveCAN* active=nullptr;bool inject_preemption=false;uint32_t interrupting_result=99;
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef* h) {
 const uint32_t observed=__builtin_popcount(h->Instance->TSR&(CAN_TSR_TME0|CAN_TSR_TME1|CAN_TSR_TME2));
 if(inject_preemption){inject_preemption=false;can_Message_t heartbeat;heartbeat.id=0x1a1;interrupting_result=active->write(heartbeat);}
 return observed;
}
int main(){
 CAN_HandleTypeDef h;ODriveCAN can;can.handle_=&h;active=&can;can_Message_t reply;
 const auto normal=can.write(reply);assert(normal==1 && h.ErrorCode==0);
 regs.TSR=CAN_TSR_TME0;inject_preemption=true;
 const auto raced=can.write(reply);
 assert(interrupting_result==1 && raced==0 && h.ErrorCode==HAL_CAN_ERROR_PARAM);
 assert(can.error_==0);
 regs.TSR=CAN_TSR_TME0|CAN_TSR_TME1|CAN_TSR_TME2;
 const auto heartbeat0=can.write(reply),heartbeat1=can.write(reply);
 assert(heartbeat0==uint32_t(-1) && heartbeat1==uint32_t(-1));
 std::cout<<"PASS: normal send succeeds; heartbeat preemption consumes the last mailbox; unchanged HAL AddTxMessage sets PARAM; USB-facing can.error remains 0; subsequent writes are blocked even with all mailboxes free. HAL error="<<h.ErrorCode<<"\n";
}
'''
with tempfile.TemporaryDirectory(prefix='mks-can-race-') as d:
 cpp=Path(d)/'race.cpp';exe=Path(d)/'race';cpp.write_text(prelude+'\n'+add+'\n'+write+'\n'+postlude)
 subprocess.run(['g++','-std=c++17','-O0','-Wall','-Wextra',str(cpp),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
