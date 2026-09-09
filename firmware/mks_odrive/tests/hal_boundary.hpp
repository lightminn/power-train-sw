#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <limits>
#include <functional>
#include <vector>
using HAL_StatusTypeDef=int;
constexpr int HAL_OK=0,HAL_ERROR=1,HAL_CAN_STATE_RESET=0,HAL_CAN_STATE_READY=1,HAL_CAN_STATE_LISTENING=2;
constexpr uint32_t HAL_CAN_ERROR_NONE=0,HAL_CAN_ERROR_TIMEOUT=0x20000,HAL_CAN_ERROR_PARAM=0x00200000,HAL_CAN_ERROR_NOT_INITIALIZED=0x00040000;
constexpr uint32_t CAN_ID_STD=0,CAN_ID_EXT=4,CAN_RTR_DATA=0,RESET=0;
constexpr uint32_t CAN_TSR_TME0=1u<<26,CAN_TSR_TME1=1u<<27,CAN_TSR_TME2=1u<<28,CAN_TSR_CODE=3u<<24,CAN_TSR_CODE_Pos=24;
constexpr uint32_t CAN_TI0R_STID_Pos=21,CAN_TI0R_EXID_Pos=3,CAN_TI0R_TXRQ=1,CAN_TDT0R_TGT=8;
constexpr int CAN_TDH0R_DATA7_Pos=24,CAN_TDH0R_DATA6_Pos=16,CAN_TDH0R_DATA5_Pos=8,CAN_TDH0R_DATA4_Pos=0;
constexpr int CAN_TDL0R_DATA3_Pos=24,CAN_TDL0R_DATA2_Pos=16,CAN_TDL0R_DATA1_Pos=8,CAN_TDL0R_DATA0_Pos=0;
constexpr uint32_t CAN_RX_FIFO0=0,CAN_RX_FIFO1=1,CAN_IT_RX_FIFO0_MSG_PENDING=1,CAN_IT_ERROR=2,CAN_IT_BUSOFF=4,CAN_IT_LAST_ERROR_CODE=8,CAN_IT_ERROR_WARNING=16,CAN_IT_ERROR_PASSIVE=32,CAN_IT_RX_FIFO0_OVERRUN=64,CAN_IT_RX_FIFO1_OVERRUN=128,CAN_TX_MAILBOX0=1,CAN_TX_MAILBOX1=2,CAN_TX_MAILBOX2=4;
constexpr int CAN_FILTERMODE_IDMASK=0,CAN_FILTERSCALE_32BIT=0;
enum FunctionalState {DISABLE=0,ENABLE=1};
struct Mailbox {uint32_t TIR=0,TDTR=0,TDHR=0,TDLR=0;};
struct Registers {uint32_t TSR=CAN_TSR_TME0;Mailbox sTxMailBox[3];} regs;
Registers* const CAN1=&regs;
struct CAN_HandleTypeDef {int State=HAL_CAN_STATE_LISTENING;uint32_t ErrorCode=0;Registers* Instance=&regs;struct {uint32_t Prescaler=4;} Init;};
struct CAN_TxHeaderTypeDef {uint32_t StdId,ExtId,IDE,RTR,DLC;FunctionalState TransmitGlobalTime;};
struct CAN_RxHeaderTypeDef {uint32_t StdId=0,ExtId=0,IDE=0,RTR=0,DLC=8;};
struct CAN_FilterTypeDef {uint32_t FilterActivation=0,FilterBank=0,FilterFIFOAssignment=0,FilterIdHigh=0,FilterIdLow=0,FilterMaskIdHigh=0,FilterMaskIdLow=0,FilterMode=0,FilterScale=0,SlaveStartFilterBank=0;};
struct StopLoop{};
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
uint32_t irq_mask=0, delay_count=0, waits=0, error_reads=0, stop_calls=0, init_calls=0, reset_calls=0, rx_reads=0;
uint32_t fail_inits=0, fifo[2]={0,0}, can_irqs_disabled=0, rcc_reset_count=0, pending_irq_cleared=0;
bool can_clock_enabled=true;
int wait_budget=1;
bool reinit_active=false, fail_stop=false;
std::function<void()> deferred_preemption, inject_free, during_reinit, inject_register_fault;
uint32_t cpu_enter_critical(){uint32_t old=irq_mask;irq_mask=1;return old;}
void cpu_exit_critical(uint32_t old){irq_mask=old;if(!irq_mask && deferred_preemption){auto f=deferred_preemption;deferred_preemption=nullptr;f();}}
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef* h){if(++error_reads>100)throw std::runtime_error("busy spin: more than 100 HAL error polls without a bounded wait");return h->ErrorCode;}
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef* h){
 require(!reinit_active,"TX HAL reached during reinit");
 uint32_t value=__builtin_popcount(h->Instance->TSR&(CAN_TSR_TME0|CAN_TSR_TME1|CAN_TSR_TME2));
 if(inject_register_fault){auto f=inject_register_fault;inject_register_fault=nullptr;f();}
 if(inject_free){auto f=inject_free;inject_free=nullptr;if(irq_mask)deferred_preemption=f;else f();}return value;
}
void set_bit(uint32_t& reg,uint32_t value){reg|=value;if(value==CAN_TI0R_TXRQ)for(int i=0;i<3;++i)if(&reg==&regs.sTxMailBox[i].TIR)regs.TSR&=~(CAN_TSR_TME0<<i);}
#define assert_param(x) ((void)0)
#define SET_BIT(r,v) set_bit(r,v)
#define WRITE_REG(r,v) ((r)=(v))
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef*,uint32_t n){return fifo[n];}
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef*,uint32_t n,CAN_RxHeaderTypeDef* header,uint8_t* buf){++rx_reads;if(!fifo[n])return HAL_ERROR;--fifo[n];*header=CAN_RxHeaderTypeDef{};std::memset(buf,0,8);return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef* h){require(!irq_mask,"HAL Stop masks motor IRQs");++stop_calls;reinit_active=true;if(during_reinit)during_reinit();if(fail_stop){h->ErrorCode|=HAL_CAN_ERROR_TIMEOUT;return HAL_ERROR;}h->State=HAL_CAN_STATE_READY;return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_DeInit(CAN_HandleTypeDef* h){HAL_CAN_Stop(h);can_clock_enabled=false;h->State=HAL_CAN_STATE_RESET;h->ErrorCode=0;return HAL_OK;}
constexpr int CAN1_TX_IRQn=0,CAN1_RX0_IRQn=1,CAN1_RX1_IRQn=2,CAN1_SCE_IRQn=3;
void HAL_NVIC_DisableIRQ(int irq){can_irqs_disabled|=1u<<irq;}
void HAL_NVIC_ClearPendingIRQ(int irq){pending_irq_cleared|=1u<<irq;}
void rcc_force_reset(){require(!irq_mask,"RCC reset masks motor IRQs");require(can_clock_enabled,"reset with CAN clock disabled");require(can_irqs_disabled==15,"CAN IRQ may run during RCC reset");++rcc_reset_count;fifo[0]=fifo[1]=0;regs.TSR=CAN_TSR_TME0|CAN_TSR_TME1|CAN_TSR_TME2;}
#define __HAL_RCC_CAN1_FORCE_RESET() rcc_force_reset()
#define __HAL_RCC_CAN1_RELEASE_RESET() ((void)0)
#define __DSB() ((void)0)
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef* h){require(!irq_mask,"HAL Init masks motor IRQs");++init_calls;can_clock_enabled=true;can_irqs_disabled=0;if(fail_inits){--fail_inits;h->ErrorCode|=HAL_CAN_ERROR_TIMEOUT;return HAL_ERROR;}h->State=HAL_CAN_STATE_READY;h->ErrorCode=0;return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef* h){require(!irq_mask,"HAL Start masks motor IRQs");h->State=HAL_CAN_STATE_LISTENING;reinit_active=false;return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef*,uint32_t){regs.TSR=CAN_TSR_TME0|CAN_TSR_TME1|CAN_TSR_TME2;return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef*,uint32_t){return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef*,uint32_t){return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef*,CAN_FilterTypeDef*){return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef* h){h->ErrorCode=0;++reset_calls;return HAL_OK;}
using osThreadId=void*;using StackType_t=uint32_t;
constexpr int sem_can=1,osPriorityNormal=0;
uint32_t osKernelSysTick(){return 100;}
int osSemaphoreRelease(int){return 0;}
void boundary_wait(){require(!irq_mask,"RTOS wait with motor IRQs masked");error_reads=0;if(++waits>=uint32_t(wait_budget))throw StopLoop{};}
int osSemaphoreWait(int,int timeout){require(timeout>0,"unbounded wait");boundary_wait();return 0;}
void osDelay(int delay){require(delay>0 && delay<=100,"missing or excessive backoff");++delay_count;boundary_wait();}
#define osThreadDef(name,func,priority,n,stack) int name=0
#define osThread(x) (&x)
osThreadId osThreadCreate(int*,void*){return reinterpret_cast<void*>(1);}
void NVIC_SystemReset(){++reset_calls;}
namespace ODriveIntf {struct CanIntf {enum Protocol{PROTOCOL_SIMPLE=0};enum Error{ERROR_NONE=0,ERROR_DUPLICATE_CAN_IDS=1};};}
inline ODriveIntf::CanIntf::Error& operator|=(ODriveIntf::CanIntf::Error& a,ODriveIntf::CanIntf::Error b){a=static_cast<ODriveIntf::CanIntf::Error>(int(a)|int(b));return a;}
class Axis;
struct Motor {Axis* axis_=nullptr;struct{Motor* timer=nullptr;}hw_config_;enum {ERROR_NONE=0,ARMED_STATE_DISARMED=0,ARMED_STATE_WAITING_FOR_TIMINGS=1};uint32_t error_=0;int armed_state_=1;bool disarmed=false;struct{float Iq_setpoint=0,Iq_measured=0;} current_control_;};
bool safety_critical_disarm_motor_pwm(Motor& m);
void safety_critical_arm_motor_pwm(Motor& m);
bool brake_resistor_armed=true;
#define __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(timer) ((timer)->disarmed=true)
struct Controller {enum ControlMode{CONTROL_MODE_POSITION_CONTROL=3};enum InputMode{INPUT_MODE_PASSTHROUGH=1};enum{ERROR_NONE=0};uint32_t error_=0;float input_pos_=0,input_vel_=0,input_torque_=0;struct{ControlMode control_mode=CONTROL_MODE_POSITION_CONTROL;InputMode input_mode=INPUT_MODE_PASSTHROUGH;float vel_limit=0,inertia=0;}config_;void input_pos_updated(){}void start_anticogging_calibration(){};};
struct Encoder {enum{ERROR_NONE=0};uint32_t error_=0;float spi_error_rate_=0,pos_estimate_=3,vel_estimate_=0;int shadow_count_=0,count_in_cpr_=0;};
struct SensorlessEstimator {enum{ERROR_NONE=0};uint32_t error_=0;float pll_pos_=0,vel_estimate_=0;};
struct Axis {
 enum AxisState{AXIS_STATE_UNDEFINED=0,AXIS_STATE_IDLE=1,AXIS_STATE_CLOSED_LOOP_CONTROL=8};
 enum{ERROR_NONE=0,ERROR_ESTOP_REQUESTED=0x4000,ERROR_WATCHDOG_TIMER_EXPIRED=0x800};
 struct{uint32_t can_node_id=13,can_heartbeat_rate_ms=20;bool can_node_id_extended=false,enable_watchdog=true;}config_;
 uint32_t error_=0,last_heartbeat_=0,feed_count=0,watchdog_current_value_=3,watchdog_reset_value_=3;
 AxisState current_state_=AXIS_STATE_IDLE,requested_state_=AXIS_STATE_UNDEFINED;
 bool can_recovery_latched_=false,can_recovery_in_progress_=false;
 Motor motor_;Controller controller_;Encoder encoder_;SensorlessEstimator sensorless_estimator_;struct{struct{float vel_limit=0,accel_limit=0,decel_limit=0;}config_;}trap_traj_;
 uint32_t get_watchdog_reset(){++feed_count;return watchdog_reset_value_;}
 void watchdog_feed();
 bool watchdog_check();
 /* ACTUAL_AXIS_METHODS */
};
constexpr int AXIS_COUNT=2;
Axis a0,a1;Axis* axes[2]={&a0,&a1};
class ODriveCAN;ODriveCAN* odCAN=nullptr;
float vbus_voltage=48;
