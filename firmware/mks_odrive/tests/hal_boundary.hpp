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
#include <array>
using HAL_StatusTypeDef=int;
constexpr int HAL_OK=0,HAL_ERROR=1,HAL_CAN_STATE_RESET=0,HAL_CAN_STATE_READY=1,HAL_CAN_STATE_LISTENING=2;
constexpr uint32_t RESET=0;
/* REAL_CAN_DEFINES */
enum FunctionalState {DISABLE=0,ENABLE=1};
struct Mailbox {uint32_t TIR=0,TDTR=0,TDHR=0,TDLR=0;};
struct Registers {uint32_t TSR=CAN_TSR_TME0;uint32_t IER=0,ESR=0,MSR=0,RF0R=0,RF1R=0;Mailbox sTxMailBox[3];} regs;
Registers* const CAN1=&regs;
struct CAN_HandleTypeDef {int State=HAL_CAN_STATE_LISTENING;uint32_t ErrorCode=0;Registers* Instance=&regs;struct {uint32_t Prescaler=4;} Init;};
#define READ_REG(r) (r)
#define CLEAR_BIT(r,b) ((r)&=~(b))
void can_clear_flag(CAN_HandleTypeDef* hc,uint32_t flag){const uint32_t bit=1u<<(flag&CAN_FLAG_MASK);
 switch(flag>>8U){case 5:hc->Instance->TSR&=~bit;break;case 2:hc->Instance->RF0R&=~bit;break;
  case 4:hc->Instance->RF1R&=~bit;break;case 1:hc->Instance->MSR&=~bit;break;default:break;}}
#define __HAL_CAN_CLEAR_FLAG(h,f) can_clear_flag((h),(f))
struct RxFrame{uint32_t id;bool ext,rtr;};
std::vector<RxFrame> rx_fifo0;
struct CAN_TxHeaderTypeDef {uint32_t StdId,ExtId,IDE,RTR,DLC;FunctionalState TransmitGlobalTime;};
struct CAN_RxHeaderTypeDef {uint32_t StdId=0,ExtId=0,IDE=0,RTR=0,DLC=8;};
struct CAN_FilterTypeDef {uint32_t FilterActivation=0,FilterBank=0,FilterFIFOAssignment=0,FilterIdHigh=0,FilterIdLow=0,FilterMaskIdHigh=0,FilterMaskIdLow=0,FilterMode=0,FilterScale=0,SlaveStartFilterBank=14;};  // vendor leaves this uninitialized; 14 is the bxCAN reset value
struct StopLoop{};
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
uint32_t irq_mask=0, delay_count=0, waits=0, error_reads=0, stop_calls=0, init_calls=0, reset_calls=0, rx_reads=0;
uint8_t observed_tx[8] = {}; uint32_t observed_tx_id=0; uint8_t observed_tx_len=0;
template<class Message> void observe_tx_attempt(const Message& msg){std::memcpy(observed_tx,msg.buf,8);observed_tx_id=msg.id;observed_tx_len=msg.len;}
uint32_t fail_inits=0, fifo[2]={0,0}, can_irqs_disabled=0, rcc_reset_count=0, pending_irq_cleared=0;
uint32_t virtual_tick=100, filter_config_calls=0;int sem_releases=0;bool fail_filter=false;
constexpr uint32_t osKernelSysTickFrequency=1000;
std::function<void()> on_wait, meas_hook;
uint32_t calibration_ms=0;
CAN_HandleTypeDef* irq_handle=nullptr;
void HAL_CAN_IRQHandler(CAN_HandleTypeDef*);
bool can_clock_enabled=true;
int wait_budget=1;
bool reinit_active=false, fail_stop=false;
std::function<void()> deferred_preemption, inject_free, during_reinit, inject_register_fault, on_irq_enable;
uint32_t cpu_enter_critical(){uint32_t old=irq_mask;irq_mask=1;return old;}
void cpu_exit_critical(uint32_t old){irq_mask=old;if(!irq_mask && deferred_preemption){auto f=deferred_preemption;deferred_preemption=nullptr;f();}if(!irq_mask && on_irq_enable){auto f=on_irq_enable;f();}}
std::function<void()> on_error_read, after_start;  // ISR right after an ErrorCode read; event after HAL Start
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef* h){if(++error_reads>100)throw std::runtime_error("busy spin: more than 100 HAL error polls without a bounded wait");
 const uint32_t value=h->ErrorCode;
 if(on_error_read){auto f=on_error_read;on_error_read=nullptr;if(irq_mask)deferred_preemption=f;else f();}
 return value;}
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
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef*,uint32_t n,CAN_RxHeaderTypeDef* header,uint8_t* buf){
 ++rx_reads;if(!fifo[n])return HAL_ERROR;--fifo[n];*header=CAN_RxHeaderTypeDef{};std::memset(buf,0,8);
 if(n==0 && !rx_fifo0.empty()){RxFrame f=rx_fifo0.front();rx_fifo0.erase(rx_fifo0.begin());
  header->IDE=f.ext?CAN_ID_EXT:CAN_ID_STD;header->StdId=f.id;header->ExtId=f.id;header->RTR=f.rtr?CAN_RTR_REMOTE:CAN_RTR_DATA;}
 regs.RF0R=(regs.RF0R&~3u)|(fifo[0]&3u);return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef* h){require(!irq_mask,"HAL Stop masks motor IRQs");++stop_calls;reinit_active=true;if(during_reinit)during_reinit();if(fail_stop){h->ErrorCode|=HAL_CAN_ERROR_TIMEOUT;return HAL_ERROR;}h->State=HAL_CAN_STATE_READY;return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_DeInit(CAN_HandleTypeDef* h){HAL_CAN_Stop(h);can_clock_enabled=false;h->State=HAL_CAN_STATE_RESET;h->ErrorCode=0;return HAL_OK;}
constexpr int CAN1_TX_IRQn=0,CAN1_RX0_IRQn=1,CAN1_RX1_IRQn=2,CAN1_SCE_IRQn=3;
void HAL_NVIC_DisableIRQ(int irq){can_irqs_disabled|=1u<<irq;}
void HAL_NVIC_ClearPendingIRQ(int irq){pending_irq_cleared|=1u<<irq;}
void rcc_force_reset(){require(!irq_mask,"RCC reset masks motor IRQs");require(can_clock_enabled,"reset with CAN clock disabled");require(can_irqs_disabled==15,"CAN IRQ may run during RCC reset");++rcc_reset_count;fifo[0]=fifo[1]=0;rx_fifo0.clear();regs.RF0R=regs.RF1R=regs.ESR=regs.MSR=regs.IER=0;regs.TSR=CAN_TSR_TME0|CAN_TSR_TME1|CAN_TSR_TME2;}
#define __HAL_RCC_CAN1_FORCE_RESET() rcc_force_reset()
#define __HAL_RCC_CAN1_RELEASE_RESET() ((void)0)
#define __DSB() ((void)0)
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef* h){require(!irq_mask,"HAL Init masks motor IRQs");++init_calls;can_clock_enabled=true;if(h->State==HAL_CAN_STATE_RESET)can_irqs_disabled=0;/* MspInit re-enables NVIC only from RESET */if(fail_inits){--fail_inits;h->ErrorCode|=HAL_CAN_ERROR_TIMEOUT;return HAL_ERROR;}h->State=HAL_CAN_STATE_READY;h->ErrorCode=0;return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef* h){require(!irq_mask,"HAL Start masks motor IRQs");h->State=HAL_CAN_STATE_LISTENING;reinit_active=false;if(after_start){auto f=after_start;f();}return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef*,uint32_t){regs.TSR=CAN_TSR_TME0|CAN_TSR_TME1|CAN_TSR_TME2;return HAL_OK;}
std::function<void()> after_activation;
int activations_before_failure=-1;  // >=0: that many activations succeed, then one fails
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef* hc,uint32_t it){
 if(activations_before_failure==0){activations_before_failure=-1;hc->ErrorCode|=HAL_CAN_ERROR_NOT_INITIALIZED;return HAL_ERROR;}
 if(activations_before_failure>0)--activations_before_failure;hc->Instance->IER|=it;if(after_activation){auto f=after_activation;f();}return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef* hc,uint32_t it){hc->Instance->IER&=~it;return HAL_OK;}
CAN_FilterTypeDef filters[28];
uint32_t filter_slave_start=14,blocked_can_irqs=0;
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef*,CAN_FilterTypeDef* f){++filter_config_calls;if(fail_filter)return HAL_ERROR;
 require(f->FilterScale==CAN_FILTERSCALE_32BIT,"16-bit filter scale is not modeled");filter_slave_start=f->SlaveStartFilterBank;
 require(f->FilterBank<28,"filter bank out of range");filters[f->FilterBank]=*f;return HAL_OK;}
// bxCAN filter semantics: CAN1 owns banks below SlaveStartFilterBank; mask or list mode.
int filter_target(const RxFrame& f){uint32_t word=f.ext?((f.id<<3)|4u):(f.id<<21);if(f.rtr)word|=2u;
 for(uint32_t i=0;i<28 && i<filter_slave_start;++i){const auto& b=filters[i];if(!b.FilterActivation)continue;
  const uint32_t id=(b.FilterIdHigh<<16)|b.FilterIdLow,mask=(b.FilterMaskIdHigh<<16)|b.FilterMaskIdLow;
  const bool hit=b.FilterMode==CAN_FILTERMODE_IDLIST?(word==id||word==mask):((word^id)&mask)==0;
  if(hit)return b.FilterFIFOAssignment==CAN_RX_FIFO0?0:1;}return -1;}
bool filter_accepts(const RxFrame& f){return filter_target(f)>=0;}
bool deliver(uint32_t id,bool ext=false,bool rtr=false){const RxFrame f{id,ext,rtr};const int target=filter_target(f);if(target<0)return false;
 if(target==1){if(fifo[1]<3)++fifo[1];else regs.RF1R|=CAN_RF1R_FOVR1;return true;}
 if(fifo[0]<3){rx_fifo0.push_back(f);++fifo[0];}else{if(!rx_fifo0.empty())rx_fifo0.back()=f;regs.RF0R|=CAN_RF0R_FOVR0;}
 regs.RF0R=(regs.RF0R&~3u)|(fifo[0]&3u);return true;}
void fire_can_irq(){require(!irq_mask,"CAN IRQ fired inside a critical section");
 if(can_irqs_disabled){++blocked_can_irqs;return;}  // NVIC-disabled CAN vectors do not run
 HAL_CAN_IRQHandler(irq_handle);}
void inject_esr(uint32_t esr){regs.ESR=esr;regs.MSR|=CAN_MSR_ERRI;}
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef* h){h->ErrorCode=0;++reset_calls;return HAL_OK;}
using osThreadId=void*;using StackType_t=uint32_t;
constexpr int sem_can=1,osPriorityNormal=0;
uint32_t osKernelSysTick(){return virtual_tick;}
// sem_can is a binary semaphore: a pending release makes the next wait return without blocking.
int sem_tokens=0;uint32_t sem_immediate=0;
int osSemaphoreRelease(int){++sem_releases;sem_tokens=1;return 0;}
void boundary_wait(){require(!irq_mask,"RTOS wait with motor IRQs masked");error_reads=0;if(on_wait){auto f=on_wait;f();}if(++waits>=uint32_t(wait_budget))throw StopLoop{};}
int osSemaphoreWait(int,int timeout){require(timeout>0,"unbounded wait");if(sem_tokens){sem_tokens=0;++sem_immediate;return 0;}boundary_wait();return 0;}
void osDelay(int delay){require(delay>0 && delay<=100,"missing or excessive backoff");++delay_count;boundary_wait();}
#define osThreadDef(name,func,priority,n,stack) int name=0
#define osThread(x) (&x)
osThreadId osThreadCreate(int*,void*){return reinterpret_cast<void*>(1);}
void NVIC_SystemReset(){++reset_calls;}
namespace ODriveIntf {struct CanIntf {enum Protocol{PROTOCOL_SIMPLE=0};enum Error{ERROR_NONE=0,ERROR_DUPLICATE_CAN_IDS=1};};}
inline ODriveIntf::CanIntf::Error& operator|=(ODriveIntf::CanIntf::Error& a,ODriveIntf::CanIntf::Error b){a=static_cast<ODriveIntf::CanIntf::Error>(int(a)|int(b));return a;}
class Axis;
struct Motor {Axis* axis_=nullptr;struct{Motor* timer=nullptr;}hw_config_;enum {ERROR_NONE=0,ARMED_STATE_DISARMED=0,ARMED_STATE_WAITING_FOR_TIMINGS=1};uint32_t error_=0;int armed_state_=1;bool disarmed=false;struct{float Iq_setpoint=0,Iq_measured=0;} current_control_;bool is_calibrated_=true;struct{int direction=1;int pole_pairs=10;}config_;bool arm();bool run_calibration();bool update(float,float,float){return true;}};
bool safety_critical_disarm_motor_pwm(Motor& m);
void safety_critical_arm_motor_pwm(Motor& m);
void advance_calibration();
inline bool Motor::arm(){safety_critical_arm_motor_pwm(*this);return true;}
inline bool Motor::run_calibration(){advance_calibration();return true;}
bool brake_resistor_armed=true;
#define __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(timer) ((timer)->disarmed=true)
struct Controller {enum ControlMode{CONTROL_MODE_POSITION_CONTROL=3};enum InputMode{INPUT_MODE_PASSTHROUGH=1};enum{ERROR_NONE=0};uint32_t error_=0;float input_pos_=0,input_vel_=0,input_torque_=0;struct{ControlMode control_mode=CONTROL_MODE_POSITION_CONTROL;InputMode input_mode=INPUT_MODE_PASSTHROUGH;float vel_limit=0,inertia=0;int load_encoder_axis=0;bool circular_setpoints=false;}config_;float pos_setpoint_=0,vel_setpoint_=0,vel_integrator_torque_=0;float* pos_estimate_linear_src_=nullptr;float* pos_estimate_circular_src_=nullptr;bool select_encoder(int){return true;}bool update(float* t){*t=0;return true;}void reset(){}void input_pos_updated(){}void start_anticogging_calibration(){};};
struct Encoder {enum{ERROR_NONE=0};uint32_t error_=0;float spi_error_rate_=0,pos_estimate_=3,vel_estimate_=0;int shadow_count_=0,count_in_cpr_=0;bool is_ready_=true;float phase_=0;struct{bool use_index=false,idx_search_unidirectional=false;}config_;bool run_index_search(){return true;}bool run_direction_find(){return true;}bool run_offset_calibration(){advance_calibration();return true;}};
struct SensorlessEstimator {enum{ERROR_NONE=0};uint32_t error_=0;float pll_pos_=0,vel_estimate_=0;};
struct Axis {
 enum AxisState{AXIS_STATE_UNDEFINED=0,AXIS_STATE_IDLE=1,AXIS_STATE_STARTUP_SEQUENCE=2,AXIS_STATE_FULL_CALIBRATION_SEQUENCE=3,AXIS_STATE_MOTOR_CALIBRATION=4,AXIS_STATE_SENSORLESS_CONTROL=5,AXIS_STATE_ENCODER_INDEX_SEARCH=6,AXIS_STATE_ENCODER_OFFSET_CALIBRATION=7,AXIS_STATE_CLOSED_LOOP_CONTROL=8,AXIS_STATE_LOCKIN_SPIN=9,AXIS_STATE_ENCODER_DIR_FIND=10,AXIS_STATE_HOMING=11};
 struct LockinConfig_t{float vel=40.0f;};
 enum{ERROR_NONE=0,ERROR_INVALID_STATE=0x1,ERROR_CURRENT_MEASUREMENT_TIMEOUT=0x8,ERROR_BRAKE_RESISTOR_DISARMED=0x10,ERROR_MOTOR_DISARMED=0x20,ERROR_CONTROLLER_FAILED=0x200,ERROR_WATCHDOG_TIMER_EXPIRED=0x800,ERROR_ESTOP_REQUESTED=0x4000};
 struct{uint32_t can_node_id=13,can_heartbeat_rate_ms=20;bool can_node_id_extended=false,enable_watchdog=true;bool startup_motor_calibration=false,startup_encoder_index_search=false,startup_encoder_offset_calibration=false,startup_homing=false,startup_closed_loop_control=false,startup_sensorless_control=false,enable_step_dir=false,step_dir_always_on=false;LockinConfig_t general_lockin,sensorless_ramp;}config_;
 uint32_t error_=0,last_heartbeat_=0,feed_count=0,watchdog_current_value_=3,watchdog_reset_value_=3;
 std::array<AxisState,10> task_chain_{{AXIS_STATE_IDLE}};AxisState& current_state_=task_chain_.front();AxisState requested_state_=AXIS_STATE_UNDEFINED;uint32_t loop_counter_=0;
 // Mirrors the real error exits: axis errors end the control loop through check_for_errors().
 bool do_checks(){if(!brake_resistor_armed)error_|=ERROR_BRAKE_RESISTOR_DISARMED;
  if(current_state_!=AXIS_STATE_IDLE && motor_.armed_state_==Motor::ARMED_STATE_DISARMED)error_|=ERROR_MOTOR_DISARMED;
  return check_for_errors();}
 bool do_updates();  // defined in scenarios.cpp after ODriveCAN: check_for_errors + heartbeat, like axis.cpp
 void update_brake_current(){}void set_step_dir_active(bool){}
 bool wait_for_current_meas(){error_reads=0;++virtual_tick;if(meas_hook){auto f=meas_hook;f();}return true;}
 bool run_homing(){return true;}bool run_lockin_spin(const LockinConfig_t&){return true;}bool run_sensorless_control_loop(){return true;}
 bool run_closed_loop_control_loop();bool run_idle_loop();void run_state_machine_loop();
 /* ACTUAL_CAN_AXIS_FIELDS */
 Motor motor_;Controller controller_;Encoder encoder_;SensorlessEstimator sensorless_estimator_;struct{struct{float vel_limit=0,accel_limit=0,decel_limit=0;}config_;}trap_traj_;
 uint32_t get_watchdog_reset(){++feed_count;return watchdog_reset_value_;}
 void watchdog_feed();
 bool watchdog_check();
 /* ACTUAL_AXIS_METHODS */
};
constexpr int AXIS_COUNT=2;
Axis a0,a1;Axis* axes[2]={&a0,&a1};
void advance_calibration(){for(uint32_t n=0;n<calibration_ms;++n){++virtual_tick;if(meas_hook){auto f=meas_hook;f();}}}
class ODriveCAN;ODriveCAN* odCAN=nullptr;
float vbus_voltage=48;
