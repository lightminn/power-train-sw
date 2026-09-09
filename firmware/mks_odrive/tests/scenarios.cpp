void run_server(ODriveCAN& can){try{can.can_server_thread();}catch(const StopLoop&){};}
can_Message_t msg(uint32_t command,bool rtr=false,uint8_t len=8,float first=1,float second=0){can_Message_t m;m.id=(13<<5)|command;m.rtr=rtr;m.len=len;std::memcpy(m.buf,&first,4);std::memcpy(m.buf+4,&second,4);return m;}
int main(int argc,char** argv){try{
 require(argc==2,"case required");std::string name=argv[1];
 CAN_HandleTypeDef h;ODriveCAN::Config_t cfg;ODriveCAN can(cfg,&h);odCAN=&can;a1.config_.can_node_id=14;can.thread_id_valid_=true;
 for(auto axis:axes){axis->motor_.axis_=axis;axis->motor_.hw_config_.timer=&axis->motor_;}
 can_Message_t reply;
 if(name=="tx_race"){
  inject_free=[&](){can_Message_t heartbeat;can.write(heartbeat);};can.write(reply);
  require(h.ErrorCode==0,"last mailbox preemption poisoned HAL PARAM");regs.TSR=CAN_TSR_TME0;require(can.write(reply)!=uint32_t(-1),"follow-up send stuck");
 }else if(name=="mailbox_full"){
  regs.TSR=0;require(can.write(reply)==0,"full mailbox must be bounded drop");require(h.ErrorCode==0,"full mailbox poisoned HAL");
 }else if(name=="hal_add_failure"){
  inject_register_fault=[&](){h.State=HAL_CAN_STATE_RESET;};can.write(reply);
#if PATCHED
  require(can.tx_error_count_==1 && can.last_hal_error_==HAL_CAN_ERROR_NOT_INITIALIZED,"HAL failure status not recorded");require(can.write(reply)==0,"error gate permits TX");
#else
  require(h.ErrorCode==0,"HAL failure left sticky error");
#endif
 }else if(name=="server_error" || name=="server_retry_failure"){
  h.ErrorCode=HAL_CAN_ERROR_PARAM;fail_inits=name=="server_retry_failure"?1:0;wait_budget=3;run_server(can);
  require(delay_count>0,"error server failed to yield");require(init_calls>0,"API error not recovered");
  require(h.ErrorCode==0,"retry did not recover");
#if PATCHED
  require(can.recovery_count_==(name=="server_retry_failure"?2u:1u),"unnecessary recovery retry");
  require(can.recovery_failure_count_==(name=="server_retry_failure"?1u:0u),"incorrect recovery failure count");
#endif
  require(a0.motor_.disarmed && a1.motor_.disarmed,"recovery missed both motor disarms");
 }else if(name=="reinit_exclusion"){
  fifo[0]=3;bool checked=false;during_reinit=[&](){checked=true;require(can.write(reply)==0,"TX permitted during HAL restart");can_Message_t rx;require(!can.read(rx),"RX permitted during HAL restart");require(irq_mask==0,"restart under critical section");};
  can.set_baud_rate(500000);require(stop_calls==0,"USB setter synchronously touches HAL");wait_budget=2;run_server(can);require(checked,"restart never executed");require(fifo[0]==0,"stale RX survived recovery");
#if PATCHED
  require(rcc_reset_count>0 && pending_irq_cleared==15,"missing IRQ-excluded RCC reset");
#endif
 }else if(name=="startup"){
  can.thread_id_valid_=false;can.start_can_server();require(!a0.error_ && !a1.error_,"startup falsely latches estop");
 }else if(name=="recovery_latch"){
  a0.current_state_=Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;a0.controller_.input_vel_=7;a0.controller_.input_torque_=2;
  h.ErrorCode=HAL_CAN_ERROR_PARAM;wait_budget=2;run_server(can);
  require(a0.error_&Axis::ERROR_ESTOP_REQUESTED,"missing retained ESTOP");require(a0.requested_state_==Axis::AXIS_STATE_IDLE,"recovery did not request IDLE");
  require(a0.controller_.input_vel_==0 && a0.controller_.input_torque_==0 && a0.controller_.input_pos_==a0.encoder_.pos_estimate_,"stale setpoint retained");
  auto m=msg(CANSimple::MSG_SET_INPUT_VEL);CANSimple::handle_can_message(m);require(a0.controller_.input_vel_==0,"latched axis accepts stale motion");
  a0.current_state_=Axis::AXIS_STATE_IDLE;a0.requested_state_=Axis::AXIS_STATE_UNDEFINED;a0.clear_errors();require(!a0.error_,"explicit clear after IDLE fails");CANSimple::handle_can_message(m);require(a0.controller_.input_vel_==1,"fresh control after explicit clear rejected");
 }else if(name=="clear_before_idle"){
  a0.current_state_=Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;a0.error_=Axis::ERROR_ESTOP_REQUESTED;a0.can_recovery_latched_=true;a0.clear_errors();require(a0.error_&Axis::ERROR_ESTOP_REQUESTED,"clear erased latch before observed IDLE");
 }else if(name=="telemetry_watchdog" || name=="unknown_watchdog"){
  for(uint32_t c: name=="telemetry_watchdog"?std::vector<uint32_t>{1,3,4,5,9,10,20,21,23}:std::vector<uint32_t>{0,8,25,31}){auto m=msg(c,true);CANSimple::handle_can_message(m);}require(a0.feed_count==0,"non-control traffic fed watchdog");
 }else if(name=="rtr_motion" || name=="short_motion" || name=="nonfinite_motion"){
  for(uint32_t c:{12,13,14}){auto m=msg(c,name=="rtr_motion",name=="short_motion"?3:8,name=="nonfinite_motion"?std::numeric_limits<float>::quiet_NaN():1);CANSimple::handle_can_message(m);}
  require(!a0.feed_count && a0.controller_.input_pos_==0 && a0.controller_.input_vel_==0 && a0.controller_.input_torque_==0,"invalid motion mutated state or fed watchdog");
  if(name=="nonfinite_motion"){auto m=msg(13,false,8,1,std::numeric_limits<float>::infinity());CANSimple::handle_can_message(m);require(!a0.feed_count && a0.controller_.input_vel_==0,"nonfinite second field partially applied");}
 }else if(name=="valid_control"){
  for(auto c:{12,13,14}){auto m=msg(c,false,c==14?4:8);CANSimple::handle_can_message(m);}require(a0.feed_count==3,"accepted finite setpoints must feed watchdog");
 }else if(name=="rtr_state_clear" || name=="short_state"){
  a0.error_=Axis::ERROR_ESTOP_REQUESTED;auto m=msg(7,name=="rtr_state_clear",name=="short_state"?1:8);int32_t state=8;std::memcpy(m.buf,&state,4);CANSimple::handle_can_message(m);require(a0.requested_state_==Axis::AXIS_STATE_UNDEFINED,"invalid state command armed axis");m=msg(24,true,0);CANSimple::handle_can_message(m);require(a0.error_==Axis::ERROR_ESTOP_REQUESTED,"RTR clears errors");
 }else if(name=="watchdog_expiry"){
  a0.current_state_=Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
  for(int n=0;n<5;++n){auto m=msg(9,true);CANSimple::handle_can_message(m);a0.watchdog_check();}
  require(a0.watchdog_current_value_==0 && (a0.error_&Axis::ERROR_WATCHDOG_TIMER_EXPIRED),"telemetry prevents watchdog expiry");
 }else if(name=="idle_watchdog"){
  a0.current_state_=Axis::AXIS_STATE_IDLE;a0.watchdog_current_value_=0;
  require(a0.watchdog_check() && a0.error_==0,"IDLE watchdog retrips and blocks neutral start");
 }else if(name=="idle_retains_expiry"){
  a0.current_state_=Axis::AXIS_STATE_IDLE;a0.error_=Axis::ERROR_WATCHDOG_TIMER_EXPIRED;
  a0.watchdog_check();require(a0.error_&Axis::ERROR_WATCHDOG_TIMER_EXPIRED,"IDLE auto-cleared prior watchdog error");
 }else if(name=="automatic_rearm"){
  a0.can_recovery_latched_=true;a0.motor_.armed_state_=Motor::ARMED_STATE_DISARMED;
  safety_critical_arm_motor_pwm(a0.motor_);require(a0.motor_.armed_state_==Motor::ARMED_STATE_DISARMED,"routine motor arm bypasses recovery latch");
  a0.error_=0;require(!a0.check_for_errors(),"direct USB error write bypasses recovery latch");
 }else if(name=="clear_during_recovery"){
  a0.can_recovery_latched_=true;a0.can_recovery_in_progress_=true;a0.error_=Axis::ERROR_ESTOP_REQUESTED;
  a0.clear_errors();require(a0.error_&Axis::ERROR_ESTOP_REQUESTED,"clear during restart erases safety latch");
 }else if(name=="irq_error_retained"){
  h.ErrorCode=HAL_CAN_ERROR_PARAM;HAL_CAN_ErrorCallback(&h);
  require(h.ErrorCode==HAL_CAN_ERROR_PARAM,"IRQ callback cleared actual error before server recorded it");
#if PATCHED
  require(can.last_hal_error_==HAL_CAN_ERROR_PARAM && can.hal_error_history_==HAL_CAN_ERROR_PARAM,"IRQ diagnostics missing");
#endif
 }else if(name=="baud_request_coalescing"){
  can.set_baud_rate(125000);can.set_baud_rate(1000000);can.set_baud_rate(123);
  require(stop_calls==0,"setter touched HAL");wait_budget=2;run_server(can);
  require(cfg.baud_rate==1000000 && h.Init.Prescaler==2,"last valid pending baud not applied");
 }else if(name=="persistent_recovery_failure"){
  h.ErrorCode=HAL_CAN_ERROR_PARAM;fail_inits=100;wait_budget=3;run_server(can);
  require(delay_count==3 && init_calls==3,"persistent failure does not back off once per bounded attempt");
  require(a0.motor_.disarmed && a1.motor_.disarmed,"persistent failure lost disarm");
#if PATCHED
  require(can.reinitializing_ && a0.can_recovery_in_progress_,"failure opened I/O or clear gate");
  require(can.write(reply)==0,"TX permitted after failed recovery");
#endif
 }else if(name=="stop_error_recorded"){
  fail_stop=true;can.set_baud_rate(500000);wait_budget=2;run_server(can);
#if PATCHED
  require(can.hal_error_history_&HAL_CAN_ERROR_TIMEOUT,"HAL Stop failure erased by reset");
  require(rcc_reset_count==1 && !can.reinitializing_,"Stop failure did not recover through RCC reset");
#else
  require(false,"no retained Stop error diagnostic");
#endif
 }else throw std::runtime_error("unknown scenario");
 require(irq_mask==0,"critical section leaked");std::cout<<"ok\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}}
