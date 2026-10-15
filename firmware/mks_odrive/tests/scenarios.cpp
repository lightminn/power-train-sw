// Real Axis::do_updates() ends with check_for_errors() and a CAN heartbeat (axis.cpp).
bool Axis::do_updates(){bool ret=check_for_errors();if(odCAN)odCAN->send_heartbeat(this);return ret;}
void run_server(ODriveCAN& can){try{can.can_server_thread();}catch(const StopLoop&){};}
can_Message_t msg(uint32_t command,bool rtr=false,uint8_t len=8,float first=1,float second=0){can_Message_t m;m.id=(13<<5)|command;m.rtr=rtr;m.len=len;std::memcpy(m.buf,&first,4);std::memcpy(m.buf+4,&second,4);return m;}
void expect_recovery_frame(Axis& axis,uint8_t flags,uint8_t generation){
 std::memset(observed_tx,0xcc,8);CANSimple::send_heartbeat(&axis);
 require(observed_tx_id==((axis.config_.can_node_id<<5)|1) && observed_tx_len==8,"heartbeat frame identity changed");
 uint32_t error=0;std::memcpy(&error,observed_tx,4);
 require(error==axis.error_ && observed_tx[4]==uint8_t(axis.current_state_),"heartbeat standard prefix changed");
 require(observed_tx[5]==0xa3,"missing explicit patch3 heartbeat marker");
 require(observed_tx[6]==flags,"wrong recovery provenance flags");
 require(observed_tx[7]==generation,"wrong recovery generation");
}
void recover_once(ODriveCAN& can,CAN_HandleTypeDef& h){waits=0;wait_budget=1;h.ErrorCode=HAL_CAN_ERROR_PARAM;run_server(can);}
can_Message_t guarded_resume(uint8_t generation){auto m=msg(7);std::memset(m.buf,0,8);m.buf[0]=8;m.buf[4]=0xa3;m.buf[5]=generation;return m;}
void run_one(ODriveCAN& can){waits=0;wait_budget=2;run_server(can);}
int main(int argc,char** argv){try{
 require(argc==2,"case required");std::string name=argv[1];
 CAN_HandleTypeDef h;ODriveCAN::Config_t cfg;ODriveCAN can(cfg,&h);odCAN=&can;a1.config_.can_node_id=14;can.thread_id_valid_=true;
 for(auto axis:axes){axis->motor_.axis_=axis;axis->motor_.hw_config_.timer=&axis->motor_;axis->controller_.pos_estimate_linear_src_=&axis->encoder_.pos_estimate_;}
 can_Message_t reply;
 if(name.rfind("guard_",0)==0){
#if RECOVERY_METADATA
  a0.current_state_=Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;recover_once(can,h);
  a0.current_state_=Axis::AXIS_STATE_IDLE;a0.requested_state_=Axis::AXIS_STATE_UNDEFINED;
  a0.controller_.input_vel_=7;a0.controller_.input_torque_=2;a0.controller_.input_pos_=-3;
  auto m=guarded_resume(1);
  if(name=="guard_resume_generation")m.buf[5]=0;
  if(name=="guard_resume_new_generation"){recover_once(can,h);a0.current_state_=Axis::AXIS_STATE_IDLE;a0.requested_state_=Axis::AXIS_STATE_UNDEFINED;}
  if(name=="guard_resume_no_can")odCAN=nullptr;
  if(name=="guard_resume_new_axis_error")a0.error_|=0x40;
  if(name=="guard_resume_estop"){auto stop=msg(2,false,0);CANSimple::handle_can_message(stop);}
  if(name=="guard_resume_manual_reinit")can.set_baud_rate(500000);
  if(name=="guard_resume_pending_hal")can.record_hal_error(HAL_CAN_ERROR_PARAM);
  if(name=="guard_resume_hal_error")h.ErrorCode=HAL_CAN_ERROR_PARAM;
  if(name=="guard_resume_reinitializing")can.reinitializing_=true;
  if(name=="guard_resume_progress")a0.can_recovery_in_progress_=true;
  if(name=="guard_resume_nonidle")a0.current_state_=Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
  if(name=="guard_resume_pending_arm")a0.requested_state_=Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
  if(name=="guard_resume_motor_error")a0.motor_.error_=1;
  if(name=="guard_resume_encoder_error")a0.encoder_.error_=1;
  if(name=="guard_resume_controller_error")a0.controller_.error_=1;
  if(name=="guard_resume_sensorless_error")a0.sensorless_estimator_.error_=1;
  if(name=="guard_resume_not_latched")a0.can_recovery_latched_=false;
  if(name=="guard_resume_ineligible")a0.can_recovery_auto_resume_eligible_=false;
  if(name=="guard_resume_external_clear" || name=="guard_resume_wrong_magic" || name=="guard_resume_reserved" || name=="guard_resume_wrong_state")a0.clear_errors();
  if(name=="guard_resume_wrong_magic")m.buf[4]=0xa2;
  if(name=="guard_resume_reserved")m.buf[7]=1;
  if(name=="guard_resume_reserved_eligible")m.buf[6]=1;
  if(name=="guard_resume_wrong_state")m.buf[0]=1;
  if(name=="guard_resume_pending_idle")a0.requested_state_=Axis::AXIS_STATE_IDLE;
  if(name=="guard_manual_zero" || name=="guard_manual_empty"){
   m=msg(24);std::memset(m.buf,0,8);if(name=="guard_manual_empty")m.len=0;
   a0.error_|=0x40;a0.motor_.error_=1;a0.can_recovery_auto_resume_eligible_=false;
  }
  if(name=="guard_manual_state4" || name=="guard_manual_state8"){
   a0.clear_errors();m=msg(7);std::memset(m.buf,0,8);m.buf[0]=8;m.len=name=="guard_manual_state4"?4:8;
  }
  if(name=="guard_clear_nonzero" || name=="guard_clear_old_cas"){
   m=msg(24);std::memset(m.buf,0,8);m.buf[0]=name=="guard_clear_old_cas"?0xa3:1;m.buf[1]=1;
  }
  bool saw_commit=false;
  if(name=="guard_resume_atomic" || name=="guard_resume_late_fault")on_irq_enable=[&](){
   on_irq_enable=nullptr;saw_commit=true;
   require(a0.error_==0 && a0.requested_state_==Axis::AXIS_STATE_CLOSED_LOOP_CONTROL && !a0.can_recovery_latched_ && !a0.can_recovery_auto_resume_eligible_,"clear and resume were not committed atomically");
   require(a0.controller_.input_vel_==0 && a0.controller_.input_torque_==0 && a0.controller_.input_pos_==a0.encoder_.pos_estimate_,"resume commit retained stale input");
   if(name=="guard_resume_late_fault")a0.error_|=0x40;
  };
  const auto generation=a0.can_recovery_generation_;
  const auto error=a0.error_;const bool latch=a0.can_recovery_latched_,eligible=a0.can_recovery_auto_resume_eligible_;
  const auto requested=a0.requested_state_;const float vel=a0.controller_.input_vel_,torque=a0.controller_.input_torque_,pos=a0.controller_.input_pos_;
  const auto motor_error=a0.motor_.error_,encoder_error=a0.encoder_.error_,controller_error=a0.controller_.error_,sensorless_error=a0.sensorless_estimator_.error_;
  if(name=="guard_resume_invalid_frame"){
   for(auto length:{0,1,2,3,5,6,7,9}){m.len=length;CANSimple::handle_can_message(m);}
   m.len=8;m.rtr=true;CANSimple::handle_can_message(m);
  }else CANSimple::handle_can_message(m);
  if(name=="guard_manual_state4" || name=="guard_manual_state8" || name=="guard_resume_success" || name=="guard_resume_pending_idle" || name=="guard_resume_atomic" || name=="guard_manual_zero" || name=="guard_manual_empty" || name=="guard_resume_replay_after_fault"){
   require(a0.error_==0 && a0.motor_.error_==0 && !a0.can_recovery_latched_ && !a0.can_recovery_auto_resume_eligible_,"accepted resume did not clear safely");
   require(a0.controller_.input_vel_==0 && a0.controller_.input_torque_==0 && a0.controller_.input_pos_==a0.encoder_.pos_estimate_,"accepted resume retained stale input");
   require(a0.requested_state_==((name=="guard_manual_zero" || name=="guard_manual_empty")?requested:Axis::AXIS_STATE_CLOSED_LOOP_CONTROL),"resume did not atomically request CLOSED_LOOP");
   if(name=="guard_resume_atomic")require(saw_commit,"resume commit not observed");
   if(name=="guard_resume_replay_after_fault"){a0.error_|=0x40;a0.requested_state_=Axis::AXIS_STATE_IDLE;CANSimple::handle_can_message(m);require(a0.error_==0x40 && a0.requested_state_==Axis::AXIS_STATE_IDLE,"replayed resume erased a newer fault or requested arm");}
  }else if(name=="guard_resume_late_fault"){
   require(saw_commit && a0.error_==0x40 && !a0.check_for_errors(),"fault arriving at resume's IRQ boundary was masked");
  }else{
   require(a0.error_==error && a0.can_recovery_latched_==latch && a0.can_recovery_auto_resume_eligible_==eligible,"rejected resume mutated fault/provenance");
   require(a0.requested_state_==requested && a0.controller_.input_vel_==vel && a0.controller_.input_torque_==torque && a0.controller_.input_pos_==pos,"rejected resume mutated requested state or input");
   require(a0.motor_.error_==motor_error && a0.encoder_.error_==encoder_error && a0.controller_.error_==controller_error && a0.sensorless_estimator_.error_==sensorless_error,"rejected resume erased component error");
  }
  require(a0.can_recovery_generation_==generation && a0.feed_count==0,"resume changed generation or fed watchdog");
  require(a0.motor_.disarmed,"CAN callback directly rearmed PWM");
#else
  require(false,"guarded resume metadata missing");
#endif
 }else if(name.rfind("resume_",0)==0){
#if PATCHED
  a0.current_state_=Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
  if(name=="resume_heartbeat"){
   a0.error_=0x12345678;expect_recovery_frame(a0,0,0);
  }else if(name=="resume_patch_identity"){
#if PATCHED
   require(can.reliability_patch_==3,"patch3 identity missing");
#else
   require(false,"patch3 identity missing");
#endif
  }else if(name=="resume_healthy"){
   recover_once(can,h);expect_recovery_frame(a0,5,1);expect_recovery_frame(a1,1,1);
   require(a0.requested_state_==Axis::AXIS_STATE_IDLE && a0.motor_.disarmed,"eligible recovery autonomously arms");
  }else if(name=="resume_retries"){
   during_reinit=[&](){a0.current_state_=Axis::AXIS_STATE_IDLE;expect_recovery_frame(a0,7,1);};
   h.ErrorCode=HAL_CAN_ERROR_PARAM;fail_inits=2;wait_budget=3;run_server(can);
   expect_recovery_frame(a0,5,3);require(init_calls==3,"wrong recovery retry count");
  }else if(name=="resume_manual" || name=="resume_manual_then_hal" || name=="resume_hal_then_manual"){
   if(name=="resume_hal_then_manual")can.record_hal_error(HAL_CAN_ERROR_PARAM);
   can.set_baud_rate(500000);
   if(name=="resume_manual_then_hal")can.record_hal_error(HAL_CAN_ERROR_PARAM);
   waits=0;wait_budget=1;run_server(can);expect_recovery_frame(a0,1,1);
  }else if(name=="resume_manual_at_dispatch"){
   on_irq_enable=[&](){if(a0.can_recovery_latched_){on_irq_enable=nullptr;can.set_baud_rate(500000);}};
   during_reinit=[&](){expect_recovery_frame(a0,3,1);during_reinit=nullptr;};
   h.ErrorCode=HAL_CAN_ERROR_PARAM;wait_budget=2;run_server(can);expect_recovery_frame(a0,1,2);
  }else if(name=="resume_manual_during_hal"){
   a1.current_state_=Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
   bool requested=false;during_reinit=[&](){if(!requested){requested=true;can.set_baud_rate(500000);can.record_hal_error(HAL_CAN_ERROR_PARAM);expect_recovery_frame(a0,3,1);expect_recovery_frame(a1,3,1);}};
   h.ErrorCode=HAL_CAN_ERROR_PARAM;wait_budget=2;run_server(can);expect_recovery_frame(a0,1,2);expect_recovery_frame(a1,1,2);
  }else if(name=="resume_manual_revokes" || name=="resume_reinit_revokes" || name=="resume_invalid_baud"){
   a1.current_state_=Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
   recover_once(can,h);
   if(name=="resume_reinit_revokes")can.reinit_can();else can.set_baud_rate(name=="resume_invalid_baud"?123:500000);
   expect_recovery_frame(a0,name=="resume_invalid_baud"?5:1,1);
   expect_recovery_frame(a1,name=="resume_invalid_baud"?5:1,1);
  }else if(name=="resume_estop_before" || name=="resume_estop_after"){
   if(name=="resume_estop_after")recover_once(can,h);
   auto m=msg(2,false,0);CANSimple::handle_can_message(m);
   if(name=="resume_estop_before")recover_once(can,h);
   expect_recovery_frame(a0,1,1);
  }else if(name=="resume_clear"){
   recover_once(can,h);a0.current_state_=Axis::AXIS_STATE_IDLE;a0.clear_errors();
   expect_recovery_frame(a0,0,1);require(a0.error_==0,"clear rejected after recovery");
  }else if(name=="resume_early_clear"){
   during_reinit=[&](){a0.clear_errors();expect_recovery_frame(a0,3,1);require(a0.error_==Axis::ERROR_ESTOP_REQUESTED,"early clear lost stop");};
   recover_once(can,h);expect_recovery_frame(a0,1,1);
  }else if(name=="resume_direct_latch"){
#if PATCHED
   a0.latch_can_fault();expect_recovery_frame(a0,3,1);
#else
   require(false,"recovery latch missing");
#endif
  }else if(name=="resume_pending_idle" || name=="resume_axis_error" || name=="resume_motor_error" || name=="resume_encoder_error" || name=="resume_controller_error" || name=="resume_sensorless_error"){
   if(name=="resume_pending_idle")a0.requested_state_=Axis::AXIS_STATE_IDLE;
   if(name=="resume_axis_error")a0.error_=Axis::ERROR_WATCHDOG_TIMER_EXPIRED;
   if(name=="resume_motor_error")a0.motor_.error_=1;
   if(name=="resume_encoder_error")a0.encoder_.error_=1;
   if(name=="resume_controller_error")a0.controller_.error_=1;
   if(name=="resume_sensorless_error")a0.sensorless_estimator_.error_=1;
   recover_once(can,h);expect_recovery_frame(a0,1,1);
  }else if(name=="resume_generation_wrap"){
   for(unsigned n=0;n<256;++n)recover_once(can,h);
   expect_recovery_frame(a0,5,0);
  }else if(name=="resume_reboot_defaults"){
   recover_once(can,h);Axis fresh;expect_recovery_frame(fresh,0,0);
  }else throw std::runtime_error("unknown resume scenario");
#else
  require(false,"patch3 recovery provenance missing");
#endif
 }else if(name=="tx_race"){
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
  require(name=="server_retry_failure"?delay_count>0:waits>0,"error server failed to yield");require(init_calls>0,"API error not recovered");
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
 }else if(name=="harness_irq_model"){
  irq_handle=&h;can.thread_id_valid_=false;can.start_can_server();
  regs.IER|=CAN_IT_RX_FIFO0_OVERRUN|CAN_IT_ERROR|CAN_IT_LAST_ERROR_CODE;
  for(int n=0;n<4;++n)deliver((13<<5)|9,false,true);
  require(fifo[0]==3 && (regs.RF0R&CAN_RF0R_FOVR0),"FIFO model did not overrun at depth 3");
  fire_can_irq();
  require(!(regs.RF0R&CAN_RF0R_FOVR0),"real IRQ handler did not clear FOVR0");
  regs.IER|=CAN_IT_ERROR|CAN_IT_LAST_ERROR_CODE;inject_esr(1u<<CAN_ESR_LEC_Pos);fire_can_irq();
  require(!(regs.ESR&CAN_ESR_LEC) && !(regs.MSR&CAN_MSR_ERRI),"real IRQ handler did not consume LEC/ERRI");
 }else if(name=="harness_state_machine"){
  a0.config_.enable_watchdog=false;uint32_t entered=0,left=0;
  meas_hook=[&](){
   if(virtual_tick==1010){auto m=msg(7,false,4);std::memset(m.buf,0,8);uint32_t s=8;std::memcpy(m.buf,&s,4);CANSimple::handle_can_message(m);}
   if(!entered && a0.current_state_==Axis::AXIS_STATE_CLOSED_LOOP_CONTROL)entered=virtual_tick;
   if(virtual_tick==1100)a0.requested_state_=Axis::AXIS_STATE_IDLE;
   if(entered && !left && a0.current_state_==Axis::AXIS_STATE_IDLE)left=virtual_tick;
   if(virtual_tick>=1200)throw StopLoop{};};
  virtual_tick=1000;try{a0.run_state_machine_loop();}catch(const StopLoop&){}meas_hook=nullptr;
  require(entered>1010 && left>1100 && a0.motor_.armed_state_==Motor::ARMED_STATE_DISARMED,"real dispatcher did not enter and leave CLOSED_LOOP with disarm");
 }else if(name=="harness_estop_exit"){
  a0.config_.enable_watchdog=false;bool left=false;
  meas_hook=[&](){
   if(virtual_tick==1010){auto m=msg(7,false,4);std::memset(m.buf,0,8);uint32_t s=8;std::memcpy(m.buf,&s,4);CANSimple::handle_can_message(m);}
   if(virtual_tick==1050){auto stop=msg(2,false,0);CANSimple::handle_can_message(stop);}
   if(virtual_tick>1050 && a0.current_state_==Axis::AXIS_STATE_IDLE)left=true;
   if(virtual_tick>=1200)throw StopLoop{};};
  virtual_tick=1000;try{a0.run_state_machine_loop();}catch(const StopLoop&){}meas_hook=nullptr;
  require(left && (a0.error_&Axis::ERROR_ESTOP_REQUESTED) && a0.motor_.armed_state_==Motor::ARMED_STATE_DISARMED,"axis error did not end CLOSED_LOOP through the real loop");
 }else if(name=="harness_irq_masked"){
  irq_handle=&h;can.thread_id_valid_=false;can.start_can_server();regs.IER|=CAN_IT_ERROR|CAN_IT_LAST_ERROR_CODE;
  can_irqs_disabled=15;inject_esr(1u<<CAN_ESR_LEC_Pos);fire_can_irq();
  require(blocked_can_irqs==1 && (regs.ESR&CAN_ESR_LEC),"NVIC-disabled CAN IRQ ran");
  can_irqs_disabled=0;fire_can_irq();require(!(regs.ESR&CAN_ESR_LEC),"re-enabled CAN IRQ did not run");
 }else if(name.rfind("info_",0)==0){
#if PATCH4
  irq_handle=&h;can.thread_id_valid_=false;can.start_can_server();
  a0.current_state_=Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
  if(name=="info_lec_no_latch"){
   for(uint32_t code=1;code<=6;++code){inject_esr(code<<CAN_ESR_LEC_Pos);fire_can_irq();
    require(!can.reinit_requested_,"IRQ requested reinit for a bus error");run_one(can);}
   require(can.lec_error_count_==6 && can.recovery_count_==0 && h.ErrorCode==0,"LEC was not informational");
  }else if(name=="info_overrun_no_latch"){
   for(int n=0;n<4;++n)deliver((13<<5)|9,false,true);
   fire_can_irq();require(!can.reinit_requested_,"IRQ requested reinit for overrun");run_one(can);
   require(can.rx_overrun_count_==1 && can.recovery_count_==0 && rx_reads==3,"overrun was not informational or FIFO not serviced");
  }else if(name=="info_status_no_latch"){
   inject_esr(CAN_ESR_EWGF|CAN_ESR_EPVF|CAN_ESR_BOFF);fire_can_irq();run_one(can);
   require(can.error_warning_count_==1 && can.error_passive_count_==1 && can.bus_off_count_==1 && can.recovery_count_==0,"status event stopped CAN");
  }else if(name=="info_mixed_param"){
   h.ErrorCode=HAL_CAN_ERROR_PARAM|HAL_CAN_ERROR_STF|HAL_CAN_ERROR_FOR|HAL_CAN_ERROR_RX_FOV0;run_one(can);
   require(can.lec_error_count_==1 && can.rx_overrun_count_==1,"mixed informational bits not counted");
   require(can.recovery_count_==1 && (can.hal_error_history_&HAL_CAN_ERROR_PARAM),"API error inside a mixed mask was swallowed");
  }else if(name=="info_state_not_listening"){
   h.State=HAL_CAN_STATE_READY;run_one(can);require(can.recovery_count_==1,"unexpected HAL state not recovered");
  }else if(name=="info_success_then_burst"){
   h.ErrorCode=HAL_CAN_ERROR_PARAM;bool burst=false;
   on_wait=[&](){if(!burst){burst=true;for(int n=0;n<4;++n)deliver((13<<5)|9,false,true);fire_can_irq();}};
   waits=0;wait_budget=3;run_server(can);on_wait=nullptr;
   require(can.recovery_count_==1 && can.rx_overrun_count_==1,"post-recovery burst caused another recovery");
   require(delay_count==0,"successful recovery still sleeps with RX unattended");
  }else if(name=="info_atomic_take"){
   // A new error IRQ right after the server snapshots ErrorCode must not double count the snapshot.
   h.ErrorCode=HAL_CAN_ERROR_STF|HAL_CAN_ERROR_RX_FOV0;
   on_error_read=[&](){h.ErrorCode|=HAL_CAN_ERROR_EPV;HAL_CAN_ErrorCallback(&h);};
   can.take_informational_errors();
   require(can.lec_error_count_==1 && can.rx_overrun_count_==1 && can.error_passive_count_==1 && h.ErrorCode==0,"informational read-count-clear was not atomic against the error IRQ");
  }else if(name=="info_repeat_recovery_yields"){
   // An API error right after every successful recovery: each back-to-back recovery must block.
   after_start=[&](){h.ErrorCode=HAL_CAN_ERROR_PARAM;};
   h.ErrorCode=HAL_CAN_ERROR_PARAM;waits=0;wait_budget=20;run_server(can);after_start=nullptr;
   require(can.recovery_count_>=3 && delay_count+1>=can.recovery_count_,"back-to-back recoveries did not block between attempts");
  }else if(name=="info_repeat_alternating_yields"){
   // Codex T2-3 path: recovery → (loop-end activation) RX IRQ leaves a token → the next plain
   // iteration's wait returns at once and its loop-end activation fails → another recovery.
   // Recoveries in that cycle still must block between attempts.
   irq_handle=&h;unsigned activations=0;
   after_start=[&](){activations=0;activations_before_failure=2;};
   after_activation=[&](){if(++activations==2){deliver((13<<5)|9,false,true);fire_can_irq();}};
   h.ErrorCode=HAL_CAN_ERROR_PARAM;waits=0;wait_budget=20;run_server(can);
   after_start=nullptr;after_activation=nullptr;activations_before_failure=-1;
   require(can.recovery_count_>=3 && delay_count+1>=can.recovery_count_,"alternating RX/API-failure path bypassed the recovery backoff");
  }else if(name=="info_storm"){
   for(uint32_t n=0;n<10000;++n){inject_esr(1u<<CAN_ESR_LEC_Pos);fire_can_irq();if(n%100==99)run_one(can);}
   run_one(can);
   require(can.lec_error_count_>0 && can.recovery_count_==0 && h.ErrorCode==0,"LEC storm stopped CAN or left errors");
   require((regs.IER&CAN_IT_ERROR)!=0,"error notifications not re-enabled after storm");
   const uint32_t before=can.lec_error_count_;inject_esr(1u<<CAN_ESR_LEC_Pos);fire_can_irq();
   require(can.lec_error_count_==before+1,"LEC after the storm was not observed (notifications stayed off)");
  }else throw std::runtime_error("unknown info scenario");
  if(name!="info_mixed_param" && name!="info_state_not_listening" && name!="info_success_then_burst" && name!="info_repeat_recovery_yields" && name!="info_repeat_alternating_yields")
   require(a0.error_==0 && !a0.can_recovery_latched_ && !a0.motor_.disarmed,"informational event stopped the motor");
#else
  require(false,"patch4 informational classification missing");
#endif
 }else if(name.rfind("filter_",0)==0 || name.rfind("txid_",0)==0){
#if PATCH4
  irq_handle=&h;
  if(name=="filter_ext_node"){a0.config_.can_node_id=0x40;a0.config_.can_node_id_extended=true;}
  if(name=="filter_out_of_range")a0.config_.can_node_id=77;
  can.thread_id_valid_=false;can.start_can_server();
  if(name=="filter_own_nodes"){
   require(deliver((13<<5)|13) && deliver((14<<5)|9,false,true),"own node frames rejected");
   require(!deliver((15<<5)|13) && !deliver((12<<5)|9,false,true),"foreign node accepted");
   require(!deliver((13<<5)|13,true) && !deliver(0x00000301,true),"extended or AK frame accepted by STD filter");
   // EXT ID whose top bits equal STD node 13: only the IDE bit separates it (mask bit 2).
   require(!deliver(13u<<23,true),"extended frame aliased into the STD node filter (IDE not compared)");
   require(fifo[0]==2,"FIFO holds foreign traffic");
   for(uint32_t cmd=0;cmd<32;++cmd)for(bool rtr:{false,true}){
    require(deliver((13<<5)|cmd,false,rtr) && deliver((14<<5)|cmd,false,rtr),"own node command rejected by filter");
    require(!deliver((15<<5)|cmd,false,rtr) && !deliver((13<<5)|cmd,true,rtr),"foreign node or EXT frame accepted");}
  }else if(name=="filter_ext_node"){
   require(deliver((0x40<<5)|9,true),"extended own node rejected");
   require(!deliver(((0x40<<5)|9)&0x7FF) && !deliver((13<<5)|9),"STD alias or old node accepted");
  }else if(name=="filter_out_of_range"){
   require(!filters[0].FilterActivation,"out-of-range node opened a filter bank");
   require(!deliver(((77<<5)|9)&0x7FF),"out-of-range node aliased into node13");
   require(deliver((14<<5)|9),"in-range peer axis lost its filter");
  }else if(name=="filter_can_node_change" || name=="filter_usb_node_change"){
   const uint32_t calls=filter_config_calls;
   if(name=="filter_can_node_change"){auto m=msg(6,false,4);std::memset(m.buf,0,8);uint32_t node=21;std::memcpy(m.buf,&node,4);CANSimple::handle_can_message(m);}
   else a0.config_.can_node_id=21;
   run_one(can);
   require(filter_config_calls>calls && deliver((21<<5)|9) && !deliver((13<<5)|9),"node change did not reprogram the filter");
  }else if(name=="filter_config_failure"){
   fail_filter=true;a0.config_.can_node_id=21;waits=0;wait_budget=3;run_server(can);
   require(can.recovery_count_>=1 && a0.can_recovery_latched_,"filter programming failure was not treated as a HAL failure");
  }else if(name=="txid_std_range" || name=="txid_ext_range"){
   const bool ext=name=="txid_ext_range";
   auto send_all=[&](){can_Message_t q=msg(9,true);CANSimple::send_heartbeat(&a0);
    CANSimple::get_motor_error_callback(&a0,q);CANSimple::get_encoder_error_callback(&a0,q);
    CANSimple::get_sensorless_error_callback(&a0,q);CANSimple::get_encoder_estimates_callback(&a0,q);
    CANSimple::get_sensorless_estimates_callback(&a0,q);CANSimple::get_encoder_count_callback(&a0,q);
    CANSimple::get_iq_callback(&a0,q);CANSimple::get_vbus_voltage_callback(&a0,q);};
   std::vector<uint32_t> rejected=ext?std::vector<uint32_t>{0x0800000Du}:std::vector<uint32_t>{0x40u,77u,0x0800000Du};
   for(uint32_t node:rejected){
    a0.config_.can_node_id=node;a0.config_.can_node_id_extended=ext;
    const uint32_t drops=can.tx_drop_count_;observed_tx_len=0xff;send_all();
    require(observed_tx_len==0xff,"out-of-range node reached CAN write");
    require(can.tx_drop_count_==drops+9,"each out-of-range TX site must count one drop");
   }
   if(ext)for(uint32_t node:{0x40u,77u}){
    a0.config_.can_node_id=node;a0.config_.can_node_id_extended=true;observed_tx_len=0xff;
    CANSimple::send_heartbeat(&a0);
    require(observed_tx_len==8 && observed_tx_id==((node<<5)|1),"valid extended node blocked or mis-encoded");
   }
  }else if(name=="txid_write_limit"){
   can_Message_t m;m.isExt=false;m.id=0x800;const uint32_t drops=can.tx_drop_count_;const uint32_t tsr=regs.TSR;
   require(can.write(m)==0 && can.tx_drop_count_==drops+1 && regs.TSR==tsr,"STD ID above 0x7FF reached HAL");
   m.isExt=true;m.id=0x20000000;require(can.write(m)==0 && can.tx_drop_count_==drops+2,"EXT ID above 29 bits reached HAL");
   m.id=0x1FFFFFFF;require(can.write(m)!=0,"maximum valid EXT ID rejected");
  }else throw std::runtime_error("unknown filter/txid scenario");
#else
  require(false,"patch4 node filter/TX range missing");
#endif
 }else throw std::runtime_error("unknown scenario");
 require(irq_mask==0,"critical section leaked");std::cout<<"ok\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}}
