import datetime,json,pathlib,time
from chassis.usb_session import motor_session, communication_snapshot, read_path
import odrive
serial='336A33523235'
result={'method':'USB property reads only; no setters or device methods', 'started_at':datetime.datetime.now(datetime.timezone.utc).isoformat(),'requested_serial':serial}
with motor_session('usb_can_read_only_diagnosis'):
    board=odrive.find_any(serial_number=serial,timeout=12)
    assert format(board.serial_number,'X')==serial
    result['communication']=communication_snapshot(board,serial=serial)
    print('COMMUNICATION '+json.dumps(result['communication']),flush=True)
    result['board']={p:read_path(board,p) for p in ('fw_version_unreleased','hw_version_major','hw_version_minor','hw_version_variant','vbus_voltage','ibus','error','can.error','can.n_restarts','can.n_tx','can.n_rx','system_stats.uptime','system_stats.min_heap_space','system_stats.stack_usage_can','system_stats.usb.rx_cnt','system_stats.usb.tx_cnt')}
    result['axes']={}
    for name in ('axis0','axis1'):
        axis=getattr(board,name)
        result['axes'][name]={p:read_path(axis,p) for p in ('current_state','requested_state','error','motor.error','encoder.error','controller.error','sensorless_estimator.error','motor.is_calibrated','encoder.is_ready','motor.config.pre_calibrated','encoder.config.pre_calibrated','encoder.vel_estimate','controller.input_vel','config.enable_watchdog','config.watchdog_timeout','config.startup_closed_loop_control','config.startup_motor_calibration','config.startup_encoder_index_search','config.startup_encoder_offset_calibration')}
    for p in ('can','can.config','system_stats'):
        value=read_path(board,p)
        result[p+'_available_attributes']=[] if value is None else [name for name in dir(value) if not name.startswith('__')]
    result['finished_at']=datetime.datetime.now(datetime.timezone.utc).isoformat()
    pathlib.Path('/evidence/usb-board-initial.json').write_text(json.dumps(result,indent=2)+'\n')
    print('SNAPSHOT '+json.dumps(result,indent=2),flush=True)
