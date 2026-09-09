import datetime,json,pathlib,hashlib
import odrive,odrive.configuration
from chassis.usb_session import motor_session,communication_snapshot,read_path
serial='336A33523235'
out=pathlib.Path('/evidence/mks-reliability'); out.mkdir(exist_ok=True)
with motor_session('mks_firmware_readonly_backup'):
    board=odrive.find_any(serial_number=serial,timeout=12)
    assert format(board.serial_number,'X')==serial
    states={name:{p:read_path(getattr(board,name),p) for p in ('current_state','requested_state','controller.input_vel','encoder.vel_estimate','motor.is_calibrated','encoder.is_ready','error','motor.error','encoder.error')} for name in ('axis0','axis1')}
    assert all(v['current_state']==1 and v['controller.input_vel']==0 and abs(v['encoder.vel_estimate'])<0.01 for v in states.values()),states
    first=odrive.configuration.get_dict(board,board,False)
    second=odrive.configuration.get_dict(board,board,False)
    assert json.dumps(first,sort_keys=True)==json.dumps(second,sort_keys=True),'configuration changed during backup'
    def count(x): return sum(count(v) if isinstance(v,dict) else 1 for v in x.values())
    text=json.dumps(first,indent=2,sort_keys=True)+'\n'
    configpath=out/(serial+'-configuration.json')
    with configpath.open('x') as f:f.write(text)
    meta={'serial':serial,'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'communication':communication_snapshot(board,serial=serial),'axes':states,'config_fields':count(first),'config_sha256':hashlib.sha256(text.encode()).hexdigest(),'config_readback_twice_equal':True,'method':'read-only Fibre configuration tree via SDK get_dict; no save/set/reboot/flash','board_attributes':list(dir(board)),'can_attributes':list(dir(board.can)),'firmware_unreleased':board.fw_version_unreleased,'user_config_loaded':getattr(board,'user_config_loaded',None)}
    (out/(serial+'-backup-metadata.json')).write_text(json.dumps(meta,indent=2)+'\n')
    print(json.dumps(meta,indent=2),flush=True)
