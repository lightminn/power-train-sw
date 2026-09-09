"""One-board backup probe. No erase, unprotect, flash write, or option-byte write."""
import datetime,hashlib,json,pathlib,struct,time,traceback
import odrive,odrive.configuration,usb.core,usb.util
from chassis.usb_session import motor_session
serial='336A33523235'
out=pathlib.Path('/evidence/mks-reliability')
record={'serial':serial,'started_at':datetime.datetime.now(datetime.timezone.utc).isoformat(),'operations':[]}
def save(): (out/(serial+'-flash-readback.json')).write_text(json.dumps(record,indent=2)+'\n')
def control(dev,typ,req,val,data):
    record['operations'].append({'type':typ,'request':req,'value':val,'payload':list(data) if isinstance(data,bytes) else data})
    return dev.ctrl_transfer(typ,req,val,0,data,timeout=2000)
def status(dev):
    raw=bytes(control(dev,0xa1,3,0,6)); return raw[0],raw[4],int.from_bytes(raw[1:4],'little')
def wait(dev,states):
    until=time.monotonic()+5
    while time.monotonic()<until:
        s=status(dev)
        if s[0]: raise RuntimeError('DFU status error '+repr(s))
        if s[1] in states:return s
        time.sleep(min(0.1,max(0.001,s[2]/1000)))
    raise TimeoutError('DFU state timeout')
def abort(dev):control(dev,0x21,6,0,b''); wait(dev,{2})
def address(dev,addr):
    control(dev,0x21,1,0,b'\x21'+struct.pack('<I',addr)); wait(dev,{5}); abort(dev)
def read(dev,addr,size):
    abort(dev); address(dev,addr)
    data=bytearray()
    for i in range(0,size,2048):
        part=bytes(control(dev,0xa1,2,2+i//2048,min(2048,size-i)))
        if len(part)!=min(2048,size-i):raise RuntimeError('short upload')
        data.extend(part)
    abort(dev); return bytes(data)
with motor_session('mks_original_flash_readback'):
    assert usb.core.find(idVendor=0x0483,idProduct=0xdf11) is None,'DFU device already present'
    board=odrive.find_any(serial_number=serial,timeout=12)
    assert format(board.serial_number,'X')==serial
    before=odrive.configuration.get_dict(board,board,False)
    assert json.dumps(before,sort_keys=True)==json.dumps(json.loads((out/(serial+'-configuration.json')).read_text()),sort_keys=True)
    for a in (board.axis0,board.axis1):
        assert a.current_state==1 and a.controller.input_vel==0 and abs(a.encoder.vel_estimate)<0.01
        for name,value in before['axis0' if a is board.axis0 else 'axis1']['config'].items():
            if name.startswith('startup_'): assert not value,(name,value)
    save()
    try: board.enter_dfu_mode()
    except Exception as e:record['enter_disconnect']=type(e).__name__
    dev=None
    until=time.monotonic()+15
    while time.monotonic()<until:
        choices=list(usb.core.find(find_all=True,idVendor=0x0483,idProduct=0xdf11))
        matches=[d for d in choices if usb.util.get_string(d,d.iSerialNumber).upper()==serial]
        if len(matches)==1:dev=matches[0];break
        time.sleep(0.2)
    if dev is None:record['error']='matching DFU serial did not enumerate';save();raise RuntimeError(record['error'])
    try:
        record['dfu_serial']=usb.util.get_string(dev,dev.iSerialNumber)
        dev.set_configuration()
        cfg=dev.get_active_configuration()
        record['alternates']=[{'alt':i.bAlternateSetting,'description':usb.util.get_string(dev,i.iInterface)} for i in cfg]
        flash=[i for i in cfg if '@Internal Flash' in usb.util.get_string(dev,i.iInterface)]
        assert len(flash)==1
        flash[0].set_altsetting()
        if status(dev)[1]==10:control(dev,0x21,4,0,b'')
        one=read(dev,0x08000000,1024*1024)
        two=read(dev,0x08000000,1024*1024)
        assert one==two,'two flash reads differ'
        sp,pc=struct.unpack_from('<II',one)
        assert 0x20000000<=sp<=0x20020000 and 0x08000000<=(pc&~1)<0x080c0000,(sp,pc)
        (out/(serial+'-original-1m.bin')).write_bytes(one)
        record.update(bytes=len(one),sha256=hashlib.sha256(one).hexdigest(),reads_equal=True,initial_sp=sp,reset_vector=pc,nvm_sha256=hashlib.sha256(one[0xc0000:]).hexdigest())
    except Exception as e:
        record['error']=repr(e); traceback.print_exc()
    finally:
        save()
        try:
            if status(dev)[1]==10:control(dev,0x21,4,0,b'')
            abort(dev);address(dev,0x08000000)
            control(dev,0x21,1,0,b'')
            try:wait(dev,{7,8})
            except usb.core.USBError:pass
            record['application_jump_sent']=True
        except Exception as e:record['application_jump_error']=repr(e)
        save()
    try:
        after=odrive.find_any(serial_number=serial,timeout=20)
        after_config=odrive.configuration.get_dict(after,after,False)
        record['config_unchanged']=json.dumps(before,sort_keys=True)==json.dumps(after_config,sort_keys=True)
        record['after_axes']=[{'state':a.current_state,'velocity':a.encoder.vel_estimate,'error':a.error,'motor_calibrated':a.motor.is_calibrated,'encoder_ready':a.encoder.is_ready} for a in (after.axis0,after.axis1)]
        record['returned_to_original_application']=True
    except Exception as e:record['application_readback_error']=repr(e)
    save()
    print(json.dumps({k:v for k,v in record.items() if k!='operations'},indent=2),flush=True)
    if record.get('error') or not record.get('config_unchanged'):raise SystemExit(1)
