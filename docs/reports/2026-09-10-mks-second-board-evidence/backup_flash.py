"""Read original target flash twice through the reviewed DFU transport; no erase/write."""
import hashlib,json
from tools.mks_dfu import BASE,FLASH_SIZE,APP_SIZE,validate_image
from chassis.usb_session import motor_session
import odrive
from flash_second_board import SERIAL,OUT,check_idle,enter_dfu,return_to_app,get_config,record
expected=json.loads((OUT/(SERIAL+'-after-original-reboot-configuration.json')).read_text())
result={'serial':SERIAL,'operations':record['operations']}
def save():
 (OUT/(SERIAL+'-flash-readback.json')).write_text(json.dumps(result,indent=2)+'\n')
with motor_session('mks_second_board_original_backup'):
 board=odrive.find_any(serial_number=SERIAL,timeout=15)
 result['before_axes']=check_idle(board)
 assert get_config(board)==expected
 for name in ('axis0','axis1'):
  assert not any(v for k,v in expected[name]['config'].items() if k.startswith('startup_'))
 transport=enter_dfu(board)
 try:
  one=transport.read(BASE,FLASH_SIZE)
  two=transport.read(BASE,FLASH_SIZE)
  assert one==two,'two original ROM reads differ'
  validate_image(one[:APP_SIZE])
  with (OUT/(SERIAL+'-original-1m.bin')).open('xb') as f:f.write(one)
  result.update(bytes=len(one),sha256=hashlib.sha256(one).hexdigest(),reads_equal=True,nvm_sha256=hashlib.sha256(one[APP_SIZE:]).hexdigest(),application_sha256=hashlib.sha256(one[:APP_SIZE]).hexdigest())
 except Exception as error:
  result['error']=repr(error)
  raise
 finally:
  save()
  board=return_to_app(transport)
  result['after_axes']=check_idle(board)
  result['configuration_unchanged']=get_config(board)==expected
  save()
  assert result['configuration_unchanged']
print(json.dumps({k:v for k,v in result.items() if k!='operations'},indent=2))
