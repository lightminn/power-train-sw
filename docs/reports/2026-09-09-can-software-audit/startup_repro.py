from pathlib import Path
import ast, json, os, subprocess, sys, tempfile
from types import SimpleNamespace
root=Path(__file__).resolve().parents[3]
work=Path(tempfile.mkdtemp(prefix='audit-can-startup-',dir='/tmp'))
os.environ['PYTHONDONTWRITEBYTECODE']='1'

def executable(path,text):
    path.write_text(text);path.chmod(0o755)

def fakebin(name):
    p=work/name;p.mkdir();return p

# Execute only the production readiness function plus its production branch,
# with ip and sudo replaced by loggers. No real network or privilege command runs.
source=(root/'scripts/jetson_gui_up.sh').read_text().splitlines()
gui_excerpt='\n'.join(source[111:116])+'\n'+ '\n'.join(source[210:223])+'\n'
b=fakebin('gui')
executable(b/'ip', '#!/bin/sh\nprintf "%s\\n" "5: can0: <NOARP,UP,LOWER_UP> mtu 16 state UNKNOWN qlen 1000" "    can state ERROR-ACTIVE restart-ms 100" "    bitrate 500000"\n')
executable(b/'sudo', '#!/bin/sh\nprintf "%s\\n" "$*" >> "$AUDIT_CALL_LOG"\n')
script=work/'gui-readiness.sh';script.write_text('add_result() { printf "result %s\\n" "$*"; }\n'+gui_excerpt)
env={'PATH':str(b)+':/usr/bin:/bin','AUDIT_CALL_LOG':str(work/'gui.calls'),'LC_ALL':'C'}
r=subprocess.run(['/bin/bash',str(script)],env=env,capture_output=True,text=True,timeout=2)
print(json.dumps({'case':'legacy_gui_normal_UNKNOWN','returncode':r.returncode,'stdout':r.stdout,'sudo_calls':(work/'gui.calls').read_text()},ensure_ascii=False))

# Run the full production watchdog using fake ip/tc/sleep commands; the reported
# details model upstream iproute2 format (disabled ctrlmode omitted).
b=fakebin('watchdog')
executable(b/'tc', '#!/bin/sh\nprintf "%s\\n" " Sent 100 bytes 1 pkt" " backlog 10b 2p requeues 0"\n')
executable(b/'ip', '#!/bin/sh\nprintf "%s\\n" "$*" >> "$AUDIT_CALL_LOG"\ncase "$*" in "-details link show dev can0") printf "%s\\n" "5: can0: <NOARP,UP,LOWER_UP> state UNKNOWN qlen 1000" "    can state ERROR-ACTIVE restart-ms 100" "    bitrate 500000";; esac\n')
executable(b/'sleep', '#!/bin/sh\nexec /bin/sleep 0.01\n')
env={'PATH':str(b)+':/usr/bin:/bin','AUDIT_CALL_LOG':str(work/'watchdog.calls'),'LC_ALL':'C'}
r=subprocess.run(['/bin/bash',str(root/'scripts/can_watchdog.sh')],env=env,capture_output=True,text=True,timeout=2)
print(json.dumps({'case':'shell_watchdog_normal_ctrlmode_omission','returncode':r.returncode,'stdout':r.stdout,'ip_calls':(work/'watchdog.calls').read_text()},ensure_ascii=False))

# New launcher checks only first-line interface flags. CAN controller LOOPBACK
# appears on the detail line, which differs from the IFF_LOOPBACK interface bit.
source=(root/'scripts/robot-start').read_text().splitlines()
new_excerpt='\n'.join(source[85:101])+'\necho CAN_GATE_ACCEPTED\n'
b=fakebin('integrated')
executable(b/'ip', '#!/bin/sh\nprintf "%s\\n" "5: can0: <NOARP,UP,LOWER_UP> state UNKNOWN qlen 1000" "    can <LOOPBACK> state ERROR-ACTIVE restart-ms 100" "    bitrate 500000"\n')
script=work/'integrated-readiness.sh';script.write_text('fail() { printf "%s\\n" "$*"; exit 1; }\n'+new_excerpt)
r=subprocess.run(['/bin/bash',str(script)],env={'PATH':str(b)+':/usr/bin:/bin'},capture_output=True,text=True,timeout=2)
print(json.dumps({'case':'integrated_CAN_controller_LOOPBACK','returncode':r.returncode,'stdout':r.stdout},ensure_ascii=False))

# Extract the pure preflight parser, bypassing module imports and all hardware.
tree=ast.parse((root/'scripts/preflight_hil.py').read_text())
node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='can_up')
for name,detail in [('wrong_bitrate','5: can0: <NOARP,UP,LOWER_UP> state UP\n can state ERROR-ACTIVE\n bitrate 250000'),('controller_LOOPBACK','5: can0: <NOARP,UP,LOWER_UP> state UP\n can <LOOPBACK> state ERROR-ACTIVE\n bitrate 500000')]:
    ns={'subprocess':SimpleNamespace(run=lambda *a,**k:SimpleNamespace(stdout=detail)),'OK':'OK','FAIL':'FAIL','WARN':'WARN'}
    exec(compile(ast.Module(body=[node],type_ignores=[]),'<production_can_up>','exec'),ns)
    print(json.dumps({'case':'preflight_'+name,'result':ns['can_up']()},ensure_ascii=False))
print(json.dumps({'case':'repro_artifact_directory','path':str(work)}))

# Full integrated launcher, with only external ip/docker commands replaced.
case_dir=work/'full-integrated';case_dir.mkdir()
target=case_dir/'target'
for part in ('etc/powertrain','run/powertrain','var/lib/powertrain'):(target/part).mkdir(parents=True)
etc=target/'etc/powertrain'
(etc/'integrated-prepared.env').write_text('POWERTRAIN_INTEGRATED_VERSION=1\nPOWERTRAIN_PROFILE=can-4ws\n')
config={'robot_id':'audit-only','token_file':'/etc/powertrain/operator_console.token','host':'0.0.0.0','session_port':9002,'input_port':9000,'ops_port':9001,'input_target_port':19000,'ops_target_port':19001,'destination_file':'/run/powertrain/operator-session.json','lease_timeout_s':2.0}
(etc/'robot.json').write_text(json.dumps(config))
(etc/'operator_console.token').write_text('audit-dummy-not-a-credential\n')
b=case_dir/'bin';b.mkdir()
executable(b/'docker','#!/bin/sh\nprintf "%s\\n" "$*" >> "$AUDIT_CALL_LOG"\n')
for mode,detail in [('normal','can state ERROR-ACTIVE'),('controller_LOOPBACK','can <LOOPBACK> state ERROR-ACTIVE'),('controller_LISTEN_ONLY','can <LISTEN-ONLY> state ERROR-ACTIVE'),('BUS_OFF','can state BUS-OFF')]:
    executable(b/'ip', '#!/bin/sh\nprintf "%s\\n" "5: can0: <NOARP,UP,LOWER_UP> state UNKNOWN qlen 1000" "    '+detail+' restart-ms 100" "    bitrate 500000"\n')
    calls=case_dir/(mode+'.calls')
    env={'PATH':str(b)+':'+str(Path(sys.executable).parent)+':/usr/bin:/bin','POWERTRAIN_ROOT':str(target),'AUDIT_CALL_LOG':str(calls),'PYTHONDONTWRITEBYTECODE':'1'}
    r=subprocess.run(['/bin/bash',str(root/'scripts/robot-start')],env=env,capture_output=True,text=True,timeout=2)
    print(json.dumps({'case':'full_robot_start_'+mode,'returncode':r.returncode,'stdout':r.stdout,'stderr':r.stderr,'docker_calls':calls.read_text() if calls.exists() else ''},ensure_ascii=False))
