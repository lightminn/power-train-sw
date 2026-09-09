import os,signal,subprocess,time
from pathlib import Path
assert os.environ['ROS_DOMAIN_ID']=='77'
assert not Path('/sys/class/net/can0').exists()
out=Path('/evidence/loop');out.mkdir(exist_ok=True)
with (out/'fixture.log').open('w') as log:
    fixture=subprocess.Popen(['python3','/evidence/loop_fixture.py'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        result=subprocess.run(['python3','/evidence/loop_client.py'],env=dict(os.environ,LOOP_HOST='127.0.0.1',LOOP_EVIDENCE_DIR=str(out)),timeout=110)
        assert result.returncode==0, result.returncode
        assert fixture.poll() is None, 'fixture unexpectedly exited'
    finally:
        if fixture.poll() is None:
            os.killpg(fixture.pid,signal.SIGINT)
            try:fixture.wait(timeout=45)
            except subprocess.TimeoutExpired:
                os.killpg(fixture.pid,signal.SIGKILL);fixture.wait();raise
        print('fixture exit',fixture.returncode,flush=True)

for path in out.glob('node-*.log'):
    assert 'Traceback' not in path.read_text(), str(path) + ' traceback during execution or cleanup'
assert fixture.returncode==0
print('All installed nodes stopped without traceback',flush=True)
