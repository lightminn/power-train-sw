import json,os,signal,subprocess,time,sys
from pathlib import Path
assert os.environ['ROS_DOMAIN_ID']=='77'
assert not Path('/sys/class/net/can0').exists()
assert Path('/sys/class/net/vcan77').exists()
assert set(p.name for p in Path('/sys/class/net').iterdir()) <= {'lo','vcan77'}, 'run in --network none with only vcan77'
out=Path('/evidence/loop');out.mkdir(exist_ok=True)
with (out/'fixture.log').open('w') as log:
    fixture=subprocess.Popen(['python3','/evidence/loop_fixture.py'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        result=subprocess.run(['python3','/evidence/loop_client.py',*sys.argv[1:]],env=dict(os.environ,LOOP_HOST='127.0.0.1',LOOP_EVIDENCE_DIR=str(out)),timeout=110)
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
if not any(flag in sys.argv for flag in ('--negative-neutral','--negative-fake')):
    observation=json.loads((out/'wheel-observation.json').read_text())
    assert observation['dds_receipt_samples']>0, 'manual DDS receipt was not observed'
    assert observation['dds_receipt_invalid']==0, observation
    assert observation['stopped_after_motion'], 'no installed wheel feedback stop after motion'
    result=json.loads((out/'loop-result.json').read_text())
    assert result['manual_ackermann_cases']==6 and result['stationary_cases']==2, result
print('All installed nodes stopped without traceback',flush=True)
