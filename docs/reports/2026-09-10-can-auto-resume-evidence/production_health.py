import json,pathlib,subprocess
out=pathlib.Path('/home/zetin/powertrain-can-evidence-20260910')
names=['powertrain_canwatchdog','powertrain_chassis','powertrain_control','powertrain_session','powertrain_ros','powertrain_observability','powertrain_chassis_telemetry','powertrain_pdist80b_telemetry']
rows=[]
for name in names:
    item=json.loads(subprocess.check_output(['docker','inspect',name]))[0]; state=item['State']
    row={'name':name,'status':state['Status'],'running':state['Running'],'started_at':state['StartedAt'],'restart_count':item['RestartCount'],'health':state.get('Health',{}).get('Status')}
    assert row['running'] and row['health'] not in ('unhealthy','starting'),row
    if name in names[:4]:
        run=subprocess.run(['docker','logs','--since',state['StartedAt'],name],capture_output=True,text=True,check=True)
        log=run.stdout+run.stderr;(out/(name+'-current-start.log')).write_text(log)
        row['traceback_or_process_death']=any(s in log for s in ('Traceback (most recent call last)','process has died','RuntimeError:'))
        assert not row['traceback_or_process_death'],row
    rows.append(row)
extra=json.loads(subprocess.check_output(['docker','inspect','powertrain-can-resume-test']))[0]['State']['Running'];assert not extra
result={'result':'PASS','services':rows,'isolated_test_container_running':extra,'docker_unit':subprocess.check_output(['systemctl','is-active','docker'],text=True).strip()};assert result['docker_unit']=='active'
(out/'production-service-health.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
