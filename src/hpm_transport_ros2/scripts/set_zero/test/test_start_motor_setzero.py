"""Offline launcher check: every ros2 command is a local fake executable."""
import json,os,pty,signal,subprocess,tempfile,time,textwrap
from pathlib import Path

def test_launcher_offline():
    source = Path(__file__).resolve().parents[3] / "start_motor_setzero.sh"
    with tempfile.TemporaryDirectory(prefix='setzero-launch-test-') as tmp:
     root=Path(tmp);ws=root/'workspace';(ws/'install').mkdir(parents=True);bindir=root/'bin';bindir.mkdir();log=root/'calls.jsonl'
     (ws/'install/setup.bash').write_text('export SETZERO_TEST_SOURCED=1\n')
     fake=bindir/'ros2';fake.write_text(textwrap.dedent('''
    #!/usr/bin/env python3
    import json,os,signal,sys,time
    from pathlib import Path
    log=Path(os.environ['SETZERO_TEST_LOG'])
    def record(event):
     with log.open('a') as f:f.write(json.dumps({'event':event,'args':sys.argv[1:],'tty':os.isatty(0),'sourced':os.getenv('SETZERO_TEST_SOURCED'),'cwd':os.getcwd()})+'\\n')
    assert sys.argv[1]=='launch'
    if sys.argv[2]=='hpm_system':
     def stop(sig,frame):record('system_stop');sys.exit(0)
     signal.signal(signal.SIGINT,stop);record('system_start')
     print('[hpm_system_node-1] [INFO] [hpm_system]: UDP connecting to 192.168.1.10:5001',flush=True)
     mode=os.getenv('SETZERO_TEST_LINK_MODE','connected')
     if mode=='exit':sys.exit(9)
     if mode=='connected':
      time.sleep(.1);record('system_connected')
      print('[hpm_system_node-1] [INFO] [hpm_system]: UDP connected to 192.168.1.10:5001',flush=True)
     while True:time.sleep(.01)
    server_mode = sys.argv[1:] == ['launch', 'set_zero', 'set_zero_server.launch.py']
    assert server_mode
    record('calibration_start')
    # Model the real calibration preflight: communications must be ready first.
    deadline=time.monotonic()+2
    while not log.exists() or not any(json.loads(line)['event']=='system_start' for line in log.read_text().splitlines()):
     assert time.monotonic()<deadline
     time.sleep(.01)
    if os.environ.get('SETZERO_TEST_WAIT'):
     def stop(sig,frame):record('calibration_stopping');time.sleep(.05);record('calibration_stopped');sys.exit(130)
     signal.signal(signal.SIGINT,stop);record('calibration_ready')
     while True:time.sleep(.01)
    record('calibration_done');sys.exit(int(os.environ.get('SETZERO_TEST_CODE','0')))
    ''').lstrip());fake.chmod(0o755)
     repo=ws/'src/hpm_transport_ros2';repo.mkdir(parents=True)
     script=repo/'start_motor_setzero.sh';script.write_text(source.read_text().replace('connect_timeout_s=10','connect_timeout_s=1'));script.chmod(0o755)
     env={**os.environ,'PATH':str(bindir)+':'+os.environ['PATH'],'SETZERO_TEST_LOG':str(log)}
     def events():return [json.loads(x) for x in log.read_text().splitlines()] if log.exists() else []
     def clear():log.unlink(missing_ok=True)
     for args in [['--module','left_leg'],['--module'],['--module','bad'],['--bogus'],['--server','--module','neck'],['--module','neck','--server'],['--server','--server'],['--module','neck','--module','left_leg']]:
      clear();r=subprocess.run([str(script),*args],env=env,capture_output=True,text=True,timeout=5)
      assert r.returncode==2,(args,r);assert not events(),args
     clear();assert subprocess.run([str(script),'--help'],env=env,capture_output=True,timeout=5).returncode==0;assert not events()
     for mode in ['connecting','exit']:
      clear();r=subprocess.run([str(script)],input='',env={**env,'SETZERO_TEST_LINK_MODE':mode},capture_output=True,text=True,timeout=5)
      assert r.returncode==1,(mode,r.stdout,r.stderr)
      assert not any(e['event'].startswith('calibration_') for e in events()),events()
      if mode=='connecting':assert events()[-1]['event']=='system_stop'
     clear();r=subprocess.run([str(script)],input='',env={**env,'SETZERO_TEST_CODE':'7'},capture_output=True,text=True,timeout=5);assert r.returncode==7,r;assert events()[-1]['event']=='system_stop'
     for args in ([], ['--server']):
      clear();master,slave=pty.openpty();p=subprocess.Popen([str(script),*args],stdin=slave,stdout=subprocess.PIPE,stderr=subprocess.PIPE,env={**env,'SETZERO_TEST_WAIT':'1'},start_new_session=True)
      try:
       deadline=time.monotonic()+5
       while not any(e['event']=='calibration_ready' for e in events()):
        assert time.monotonic()<deadline;time.sleep(.01)
       start = next(e for e in events() if e['event']=='calibration_start')
       assert start['args'] == ['launch', 'set_zero', 'set_zero_server.launch.py']
       assert start['cwd'] == str(ws) and start['sourced'] == '1'
       os.killpg(p.pid,signal.SIGINT);out,err=p.communicate(timeout=5);assert p.returncode==130,(p.returncode,out,err)
       names=[e['event'] for e in events()];assert names.index('calibration_stopped')<names.index('system_stop'),names
      finally:
       if p.poll() is None:p.kill();p.wait()
       os.close(master);os.close(slave)
     print('PASS: no arguments start the Action server; invalid arguments start nothing; legacy module arguments rejected; connected gate, no-connect timeout, early communication exit, exit status and Ctrl+C cleanup order verified with fake ros2.')


if __name__ == "__main__":
    test_launcher_offline()
