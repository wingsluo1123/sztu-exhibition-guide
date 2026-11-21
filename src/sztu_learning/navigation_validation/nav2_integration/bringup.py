"""Own localization, private input adapter and Nav2. Never create motor output."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seed',type=float,nargs=3,required=True)
    args=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='0': raise RuntimeError('robot domain0 required')
    args.output.mkdir(parents=True,exist_ok=False)
    stopping=False; processes={}; handles=[]
    def stop(*_):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    commands=dict(
        localization=[sys.executable,str(HERE.parent/'diagnostic_session.py'),
                      '--run',str(args.output/'localization'),'--duration','0','--seed',*map(str,args.seed)],
        adapter=[sys.executable,str(HERE/'input_adapter.py')],
        nav2=['bash',str(HERE/'run_stack.bash')])
    report=dict(commands=commands,started_epoch=time.time(),motor_output=False)
    try:
        for name,command in commands.items():
            handle=(args.output/(name+'.log')).open('x');handles.append(handle)
            processes[name]=subprocess.Popen(command,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
            time.sleep(.75)
        print(json.dumps(dict(started={k:v.pid for k,v in processes.items()},motor_output=False)),flush=True)
        while not stopping:
            for name,process in processes.items():
                if process.poll() is not None:
                    raise RuntimeError(name+' exited '+str(process.returncode))
            time.sleep(.2)
    except Exception as error:
        report['error']=repr(error)
    finally:
        for process in reversed(list(processes.values())):
            if process.poll() is None: process.send_signal(signal.SIGINT)
        for process in processes.values():
            try:process.wait(timeout=10.)
            except subprocess.TimeoutExpired:
                process.terminate();process.wait(timeout=5.)
        report.update(ended_epoch=time.time(),exit_codes={k:p.returncode for k,p in processes.items()})
        for handle in handles:handle.close()
        (args.output/'supervisor.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
