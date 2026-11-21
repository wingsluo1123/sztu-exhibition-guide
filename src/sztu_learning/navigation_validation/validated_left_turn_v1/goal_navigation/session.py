"""Bounded localization+one mission supervisor; never retries a motor action."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--seed',type=float,nargs=3,required=True)
    args,forward=p.parse_known_args()
    if any(x in forward for x in ('--run','--output','--seed')):raise ValueError('supervisor-owned argument')
    args.run.mkdir(parents=True,exist_ok=False)
    root=Path(__file__).resolve().parent
    session=subprocess.Popen(['bash',str(root.parent/'run_diagnostic.bash'),'--run',str(args.run/'localization'),
                              '--duration','240','--seed',*map(str,args.seed)],start_new_session=True)
    stopping=False
    def request_stop(*_):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGINT,request_stop);signal.signal(signal.SIGTERM,request_stop)
    mission=None
    try:
        deadline=time.monotonic()+30;ready=False
        while time.monotonic()<deadline and session.poll() is None and not stopping:
            events=args.run/'localization/events.jsonl'
            if events.exists():
                for line in reversed(events.read_text().splitlines()[-30:]):
                    try:event=json.loads(line)
                    except ValueError:continue
                    if event['kind']=='nav_status':
                        ready=event['value']['valid'] and 0<=time.time()-event['received_epoch']<.25
                        break
            if ready:break
            time.sleep(.1)
        if not ready:raise RuntimeError('localization_not_ready_no_mission_started')
        mission=subprocess.Popen(['bash',str(root/'run_navigate.bash'),'--output',str(args.run/'mission'),*forward])
        deadline=time.monotonic()+200
        while mission.poll() is None:
            if stopping or time.monotonic()>deadline or session.poll() is not None:
                mission.terminate();mission.wait(timeout=8);break
            time.sleep(.1)
    finally:
        if mission is not None and mission.poll() is None:mission.terminate();mission.wait(timeout=8)
        if session.poll() is None:session.send_signal(signal.SIGINT);session.wait(timeout=25)


if __name__=='__main__':main()
