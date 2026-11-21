"""One bounded localization+trial session. No retry, even if preflight fails."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--speed',type=float,choices=[.10,.15],default=.10)
    p.add_argument('--duration',type=float,default=15.)
    p.add_argument('--stop-lead',type=float,default=0.)
    p.add_argument('--observe-deviation',action='store_true')
    p.add_argument('--execute-one-attempt',action='store_true')
    p.add_argument('--seed',type=float,nargs=3,required=True)
    args=p.parse_args();args.run.mkdir(parents=True,exist_ok=False)
    root=Path(__file__).resolve().parent
    session=subprocess.Popen(['bash',str(root/'run_diagnostic.bash'),'--run',str(args.run/'localization'),
                              '--duration','75','--seed',*map(str,args.seed)],start_new_session=True)
    try:
        deadline=time.monotonic()+30
        events=args.run/'localization/events.jsonl'
        ready=False
        while time.monotonic()<deadline and session.poll() is None:
            if events.exists():
                lines=events.read_text().splitlines()
                # Only a recent status authorizes starting preflight; trial verifies independently.
                for line in reversed(lines[-30:]):
                    try:e=json.loads(line)
                    except ValueError:continue
                    if e['kind']=='nav_status':
                        ready=e['value']['valid'] and 0<=time.time()-e['received_epoch']<.25
                        break
            if ready:break
            time.sleep(.1)
        if not ready:raise RuntimeError('localization_not_ready_no_trial_started')
        command=['bash',str(root/'run_straight.bash'),'--output',str(args.run/'trial'),'--speed',str(args.speed),'--duration',str(args.duration),'--stop-lead',str(args.stop_lead)]
        if args.execute_one_attempt:command.append('--execute-one-attempt')
        if args.observe_deviation:command.append('--observe-deviation')
        # One invocation only; no automatic retry or resume.
        subprocess.run(command,check=True,timeout=45)
    finally:
        if session.poll() is None:
            session.send_signal(signal.SIGINT)
            session.wait(timeout=25)


if __name__=='__main__':main()
