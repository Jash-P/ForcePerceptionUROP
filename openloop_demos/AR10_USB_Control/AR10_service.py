#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import os, sys, json, time

# 1) Ensure we’re in the right directory
HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)

# 2) Wait for the Maestro’s serial port to appear
port = '/dev/ttyACM0'
deadline = time.time() + 10.0
while not os.path.exists(port) and time.time() < deadline:
    sys.stderr.write("[SERVICE] waiting for {}...\n".format(port))
    sys.stderr.flush()
    time.sleep(0.5)
if not os.path.exists(port):
    sys.stderr.write("[SERVICE] ERROR: {} not found after 10s\n".format(port))
    sys.stderr.flush()
    sys.exit(1)

# 3) Import and instantiate the AR10 class
from AR10_class import AR10
hand = AR10()

sys.stderr.write("[SERVICE] started up and ready\n"); sys.stderr.flush()

# 4) Command dispatcher
def handle(cmd):
    action = cmd.get('action')
    if action == 'open':
        hand.open_hand()
        return {'status': 'ok'}
    elif action == 'close':
        hand.close_hand()
        return {'status': 'ok'}
    elif action == 'read_positions':
        # Return calibrated positions for joints 0–9
        pos = [hand.get_position(hand.joint_to_channel(j)) for j in range(10)]
        return {'status': 'ok', 'positions': pos}
    elif action == 'shutdown':
        hand.close()  # close the USB port
        return {'status': 'ok'}
    else:
        return {'status': 'error', 'error': 'unknown action {}'.format(action)}

# 5) Main I/O loop
while True:
    sys.stderr.write("[SERVICE] waiting for input...\n"); sys.stderr.flush()
    raw = sys.stdin.readline()
    sys.stderr.write("[SERVICE] raw readline: {!r}\n".format(raw)); sys.stderr.flush()

    # If stdin closed or EOF
    if raw is None or raw == '':
        sys.stderr.write("[SERVICE] stdin closed, exiting\n"); sys.stderr.flush()
        break

    line = raw.strip()
    if not line:
        continue

    # Parse JSON
    try:
        cmd = json.loads(line)
        sys.stderr.write("[SERVICE] parsed cmd: {}\n".format(cmd)); sys.stderr.flush()
    except Exception as e:
        sys.stderr.write("[SERVICE] JSON parse error: {}\n".format(e)); sys.stderr.flush()
        resp = {'status': 'error', 'error': 'invalid JSON'}
    else:
        # Dispatch
        try:
            resp = handle(cmd)
        except Exception as e:
            resp = {'status': 'error', 'error': str(e)}

    # Send JSON response
    out = json.dumps(resp)
    sys.stderr.write("[SERVICE] sending resp: {}\n".format(out)); sys.stderr.flush()
    sys.stdout.write(out + "\n"); sys.stdout.flush()