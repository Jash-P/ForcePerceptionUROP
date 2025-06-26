#!/usr/bin/env python3
import os, subprocess, json, time, threading, atexit

HERE    = os.path.dirname(os.path.abspath(__file__))
SERVICE = os.path.join(HERE, 'AR10_control', 'AR10_service.py')
print(f"[CLIENT] Looking for service at: {SERVICE!r}")
assert os.path.isfile(SERVICE), "AR10_service.py not found"

proc = subprocess.Popen(
    ['python2', SERVICE],
    stdin  = subprocess.PIPE,
    stdout = subprocess.PIPE,
    stderr = subprocess.PIPE,
    text=True, bufsize=1
)
print(f"[CLIENT] Launched service PID={proc.pid}")

# log all service stderr
def log_err():
    for line in proc.stderr:
        print("[SERVICE ERR]", line.rstrip())
threading.Thread(target=log_err, daemon=True).start()

@atexit.register
def cleanup():
    try:
        proc.stdin.write(json.dumps({'action':'shutdown'}) + "\n")
        proc.stdin.flush()
    except:
        pass
    proc.terminate()
    proc.wait(1)

def ar10_send(cmd):
    print(f"[CLIENT] sending: {cmd}")
    proc.stdin.write(json.dumps(cmd) + "\n")
    proc.stdin.flush()

    # wait for a JSON response line
    while True:
        raw = proc.stdout.readline()
        if raw is None or raw == "":
            raise RuntimeError("Service stdout closed")
        print(f"[CLIENT] got raw stdout: {raw.rstrip()!r}")
        if raw.strip().startswith("{"):
            resp = json.loads(raw)
            print(f"[CLIENT] parsed resp: {resp}")
            return resp

time.sleep(0.2)

print("→ OPEN →", ar10_send({'action':'open'}))
time.sleep(2)
print("→ CLOSE →", ar10_send({'action':'close'}))
