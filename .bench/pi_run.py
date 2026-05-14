#!/usr/bin/env python3
"""Run a command on the Pi via SSH. Usage: pi_run.py "<cmd>" """
import sys, paramiko
HOST="raspberrypi.tail7c9eac.ts.net"; USER="pi"; PW="aw"
cmd = sys.argv[1] if len(sys.argv) > 1 else "echo hi"
c = paramiko.SSHClient(); c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(HOST, username=USER, password=PW, timeout=15)
stdin, stdout, stderr = c.exec_command(cmd, timeout=120)
out = stdout.read().decode(errors="replace")
err = stderr.read().decode(errors="replace")
rc = stdout.channel.recv_exit_status()
sys.stdout.write(out)
if err: sys.stderr.write(err)
c.close(); sys.exit(rc)
