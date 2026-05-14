#!/usr/bin/env python3
"""Upload a file to the Pi. Usage: pi_put.py <local> <remote>"""
import sys, paramiko
HOST="raspberrypi.tail7c9eac.ts.net"; USER="pi"; PW="aw"
local, remote = sys.argv[1], sys.argv[2]
c = paramiko.SSHClient(); c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(HOST, username=USER, password=PW, timeout=15)
sftp = c.open_sftp(); sftp.put(local, remote); sftp.close(); c.close()
print(f"ok {local} -> {remote}")
