#!/usr/bin/env python3
"""Download a file from the Pi. Usage: pi_get.py <remote> <local>"""
import sys, paramiko
HOST="raspberrypi.tail7c9eac.ts.net"; USER="pi"; PW="aw"
remote, local = sys.argv[1], sys.argv[2]
c = paramiko.SSHClient(); c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(HOST, username=USER, password=PW, timeout=15)
sftp = c.open_sftp(); sftp.get(remote, local); sftp.close(); c.close()
print(f"ok {remote} -> {local}")
