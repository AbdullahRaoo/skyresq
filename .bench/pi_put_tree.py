#!/usr/bin/env python3
"""Recursively upload a local directory to the Pi.
Usage: pi_put_tree.py <local_dir> <remote_dir>"""
import os, sys, paramiko, stat
HOST="raspberrypi.tail7c9eac.ts.net"; USER="pi"; PW="aw"
local, remote = sys.argv[1].rstrip("/"), sys.argv[2].rstrip("/")
c = paramiko.SSHClient(); c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(HOST, username=USER, password=PW, timeout=30)
sftp = c.open_sftp()
def mkdir_p(path):
    try:
        sftp.stat(path)
    except IOError:
        parent = os.path.dirname(path)
        if parent and parent != "/":
            mkdir_p(parent)
        sftp.mkdir(path)
mkdir_p(remote)
for root, dirs, files in os.walk(local):
    rel = os.path.relpath(root, local)
    rdir = remote if rel == "." else f"{remote}/{rel}"
    mkdir_p(rdir)
    for fn in files:
        lp = os.path.join(root, fn); rp = f"{rdir}/{fn}"
        sftp.put(lp, rp)
        # preserve exec bit
        if os.access(lp, os.X_OK):
            sftp.chmod(rp, 0o755)
        print(f"ok {lp} -> {rp}")
sftp.close(); c.close()
