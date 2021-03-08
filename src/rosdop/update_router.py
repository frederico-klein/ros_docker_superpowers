#!/usr/bin/env python
# -*- coding: utf-8 -*-

try:
    from rospy import spin
    waitforever = spin()
except:
    import time
    waitforever = lambda: time.sleep(86400) # a day, not forever
import subprocess, shutil


def update_resolvconf():
    shutil.copyfile(src="/etc/resolv.conf",dst="/etc/resolv.conf.backup")
    with open("/etc/resolv.conf","a") as resolvfile:

        resolvfile.write("nameserver 172.28.6.53")

def revert_resolvconf():
    shutil.copyfile(dst="/etc/resolv.conf",src="/etc/resolv.conf.backup")

def update_routes():
    subprocess.Popen(["sudo","route","add","-net","172.28.6.0","netmask","255.255.255.0","gw","lavine.local"])
    subprocess.Popen(["sudo","route","add","-net","172.28.5.0","netmask","255.255.255.0","gw","poop.local"])

def revert_routes():
    subprocess.Popen(["sudo","route","del","-net","172.28.6.0","netmask","255.255.255.0","gw","lavine.local"])
    subprocess.Popen(["sudo","route","del","-net","172.28.5.0","netmask","255.255.255.0","gw","poop.local"])




class Router():
    def __enter__(self):
        update_resolvconf()
        update_routes()
    def __exit__(self, *exc):
        revert_resolvconf()
        revert_routes()

if __name__ == '__main__':
    with Router as MyRouter:
        waitforever()
