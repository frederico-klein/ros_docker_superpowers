#!/usr/bin/env python
# -*- coding: utf-8 -*-

### this should be a bunch of shell scripts. IDK what I am doing.

## creates the bridge.

import rospy
import subprocess

class DockerBridge():
    """
    This can be invoked with something like:
    from cr_bridge import DockerBridge

    myDockerBridge = DockerBridge(name="br0",subnet="172.28.0.0/16",iprange="172.28.6.0/24",gateway="172.28.6.254")
    myDockerBridge.create()

    or launched as a node with parameters from a launch file, with the same names.

    """
    def __init__(self,name="br0",
    subnet="172.28.0.0/16",
    iprange="172.28.6.0/24",
    gateway="172.28.6.254"):
        self.Name = name
        self.Driver = "bridge"
        self.Subnet = subnet
        self.IPRange = iprange
        self.Gateway = gateway
        rospy.init_node('docker_bridge', anonymous=True)
    def create(self):
        ## I want to read the private parameters here, since I already started the node, so I catkin_ws
        self.Name = rospy.get_param('~name', default = self.Name)
        rospy.logdebug('Parameter %s has value %s', rospy.resolve_name('~name'), self.Name)
        self.Subnet = rospy.get_param('~subnet', default = self.Subnet)
        rospy.logdebug('Parameter %s has value %s', rospy.resolve_name('~subnet'), self.Subnet)
        self.IPRange = rospy.get_param('~iprange', default = self.IPRange)
        rospy.logdebug('Parameter %s has value %s', rospy.resolve_name('~iprange'), self.IPRange)
        self.Gateway = rospy.get_param('~gateway', default = self.Gateway)
        rospy.logdebug('Parameter %s has value %s', rospy.resolve_name('~gateway'), self.Gateway)

        rospy.loginfo("Creating bridge {}".format(self.Name))
        ##check if there is a bridge already
        proc = subprocess.Popen(['docker','network','ls'], stdout=subprocess.PIPE)
        output = proc.stdout.read()

        if self.Name in output: #if there isn't create one
            rospy.loginfo("found {} docker network.".format(self.Name))
        else:
            # I am unsure about this.
            procDockerNetworkCreate = subprocess.Popen([
            "docker","network","create",
            "--driver={}".format(self.Driver),
            "--subnet={}".format(self.Subnet),
            "--ip-range={}".format(self.IPRange),
            "--gateway={}".format(self.Gateway),
            self.Name], stdout=subprocess.PIPE)
    def __del__(self):
        rospy.loginfo("Shutting down. Deleting bridge {}".format(self.Name))
        procDel = subprocess.Popen(['docker','network','rm',self.Name], stdout=subprocess.PIPE)

if __name__ == '__main__':
    try:
        myDockerBridge = DockerBridge(name="br0",subnet="172.28.0.0/16",iprange="172.28.6.0/24",gateway="172.28.6.254")
        myDockerBridge.create()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
