#!/usr/bin/env python
PKG = 'rosdop'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.
import rospy
import sys
import unittest
import subprocess
from rosdop.docker_master import DockerMasterInterface as DMI

rospy.init_node("transient_test_node", anonymous=True, log_level=rospy.DEBUG)
## NOTE: http://wiki.ros.org/rostest/Connecting%20to%20the%20rostest%20master
## Rostest master is transient, so you need to run this test with a roslaunch
## otherwise it will not connect to an existing master.

myDMI = DMI(-1) ## so it doesnt close everything

class TestBareBones(unittest.TestCase):

    def test_ping_everthing(self):
        for a_host, ip in myDMI.master.docker_hosts.iteritems():
            self.ping_hosts(a_host)
    def test_ping_everthing_as_local(self):
        for a_host, ip in myDMI.master.docker_hosts.iteritems():
            host_dot_local = a_host+".local"
            self.ping_hosts(host_dot_local)
    def test_ping_containers(self):
        for host,containers in myDMI.master.HostDic.iteritems():
            for a_container,ip in containers.iteritems(): ## we don't care right now for the IP
                container_at_host = a_container+"."+host
                self.ping_hosts(container_at_host)

    def ping_hosts(self, a_host):
        print(a_host)
        rospy.loginfo("Pinging: {}".format(a_host))
        list_args = ["/bin/ping",a_host, "-c","1"]
        proc = subprocess.Popen(list_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        res = proc.communicate()
        output = res[0]
        errorout = res[1] # I think
        rospy.logdebug("output:\n{}".format(output))
        rospy.logerr(errorout)
        expr="Could not ping host: {}".format(a_host)
        self.assertTrue(proc.returncode is 0,expr)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_bare_bones', TestBareBones)
