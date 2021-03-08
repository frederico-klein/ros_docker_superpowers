#!/usr/bin/env python
PKG = 'rosdop'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import sys
import unittest
import subprocess
from rosdop.docker_master import DockerMasterInterface as DMI

myDMI = DMI(-1) ## so it doesnt close everything
## A sample python unit test
class TestBareBones(unittest.TestCase):
    ## test 1 == 1
    def test_one_equals_one(self): # only functions with 'test_'-prefix will be run!
        self.assertEquals(1, 1, "1!=1")
    def ping_everthing(self):
	for a_host in myDMI.master.
	list_args = ["/bin/ping",a_host]
	proc = subprocess.Popen(list_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_bare_bones', TestBareBones)
