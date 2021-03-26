#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import subprocess
import socket
import traceback

#import re

class DockerLogged(object):

    def __init__(self):
        self.proc_list = []
        self.Name = ""
        self.ownHostName = socket.gethostname()
        self.attachOwnHostNameToDockerNames = False
        self.warnedAboutName = False
        rospy.on_shutdown(self.__ultraclose)

    def TrueName(self):
        fullname = self.Name+"."+self.ownHostName
        if self.attachOwnHostNameToDockerNames:
            if not self.warnedAboutName:
                rospy.loginfo("hostname_as_suffix set. Changing name from {} to {}".format(self.Name, fullname ))
                self.warnedAboutName = True
            return fullname
        else:
            return self.Name

    def FullName(self):
        return self.Name+"."+self.ownHostName

    #def afps(self, owner, private_name, classatrname):
    def afps(self, classatrname, private_name, default_attribute = ""):

        """
        old signature attribute_from_param_setter(owner, private_name, classatrname, default_attribute = getattr(self,  classatrname))

        """
        if default_attribute is "":
            default_attribute = getattr(self,  classatrname)
        setattr(self, classatrname, rospy.get_param('~{}'.format(private_name), default = default_attribute))#
        updated_param = getattr(self,  classatrname)
        rospy.logdebug('Parameter %s has value %s', rospy.resolve_name('~{}'.format(private_name)), updated_param)
        # I want to keep this updated because I will use these attributes for interprocess talk. it's just easier like this.
        rospy.set_param('~{}'.format(private_name), updated_param)

    def lspPopen(self, list_args, with_retries=False, num_retries = 1):
        """
        old logged_subprocess_popen
        """
        tries = num_retries
        while tries>0:
            rospy.logdebug("command being run:{}".format(list_args))
            proc = subprocess.Popen(list_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            res = proc.communicate()
            # output = proc.stdout.read()
            output = res[0]
            rospy.logdebug("process response: {}".format(output))
            errorout = res[1] # I think
            tries-=1
            if with_retries:
                if proc.returncode is 0:
                    break
                else:
                    rospy.logdebug("COMMAND: \n{}\n Did not work yet. Maybe try again in a sec?\nRESPONSE stdout: {}\nerrout: {}".format(list_args, output, errorout))
                    if proc.returncode is not 0:
                        rospy.logdebug(repr(traceback.format_stack()))
            else:
                break
            rospy.sleep(1)

        if proc.returncode is not 0:
            for a_line in repr(traceback.format_stack()):
                rospy.logerr(a_line)
            rospy.logerr(errorout)
            rospy.signal_shutdown(errorout)
        rospy.logdebug("output:\n{}".format(output))
        self.proc_list.append(proc)
        return (output, proc) ### i probably should catch all of those procs and kill them in the end.

    def lspPopenRetry(self, list_args, num_retries=10):
        return self.lspPopen(list_args, with_retries=True, num_retries=num_retries)

    def oLspPopen(self, list_args):
        return self.lspPopen(list_args)[0]

    def __ultraclose(self):
        for proc in self.proc_list:
            ## they should be dead already
            try:
                proc.kill()
            except:
                pass

class DockerLoggedNamed(DockerLogged):
    def updateHostName(self):
        self.RosHostName = rospy.get_node_uri().split("/")[2].split(":")[0]
        rospy.logdebug(self.RosHostName)
        rospy.logdebug(self.ownHostName)

        #assert(self.HostName is self.ownHostName) # maybe they will differ if ROS_HOSTNAME is used; I still don't know which one will be correct.
        ##Should not be an afps parameter because I don't want it to be changed
        rospy.logdebug("Hostname running this node: {}".format(self.RosHostName))
        rospy.logdebug("Own Hostname running this node: {}".format(self.ownHostName))
        rospy.set_param("~HostName",self.ownHostName)
