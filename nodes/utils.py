#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import subprocess
#import re

class DockerLogged(object):

    def __init__(self):
        self.proc_list = []

        rospy.on_shutdown(self.__ultraclose)

    #def afps(self, owner, private_name, classatrname):
    def afps(self, classatrname, private_name):

        """
        old signature attribute_from_param_setter(owner, private_name, classatrname)

        """
        ##very non pythonic. improve
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
            proc = subprocess.Popen(list_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            res = proc.communicate()
            rospy.logdebug("command being run:{}".format(list_args))
            # output = proc.stdout.read()
            output = res[0]
            errorout = res[1] # I think
            tries-=1
            if with_retries:
                if proc.returncode is 0:
                    break
                else:
                    rospy.logdebug("COMMAND: {} Did not work yet. Maybe try again in a sec?".format(list_args))
            else:
                break
            rospy.sleep(1)

        if proc.returncode is not 0:
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
