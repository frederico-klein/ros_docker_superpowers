#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rosparam
import subprocess
import socket
import traceback

#import re

def logstack():
    message = " ".join(traceback.format_stack())
    try:
        rospy.logdebug_once(message) ##I think this is new.
    except:
        rospy.logdebug(message)
    return message

def flatten(nl):
    listA = []
    for item in nl:
        if isinstance(item,list):
            listA.extend(flatten(item))
        else:
            listA.append(item)
    return listA

class ShellExecutionError(Exception):
    """Base class for exceptions in this module."""
    def __init__(self, message, errorout):
        self.message = message
        self.errout = errorout
        rospy.logerr(self.message)
        rospy.logerr(self.errout)
        rospy.signal_shutdown(self.errout)

def wait_on_param(param, message = "No message given.", tries = 100, check_if_inside = None, check_if = None):
    num_tries = tries
    outparam = None
    rate = rospy.Rate(1)
    realparamname = "~{}".format(param)
    rospy.logdebug("Local node wait_on_param reached")
    rospy.logdebug(realparamname)
    def retry(tries):
        rospy.logdebug(message)
        tries -= 1
        rate.sleep()
        ##it's useless to retry if master is dead, so:
        #if param is not "Alive" and rospy.get_param("~Alive") == False:
        #    rospy.signal_shutdown("Node process ended. Closing...")
        return tries
    while (num_tries> 0):
        try:
            rospy.logdebug("trying to read param: {}".format(realparamname))
            outparam = rospy.get_param(realparamname)
            rospy.logdebug("{}: {}".format(realparamname, outparam))
            #rospy.logdebug("all params: {}".format(rosparam.list_params("/")))
            if check_if_inside is None :
                if outparam is check_if:
                    rospy.logdebug("condition met. ")
                    break
                elif check_if is None:
                    rospy.logdebug("parameter set. ")
                    break
                else:
                    num_tries = retry(num_tries)
            elif check_if_inside in outparam :
                rospy.logdebug("found it. ")
                break
            else:
                num_tries = retry(num_tries)
        except rospy.ROSException as e: ## maybe it will collide with the thing on top, needs checking.
            rospy.logerr("Unexpected! {}".format(e))
        except rospy.ROSInterruptException:
            break

    if outparam is None:
        rospy.logerr("Could not get param: {}".format(realparamname))
    return outparam


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
        message = ""
        output = ""
        errorout = ""
        command_str = " ".join(list_args)
        while tries>0:
            rospy.logdebug("command being run:\n\n{}\n\n".format(command_str))
            proc = subprocess.Popen(list_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            proc.wait()
            output, errorout = proc.communicate()
            #res = proc.communicate()
            #output = res[0]
            #errorout = res[1] # I think
            # output = proc.stdout.read()

            rospy.logdebug("process response: {}".format(output))

            tries-=1
            if with_retries:
                if proc.returncode is 0:
                    break
                else:
                    rospy.logdebug("COMMAND: \n{}\n Did not work yet. Maybe try again in a sec?\nRESPONSE stdout: {}\nerrout: {}".format(list_args, output, errorout))
                    if proc.returncode is not 0:
                         message = logstack()
            else:
                break
            #rospy.sleep(1)

        if proc.returncode is not 0:
            raise ShellExecutionError(message, errorout)
        rospy.loginfo("\nlspPopen Command:\n{}\noutput:\n{}".format(command_str,output))
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
