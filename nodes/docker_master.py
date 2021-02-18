#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy
import rosnode
import rosparam

from utils import DockerLoggedNamed
from rosdop.srv import addVolume, addVolumeResponse
from rosdop.srv import RmVolume, RmVolumeResponse
from rosdop.srv import addDockerMachine, addDockerMachineResponse
from rosdop.srv import RmDockerMachine, RmDockerMachineResponse

class Error(Exception):
    """Base class for exceptions in this module."""
    pass

class NoMaster(Error):
    """Exception raised for errors in the input.

    Attributes:
        expression -- input expression in which the error occurred
        message -- explanation of the error
    """

    def __init__(self):
        self.message = "No Master"

class DockerMasterInterface():
    def __init__(self):
        self.master = DockerMaster()
        self.master_handle = self.get_master()

        if self.master_handle is not None:
            rospy.wait_for_service('{}/add_volume'.format(self.master_handle))
            try:
                self.master.TubVolumeDic = rosparam.get_param("{}/TubVolumeDic".format(self.master_handle))
                self.master.HostDic = rosparam.get_param("{}/HostDic".format(self.master_handle))
                self.add_vol = rospy.ServiceProxy('{}/add_volume'.format(self.master_handle), addVolume)
                self.rm_vol = rospy.ServiceProxy('{}/rm_volume'.format(self.master_handle), RmVolume)
                self.add_host = rospy.ServiceProxy('{}/add_host'.format(self.master_handle), addDockerMachine)
                self.rm_host = rospy.ServiceProxy('{}/rm_host'.format(self.master_handle), RmDockerMachine)

            except rospy.ServiceException as e:
                rospy.logfatal("Service call failed: %s"%e)
                raise Exception

        rospy.loginfo("DMI init OK. ")

    def get_ws_volume_by_name(self,name):

        try:
            ### if everything is loaded at once this thing here is failing. I probably need to add some more clever stops somewhere else. Now I will make it sleep
            tries = 10
            while tries >0:
                ###It doesnt make sense not to update this. Idk what I was thinking
                self.master.TubVolumeDic = rosparam.get_param("{}/TubVolumeDic".format(self.master_handle))
                if len(self.master.TubVolumeDic) != 0 and name in self.master.TubVolumeDic:
                    break
                else:
                    tries -= 1
                    rospy.logwarn("Volume List is empty or does not contain desired volume. Have you mounted a volume yet?")
                    time.sleep(3)
        except rospy.ROSException as e:
            rospy.logerr("Unexpected! {}".format(e))

        return self.master.TubVolumeDic[name]

    def get_ws_host_by_name(self,name):

        try:
            ### if everything is loaded at once this thing here is failing. I probably need to add some more clever stops somewhere else. Now I will make it sleep
            tries = 10
            while tries >0:
                ###It doesnt make sense not to update this. Idk what I was thinking
                self.master.HostDic = rosparam.get_param("{}/HostDic".format(self.master_handle))
                if len(self.master.HostDic) != 0 and name in self.master.HostDic:
                    break
                else:
                    tries -= 1
                    rospy.logwarn("Volume List is empty or does not contain desired host. Have you initiated a host yet?")
                    time.sleep(3)
        except rospy.ROSException as e:
            rospy.logerr("Unexpected! {}".format(e))

        return self.master.HostDic[name]


    def get_master(self):
        tries = 10
        while (tries > 0):
            for node in rosnode.get_node_names():
                if "docker_master" in node:
                    rospy.loginfo("Found master. Waiting for it to finish loading...")
                    try:
                        if rosparam.get_param("{}/Ready".format(node)):
                            rospy.loginfo("Master alive, continuing.")
                            return node
                    except:
                        tries -= 1
                        rospy.logwarn("Docker Master did not finish initializing. Will retry {} more times.".format(tries))
                        time.sleep(3)
            else:
                tries -= 1
                rospy.logwarn("Docker Master not found yet. Is it running? Will retry {} more times.".format(tries))
                time.sleep(3)

        if "docker_master" not in rosnode.get_node_names():
            rospy.logfatal("Didn't find master. ")
            raise NoMaster()
            #print("hello")
            #self.master = DockerMaster()
            ## let's wait for the dockerMaster services here:

            return None
    def addVolume(self,VolumeName, WsPath):
        rospy.loginfo("Service add volume called.")
        self.add_vol(VolumeName, WsPath)
        self.master.TubVolumeDic = rosparam.get_param("{}/TubVolumeDic".format(self.master_handle))

    def rmVolume(self,VolumeName):
        rospy.loginfo("Service rm volume called.")
        self.rm_vol(VolumeName)
        self.master.TubVolumeDic = rosparam.get_param("{}/TubVolumeDic".format(self.master_handle))

    def addHost(self,TubName, HostName, IP):
        rospy.loginfo("Service add host called.")
        self.add_host(TubName, HostName, IP)
        self.master.HostDic = rosparam.get_param("{}/HostDic".format(self.master_handle))

    def rmHost(self,TubName, HostName):
        rospy.loginfo("Service rm host called.")
        self.rm_host(TubName, HostName)
        self.master.HostDic = rosparam.get_param("{}/HostDic".format(self.master_handle))

class DockerMaster(DockerLoggedNamed):
    """
    This is the node that control other docker resources.
    If you want to deal with multi-master, then you need to assign each resource
    to a master or implement a default thing. I haven't done it, so there is a
    single master.
    """
    TubVolumeDic = {}
    HostDic = {}
    HostName = ""
    UseDnsMasq = False

    def __init__(self):
        super(DockerMaster, self).__init__()

    def startup(self):
        rospy.init_node('docker_master', anonymous=False, log_level=rospy.DEBUG)
        self.updateHostName()

        ### I will want to keep a list of volumes, nodes and bridges.
        #OR I can just get them from the docker tools which already do all of that?
        # then again I can maybe have a bunch of stuff running everywhere?
        # occam's razor tells me all of that is bollocks and I should implement when I need it.
        # I am probably going to read this line over and be upset (why didn't I do this sooner?)
        # because you didn't know you were going to need it.

        ##Right now I just want a list of Tubvolumes:
        #self.TubVolumeDic = {}
        rospy.set_param("~TubVolumeDic",self.TubVolumeDic)
        rospy.set_param("~HostDic",self.HostDic)
        self.addVolumeSrv = rospy.Service('~add_volume', addVolume, self.handle_add_volume)
        self.rmVolumeSrv = rospy.Service('~rm_volume', RmVolume, self.handle_rm_volume)
        self.addHostSrv = rospy.Service('~add_host', addDockerMachine, self.handle_add_host)
        self.rmHostSrv = rospy.Service('~rm_host', RmDockerMachine, self.handle_rm_host)
        #rospy.set_param("~TubVolumeDic",[])
        rospy.on_shutdown(self.close)
        rospy.set_param("~Ready", True)

    def handle_add_volume(self,req):
        rospy.loginfo("Adding Volume {}:{} to list".format(req.VolumeName,  req.WsPath))
        rospy.logdebug("Current volume dictionary: before adding %s"%(self.TubVolumeDic))
        self.TubVolumeDic[req.VolumeName] = req.WsPath
        rospy.logdebug("Current volume dictionary: %s"%(self.TubVolumeDic))
        rospy.set_param("~TubVolumeDic",self.TubVolumeDic)
        return addVolumeResponse()

    def handle_rm_volume(self,req):
        rospy.loginfo("Removing Volume {} from list".format(req.VolumeName))
        self.TubVolumeDic.pop(req.VolumeName)
        rospy.set_param("~TubVolumeDic",self.TubVolumeDic)
        return RmVolumeResponse()

    def handle_add_host(self,req):
        rospy.loginfo("Adding Ros Docker Host (tub) {}:{}, {} to list".format(req.TubName, req.HostName,  req.IP))
        rospy.logdebug("Current host dictionary: before adding %s"%(self.HostDic))
        ###there is maybe a clever way of doing this, but I am not having it.
        if req.HostName in self.HostDic:
            self.HostDic[req.HostName].update({req.TubName: req.IP})
        else:
            self.HostDic.update({req.HostName:{req.TubName: req.IP}})
        #self.HostDic[req.HostName].update({req.TubName: req.IP})
        rospy.logdebug("Current host dictionary: %s"%(self.HostDic))
        rospy.set_param("~HostDic",self.HostDic)
        return addDockerMachineResponse()

    def handle_rm_host(self,req):
        rospy.loginfo("Removing Ros Docker Host (tub) {} from {} list".format(req.TubName, req.HostName))
        self.HostDic[req.HostName].pop(req.TubName)
        rospy.set_param("~HostDic",self.HostDic)
        return RmDockerMachineResponse()

    def close(self):
        rospy.loginfo("Shutting down. Waiting for other processes to close") ## this is a lie and it will fail if anything takes less than N seconds to close. I need to actually do this, hook them and then get the closure notice
        time.sleep(3)

if __name__ == '__main__':
    try:
        master = DockerMaster()
        master.startup()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
