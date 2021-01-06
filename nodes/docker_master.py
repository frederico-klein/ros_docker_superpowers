#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy
import rosnode
import rosparam
from rosdop.srv import addVolume, addVolumeResponse
from rosdop.srv import RmVolume, RmVolumeResponse

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
        self.master.TubVolumeDic = rosparam.get_param("//docker_master/TubVolumeDic")

        if self.master_handle is not None:
            rospy.wait_for_service('{}/add_volume'.format(self.master_handle))
            try:
                self.add_vol = rospy.ServiceProxy('{}/add_volume'.format(self.master_handle), addVolume)
                self.rm_vol = rospy.ServiceProxy('{}/rm_volume'.format(self.master_handle), RmVolume)

            except rospy.ServiceException as e:
                rospy.logfatal("Service call failed: %s"%e)

        rospy.loginfo("DMI init OK. ")

    def get_ws_volume_by_name(self,name):
        ### if everything is loaded at once this thing here is failing. I probably need to add some more clever stops somewhere else. Now I will make it sleep
        tries = 10
        while tries >0:

            if len(self.master.TubVolumeDic) != 0 and name in self.master.TubVolumeDic:
                break
            else:
                tries -= 1
                rospy.logwarn("Volume List is empty or does not contain desired volume. Have you mounted a volume yet?")
                time.sleep(3)

        return self.master.TubVolumeDic[name]

    def get_master(self):
        tries = 10
        while (tries > 0):
            for node in rosnode.get_node_names():
                if "docker_master" in node:
                    rospy.loginfo("Found master")
                    return node
            else:
                tries -= 1
                rospy.logwarn("Docker Master not found yet. Is it running? Will retry {} more times.".format(tries))
                time.sleep(0.3)

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
        self.master.TubVolumeDic = rosparam.get_param("//docker_master/TubVolumeDic")

    def rmVolume(self,VolumeName):
        rospy.loginfo("Service rm volume called.")
        self.rm_vol(VolumeName)
        self.master.TubVolumeDic = rosparam.get_param("//docker_master/TubVolumeDic")

class DockerMaster():
    """
    This is the node that control other docker resources.
    If you want to deal with multi-master, then you need to assign each resource
    to a master or implement a default thing. I haven't done it, so there is a
    single master.
    """
    TubVolumeDic = {}
    def startup(self):
        rospy.init_node('docker_master', anonymous=False)
        ### I will want to keep a list of volumes, nodes and bridges.
        #OR I can just get them from the docker tools which already do all of that?
        # then again I can maybe have a bunch of stuff running everywhere?
        # occam's razor tells me all of that is bollocks and I should implement when I need it.
        # I am probably going to read this line over and be upset (why didn't I do this sooner?)
        # because you didn't know you were going to need it.

        ##Right now I just want a list of Tubvolumes:
        #self.TubVolumeDic = {}
        rospy.set_param("~TubVolumeDic",self.TubVolumeDic)
        self.addVolumeSrv = rospy.Service('~add_volume', addVolume, self.handle_add_volume)
        self.rmVolumeSrv = rospy.Service('~rm_volume', RmVolume, self.handle_rm_volume)
        #rospy.set_param("~TubVolumeDic",[])
        rospy.on_shutdown(self.close)

    def handle_add_volume(self,req):
        rospy.loginfo("Adding Volume {}:{} to list".format(req.VolumeName,  req.WsPath))
        self.TubVolumeDic[req.VolumeName] = req.WsPath
        rospy.set_param("~TubVolumeDic",self.TubVolumeDic)
        return addVolumeResponse()

    def handle_rm_volume(self,req):
        rospy.loginfo("Removing Volume {} from list".format(req.VolumeName))
        self.TubVolumeDic.pop(req.VolumeName)
        rospy.set_param("~TubVolumeDic",self.TubVolumeDic)
        return RmVolumeResponse()

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
