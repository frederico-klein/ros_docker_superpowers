#!/usr/bin/env python
# -*- coding: utf-8 -*-

### this should be a bunch of shell scripts. IDK what I am doing.

## creates the bridge.

import rospy
import subprocess
from utils import attribute_from_param_setter as afps
from docker_master import DockerMasterInterface as DMI

class Tub():
    """
    This can be invoked with something like:
    from DockerTub import Tub

    myTub = Tub(name="tub0")
    myTub.build()
    myTub.start()

    or launched as a node with parameters from a launch file, with the same names.

    """
    def __init__(self,
            name="tub0",
            network_name="br0",
            imagename="tch_new",
            tub_volume="tubvolume0",
            hostname="torch_machine4",
            dockerfile_directory=".",
            machineip="172.28.6.31"):
        self.Name = name
        self.NetworkName = network_name
        self.ImageName = imagename
        self.TubVolume = tub_volume
        self.MachineHostname = hostname
        self.DockerfileDirectory = dockerfile_directory
        self.IP = machineip

        self.DMI = DMI()
        rospy.init_node('docker_tub', anonymous=True, log_level=rospy.DEBUG)

    def updateBuild(self):
        ## this no longer works for some builds where I wanted nvidia to be present during the build to detect the graphics card and know what to do. I am not 100% sure it ever worked, so it is a maybe on the todo list
        rospy.loginfo("Starting build process...")
        proc = subprocess.Popen(['docker','build',"-t",
        self.ImageName,
        self.DockerfileDirectory
        ], stdout=subprocess.PIPE)

    def create(self):
        ## I want to read the private parameters here, since I already started the node, so I catkin_ws
        ## this is the best I could do with python 2.7 and my python fu

        afps(self,"name","Name")
        afps(self,"imagename","ImageName")
        afps(self,"network_name","NetworkName")
        afps(self,"tub_volume","TubVolume")

        ###here I need to get the TubVolume's properies.
        self.WsPath = self.DMI.get_ws_volume_by_name(self.TubVolume)
        #self.Display = ":0"

        afps(self,"dockerfile_directory","DockerfileDirectory")
        afps(self,"machineip","IP")

        rospy.loginfo("Mounting docker image {}".format(self.Name))

        ##check if there is a volume already
        proc = subprocess.Popen(['docker','ps'], stdout=subprocess.PIPE)
        output = proc.stdout.read()

        if self.Name in output: #if there isn't create one
            rospy.loginfo("found {} docker image.".format(self.Name))
        else:

            ## I guess it cant be interactive this time. for interaction we will rely on sshd working
            rospy.loginfo("Hello")
            proc_list = ['docker','run',"--gpus",'"device=0"',
                 #"--rm",
                 #"-it",
                 "--name", self.Name,
                 "-t","-d",
                 "-u","root",
                 #"-e","DISPLAY=$DISPLAY",
                 #"-v","/tmp/.X11-unix:/tmp/.X11-unix",
                 "-v","{}:{}".format(self.TubVolume,self.WsPath),
                 "-v","/mnt/share:/mnt/share",
                 "-h","{}".format(self.MachineHostname),
                 "--network={}".format(self.NetworkName),
                 "--ip={}".format(self.IP),
                 self.ImageName,
                 "bash"
             ]
            rospy.logdebug(proc_list)
            proc = subprocess.Popen(proc_list, stdout=subprocess.PIPE)
            output = proc.stdout.read()
            rospy.logdebug(output)
        rospy.on_shutdown(self.close)

    def close(self):
        rospy.loginfo("Shutting down. Stopping container {}".format(self.Name))
        ##docker stop kill or rm???
        procstop = subprocess.Popen(['docker','stop',self.Name], stdout=subprocess.PIPE)
        outputstop = procstop.stdout.read()
        rospy.logdebug(outputstop)
        rospy.loginfo("Deleting container {}".format(self.Name))
        procDel = subprocess.Popen(['docker','rm',self.Name], stdout=subprocess.PIPE)
        outputDel = procDel.stdout.read()
        rospy.logdebug(outputDel)

if __name__ == '__main__':
    try:
        myTub = Tub()
        myTub.create()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
