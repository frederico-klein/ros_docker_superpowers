#!/usr/bin/env python
# -*- coding: utf-8 -*-

### this should be a bunch of shell scripts. IDK what I am doing.

## creates the bridge.

import rospy
from utils import DockerLogged
from docker_master import DockerMasterInterface as DMI

class Tub(DockerLogged):
    """
    This can be invoked with something like:
    from DockerTub import Tub

    myTub = Tub(name="tub0")
    myTub.build()
    myTub.start()

    or launched as a node with parameters from a launch file, with the same names.

    """
    def __init__(self,
            name=                   "tub0"          ,
            imagename=              "tch_new"       ,
            network_name=           "br0"           ,
            tub_volume=             "tubvolume0"    ,
            hostname=               "torch_machine4",
            dockerfile_directory=   "."             ,
            machineip=              "172.28.6.31"   ):
        super(Tub, self).__init__()
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
        self.lspPopen(['docker','build',"-t",
            self.ImageName,
            self.DockerfileDirectory
            ])

    def create(self):
        ## I want to read the private parameters here, since I already started the node, so I catkin_ws
        ## this is the best I could do with python 2.7 and my python fu

        self.afps("Name"        ,"name"          )
        self.afps("ImageName"   ,"imagename"     )
        self.afps("NetworkName" ,"network_name"  )
        self.afps("TubVolume"   ,"tub_volume"    )
        self.afps("MachineHostname"    ,"hostname")

        ###here I need to get the TubVolume's properies.
        self.WsPath = self.DMI.get_ws_volume_by_name(self.TubVolume)
        #self.Display = ":0"

        self.afps("DockerfileDirectory"  , "dockerfile_directory")
        self.afps("IP"                   , "machineip"           )

        rospy.loginfo("Mounting docker image {}".format(self.Name))

        ##check if there is a volume already
        output = self.oLspPopen(['docker','ps'])

        if self.Name in output: #if there isn't create one
            rospy.logwarn("found {} docker container found.".format(self.Name))
        else:

            ## I guess it cant be interactive this time. for interaction we will rely on sshd working
            rospy.loginfo("Docker container not found. Creating one. ")
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
            self.lspPopen(proc_list)
        rospy.on_shutdown(self.close)

    def close(self):
        rospy.loginfo("Starting shutting down sequence for {}.".format(self.Name))

        output = self.oLspPopen(['docker','ps'])

        if self.Name in output: #if there isn't create one
            rospy.loginfo("Found {} docker container found to be running. Now stopping... ".format(self.Name))
            ##docker stop kill or rm???
            self.lspPopenRetry(['docker','stop',self.Name])
            rospy.loginfo("Stop okay.")
        else:
            rospy.logerr("Docker container {} already closed!! There are issues with the container initialization or persistence.".format(self.Name))

        output = self.oLspPopen(['docker','container','ls','-a'])

        if self.Name in output: #if there isn't create one
            rospy.loginfo("Found {} docker container found. Now deleting... ".format(self.Name))
            ##docker stop kill or rm???
            self.lspPopenRetry(['docker','rm',self.Name])
            rospy.loginfo("Delete okay. Bye!")
        else:
            rospy.logerr("Unexpected. Docker container {} already deleted.".format(self.Name))

if __name__ == '__main__':
    try:
        myTub = Tub()
        myTub.create()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
