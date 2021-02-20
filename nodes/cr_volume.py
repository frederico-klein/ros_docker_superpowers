#!/usr/bin/env python
# -*- coding: utf-8 -*-

### this should be a bunch of shell scripts. IDK what I am doing.

## creates the volume.

import rospy
import subprocess
from docker_master import DockerMasterInterface as DMI
#from utils import attribute_from_param_setter as afps
#from utils import logged_subprocess_popen as lspPopen
from utils import DockerLogged

# class TubVolumeInterface():
#     def __init__(self):
#         pass
#     def get_volume_by_name(self):
#         tries = 10
#         while (tries > 0):
#             for node in rosnode.get_node_names():
#                 if "docker_master" in node:
#                     rospy.loginfo("Found master")
#                     return node
#         pass

class TubVolume(DockerLogged):
    """
    This is a vieux sshfs wrapper. I am calling it tub volume because it will mount always in the same path in a docker host thing (which I am calling a Tub)
    This can be invoked with something like:
    from cr_volume import TubVolume

    myTubVolume = TubVolume(name="sshvolume-workspace-torch_new0",
            username="fred",
            sshfs_hostname="serverino",
            identity_file="/home/fred/.ssh",
            tub_path = "/home/fred/whole_serverino/tub/Tch")
    myTubVolume.create()

    or launched as a node with parameters from a launch file, with the same names.

    """
    def __init__(self,
            name="sshvolume-workspace-torch_new899",
            username="frederico",
            sshfs_hostname="poop.local", ##the name of the machine that has the sshfs path we want to share
            identity_file="/root/.ssh/id_rsa",
            tub_path = "./", ##no idea how to set this correctly, probably only as a param
            ws_path = "/workspace"
            ):
        super(TubVolume, self).__init__()
        rospy.init_node('docker_volume', anonymous=True)#, log_level=rospy.DEBUG)
        rospy.loginfo("Docker Volume spawner node started. ")
        self.Name = name
        self.Driver = "vieux/sshfs"
        self.UserName = username
        self.IdentityFile = identity_file
        self.SshfsHostname = sshfs_hostname
        self.TubPath = tub_path
        self.WsPath = ws_path
        ###attempts to find docker_master
        self.DMI = DMI(1)

        ##Now I will register myself with the master
        #rospy.get_param("{}/TubVolumeDic".format(self.master))
        rospy.logdebug("Added DMI OK.")

    def open(self):
        ## I want to read the private parameters here, since I already started the node, so I catkin_ws

        #self.Name = rospy.get_param('~name', default = self.Name)
        #rospy.logdebug('Parameter %s has value %s', rospy.resolve_name('~name'), self.Name)

        # self.UserName = rospy.get_param('~username', default = self.UserName)
        # rospy.logdebug('Parameter %s has value %s', rospy.resolve_name('~username'), self.UserName)
        #
        # self.IdentityFile = rospy.get_param('~identity_file', default = self.IdentityFile)
        # rospy.logdebug('Parameter %s has value %s', rospy.resolve_name('~identity_file'), self.IdentityFile)
        #
        # self.SshfsHostname = rospy.get_param('~sshfs_hostname', default = self.SshfsHostname)
        # rospy.logdebug('Parameter %s has value %s', rospy.resolve_name('~sshfs_hostname'), self.SshfsHostname)
        #
        # self.TubPath = rospy.get_param('~tub_path', default = self.TubPath)
        # rospy.logdebug('Parameter %s has value %s', rospy.resolve_name('~tub_path'), self.TubPath)

        self.afps("Name", "name")
        self.afps("UserName", "username")
        self.afps("IdentityFile", "identity_file")
        self.afps("SshfsHostname", "sshfs_hostname")
        self.afps("TubPath", "tub_path")
        self.afps("WsPath", "ws_path")

        self.DMI.addVolume(self.Name, self.WsPath)
        rospy.loginfo("Added volume to DMI OK.")

        rospy.loginfo("Creating volume {}".format(self.Name))
        ##check if there is a volume already

        #proc = subprocess.Popen(['docker','volume','ls'], stdout=subprocess.PIPE)
        #output = proc.stdout.read()
        output = self.lspPopen(['docker','volume','ls'])[0]

        if self.Name in output: #if there isn't create one
            rospy.loginfo("found {} docker volume.".format(self.Name))
        else:
            # I am unsure about this.
            list_args = [
                "docker","volume","create",
                "-d",self.Driver,
                "-o","sshcmd={}@{}:{}".format(self.UserName,
                self.SshfsHostname, self.TubPath),
                "-o","IdentityFile={}".format(self.IdentityFile),
                #"-o","password={}".format("some_elusive_password"),
                self.Name]
            #rospy.logdebug("command being run:{}".format(list_args))
            #procTubVolumeCreate = subprocess.Popen(list_args, stdout=subprocess.PIPE)
            self.lspPopen(list_args)

        rospy.on_shutdown(self.close)

    def close(self):
        rospy.loginfo("Shutting down. Deleting volume {}".format(self.Name))
        try:
            self.DMI.rmVolume(self.Name)
        except:
            rospy.logwarn("Removing volume from docker master host list failed. Some docker services may be left dangling!")
        self.lspPopenRetry(['docker','volume','rm',self.Name])

if __name__ == '__main__':
    try:
        myTubVolume = TubVolume(
                name="tubvolume0",
                username="frederico",
                sshfs_hostname="poop.local", ##the name of the machine that has the sshfs path we want to share
                identity_file="/root/.ssh/id_rsa",
                tub_path = "/home/frederico/whole_poop/catkin_docker/src/dop_tch", ##no idea how to set this correctly
                ws_path = "/workspace"
                )
        myTubVolume.open()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


            # list_args = [
            # "docker","volume","create",
            # "-d",self.Driver,
            # "-o","sshcmd={}@{}:{}".format(self.UserName,self.SshfsHostname, self.TubPath),
            # "-o","IdentityFile={}".format(self.IdentityFile),
            # "-o","sshfs_debug",
            # #"-o","password={}".format("please_dont_put_your_password_here_like_ever"),
            # self.Name]
