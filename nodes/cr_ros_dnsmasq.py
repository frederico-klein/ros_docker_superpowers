#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os, shutil
import rospkg
from DockerTub import Tub
from std_srvs.srv import Empty

class DnsMasqTub(Tub):
    def __init__(self):
        super(DnsMasqTub, self).__init__(1.5, dns_check = False)
        self.rospack = rospkg.RosPack()
        self.updateHostSrv = rospy.Service('~upd_host', Empty, self.handle_update_host)

        self.afps("RosDnsMasqPort","ros_dnsmasq_port", default_attribute=10053)

        self.ros_msq_dir = "/tmp/ros_dnsmasq.d/"
        if not os.path.exists(self.ros_msq_dir):
            os.makedirs(self.ros_msq_dir)
        rospy.logdebug("Using auxiliary dnsmasq in {}".format(self.ros_msq_dir))
        self.update_host_list()
        self.DMI.register_dnsmasq(self.IP)

    def handle_update_host(self,req):
        rospy.logwarn("upd_host service called")
        self.update_host_list()
        return []

    def get_own_volumes(self):
        return   ["-v", "{}:/etc/dnsmasq.conf".format(self.dnsmasqfile)]

    def get_entrypoint(self):
        return []

    def update_host_list(self):
        #self.DMI.update_hosts()
        self.generate_dnsmasqconf()
        self.restart_dnsmasq_docker()

    def generate_dnsmasqconf(self):
        self.dnsmasqfile = self.ros_msq_dir + "/dnsmasq.conf"
        rospy.logdebug("generating file {}".format(self.dnsmasqfile))
        shutil.copyfile(self.rospack.get_path('rosdop') + "/dnsmasq/ros-tmp-hosts", dst=self.dnsmasqfile)
        #address=/torch_machine4.poop/172.28.5.31
        with open(self.dnsmasqfile, "a") as myfile:
            add_expression = "address=/torch_machine4.poop/172.28.5.31"
            rospy.logdebug("added:\n\t{}".format(add_expression))
            myfile.write(add_expression)
            # myfile.write("address=/torch_machine4.poop/172.28.5.31")


    def restart_dnsmasq_docker(self):
        ##if there is a container running as dnsmasq I have to stop it
        self.close(silent = True, reset = True)
        self.create()

    def get_dns(self): ##I am providing the dns here. we don't want a loop
        return []

if __name__ == '__main__':
    try:
        dnsmasqTub = DnsMasqTub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
