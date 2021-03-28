#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os, shutil
import rospkg
import rosparam
from DockerTub import Tub
from utils import logstack, wait_on_param
from std_srvs.srv import Empty
from threading import Lock

class DnsMasqTub(Tub):
    def __init__(self):
        super(DnsMasqTub, self).__init__(1.5, dns_check = False)
        self.lock = Lock()

    def post_init(self): ## makes sure the object always exists.
        super(DnsMasqTub, self).post_init()
        self.Ready = rospy.set_param("~Ready", True)

        self.rospack = rospkg.RosPack()
        self.updateHostSrv = rospy.Service('~upd_host', Empty, self.handle_update_host)

        self.afps("RosDnsMasqPort","ros_dnsmasq_port", default_attribute=10053)

        self.ros_msq_dir = "/tmp/ros_dnsmasq.d"
        if not os.path.exists(self.ros_msq_dir):
            os.makedirs(self.ros_msq_dir)
        rospy.logdebug("Using auxiliary dnsmasq in {}".format(self.ros_msq_dir))
        self.update_host_list(initial = True)
        self.DMI.register_dnsmasq(self.IP)

    def handle_update_host(self,req):
        rospy.logwarn("upd_host service called")
        self.update_host_list()
        return []

    def get_own_volumes(self):
        return   ["-v", "{}:/etc/dnsmasq.conf".format(self.dnsmasqfile)]

    def get_entrypoint(self):
        return []

    def update_host_list(self, initial = False):
        #self.DMI.update_hosts()
        self.generate_dnsmasqconf(init = initial)
        self.restart_dnsmasq_docker()

    def generate_dnsmasqconf(self, init = False):
        self.dnsmasqfile = self.ros_msq_dir + "/dnsmasq.conf"
        rospy.logdebug("generating file {}".format(self.dnsmasqfile))
        shutil.copyfile(self.rospack.get_path('rosdop') + "/dnsmasq/ros-tmp-hosts", dst=self.dnsmasqfile)

        self.DMI.wait_on_param("Ready", message = "Waiting for master ready flag to be set to true.", check_if = True)

        self.DMI.update_from_master()
        host_list = []
        if not init:
            rospy.logwarn(self.DMI.master.docker_hosts)
            rospy.logwarn(self.DMI.master.HostDic)
            host_list.extend(generate_string_list_for_dnsmasqfile_from_docker_hosts(self.DMI.master.docker_hosts))
            host_list.extend(generate_string_list_for_dnsmasqfile_from_HostDic(self.DMI.master.HostDic))
        #address=/torch_machine4.poop/172.28.5.31
        with open(self.dnsmasqfile, "a") as myfile:
            for add_expression in host_list:
                #add_expression = "address=/torch_machine4.poop/172.28.5.31"
                rospy.logdebug("added:\n\t{}".format(add_expression))
                myfile.write("{}\n".format(add_expression))
                # myfile.write("address=/torch_machine4.poop/172.28.5.31")


    def restart_dnsmasq_docker(self):
        ##if there is a container running as dnsmasq I have to stop it
        ###this does not really work.
        wait_on_param("Ready", check_if = True)
        self.Ready = False
        rospy.set_param("Ready", self.Ready)
        self.lock.acquire()

        logstack()
        rospy.loginfo("=========RESET ISSUED========")
        rospy.loginfo("=========NOW CLOSING DNS========")
        self.close(silent = False, reset = True)
        #self.close(silent = True, reset = True)
        rospy.loginfo("=========NOW CREATING DNS AGAIN========")
        self.create(ready_flag = "Ready")
        rospy.loginfo("=========ALL DONE========")
        self.lock.release()

        self.Ready = True
        rospy.set_param("Ready", self.Ready)

    def get_dns(self): ##I am providing the dns here. we don't want a loop
        return []

def generate_string_list_for_dnsmasqfile_from_docker_hosts(docker_hosts):
    string_list = []
    suffix_list = [".Home", ".local", ""]
    for key, value in docker_hosts.iteritems():
        for suffix in suffix_list:
            string_list.append("address=/{}{}/{}".format(key,suffix,value))
    return string_list

def generate_string_list_for_dnsmasqfile_from_HostDic(HostDic):
    string_list = []
    for suffix, machinesDic in HostDic.iteritems():
        for machine, ip in machinesDic.iteritems():
            string_list.append("address=/{}.{}/{}".format(machine,suffix,ip))
    return string_list

if __name__ == '__main__':
    try:
        dnsmasqTub = DnsMasqTub()
        dnsmasqTub.post_init()
        rospy.loginfo("RosDnsMasq seems OK.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if dnsmasqTub.created:
            dnsmasqTub.close()
