#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rosnode
import rosparam
from rosgraph.masterapi import MasterError

from utils import DockerLoggedNamed
from std_srvs.srv import Empty, EmptyResponse
from rosdop.srv import addVolume, addVolumeResponse
from rosdop.srv import RmVolume, RmVolumeResponse
from rosdop.srv import addDockerMachine, addDockerMachineResponse
from rosdop.srv import RmDockerMachine, RmDockerMachineResponse
#from rosdop.srv import GenericString, GenericStringResponse
from rosdop.srv import DMILevel, DMILevelResponse

import socket
import dns.resolver
#
# # Basic query
# for rdata in dns.resolver.query('www.yahoo.com'):
#     print rdata.target
#
# # Set the DNS Server
# resolver = dns.resolver.Resolver()
# resolver.nameservers=[socket.gethostbyname('ns1.cisco.com')]
# for rdata in resolver.query('www.yahoo.com') :
#     print rdata.target

###TODO: maybe use bond?

class Error(Exception):
    """Base class for exceptions in this module."""
    pass

class UnknownError(Error):
    def __init__(self, message):
        rospy.signal_shutdown(message)

class NoMaster(Error):
    """Exception raised for errors in the input.

    Attributes:
        expression -- input expression in which the error occurred
        message -- explanation of the error
    """

    def __init__(self):
        self.message = "No Master"
        rospy.signal_shutdown(self.message)



class DockerMasterInterface():

    def safeServiceCall(self, serv, arg_list):
        try:
            serv(*arg_list)
        except rospy.ServiceException as e:
            ## why can't it do this?
            ##maybe master is dead. parameters stay in the core after a node has signed off, so we can check for this!
            if self.wait_on_param("Alive", check_if = False):
                rospy.signal_shutdown("Master process ended. Closing...")
            else:
                ##then I don't know, so we raise
                raise UnknownError("Some ServiceException: %s"%e)

    def update_from_master(self):
        ###all parameters
        all_pars = rosparam.get_param(self.master_handle)
        rospy.logdebug("all parameters: {}".format(all_pars))
        for par,value in all_pars.iteritems():
            rospy.logdebug("{},{}".format(par,value))
            setattr(self.master, par, value)
        rospy.logdebug(dir(self.master))

    def signal_death(self):
        rospy.logwarn("signal_death not initialized but called. you have an inconsistent behaviour in your code!")

    def wait_on_param(self, param, message = "No message given.", tries = 100, check_if_inside = None, check_if = None):
        outparam = None
        realparamname = "{}/{}".format(self.master_handle, param)
        rospy.logdebug("wait_on_param reached")
        rospy.logdebug(realparamname)
        while (tries> 0):
            try:
                rospy.logdebug("trying to read param: {}".format(realparamname))
                outparam = rosparam.get_param(realparamname)
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
                        rospy.logdebug(message)
                        tries -= 1
                        self.rate.sleep()
                elif check_if_inside in outparam :
                    rospy.logdebug("found it. ")
                    break
                else:
                    rospy.logdebug(message)
                    tries -= 1
                    self.rate.sleep()
            except MasterError as ma:
                ## this is fine. it means the parameter is not there yet, which we expect
                tries -= 1
                rospy.logdebug("waiting on param {}".format(realparamname))
                rospy.logdebug(message)
                self.rate.sleep()
            except rospy.ROSException as e: ## maybe it will collide with the thing on top, needs checking.
                rospy.logerr("Unexpected! {}".format(e))

        if outparam is None:
            rospy.logerr("Could not get param: {}".format(realparamname))
        return outparam

    def __init__(self, level, dns_check = True):
        self.master = DockerMaster()
        self.rate = rospy.Rate(1)
        self.master_handle = self.get_master()
        self.dnsmasqIP = None
        self.node_name = rospy.get_name()
        self.aliveLevel = level

        if self.master_handle is not None:
            rospy.wait_for_service('//{}/add_volume'.format(self.master_handle))
            try:
                self.wait_on_param("Ready", "Waiting for master ready flag to be set to true.", check_if = True)
                self.update_from_master()

                self.master.UseDnsMasq = rosparam.get_param("{}/UseDnsMasq".format(self.master_handle))
                self.master.TubVolumeDic = rosparam.get_param("{}/TubVolumeDic".format(self.master_handle))
                self.master.HostDic = rosparam.get_param("{}/HostDic".format(self.master_handle))
                self.add_vol = rospy.ServiceProxy('{}/add_volume'.format(self.master_handle), addVolume)
                self.rm_vol = rospy.ServiceProxy('{}/rm_volume'.format(self.master_handle), RmVolume)
                self.add_host = rospy.ServiceProxy('{}/add_host'.format(self.master_handle), addDockerMachine)
                self.rm_host = rospy.ServiceProxy('{}/rm_host'.format(self.master_handle), RmDockerMachine)
                self.update_hosts = rospy.ServiceProxy('{}/upd_host'.format(self.master_handle), Empty)


                # self.register_dmi = rospy.ServiceProxy('{}/register_dmi'.format(self.master_handle), GenericString)
                # self.register_dmi(self.node_name)


                #mySrvCall = DMILevel()
                #mySrvCall.Level = level
                #mySrvCall.DMIName = self.node_name
                #self.register_dmi(mySrvCall)
                if self.aliveLevel >0:
                    rospy.logwarn("DMI registered as essential. Any error here will shutdown docker_master and all DMI linked nodes!")
                    self.signal_death = rospy.ServiceProxy('{}/die'.format(self.master_handle), Empty)
                    self.perishSrv = rospy.Service('~perish', Empty, self.close_handle)
                    self.register_dmi = rospy.ServiceProxy('{}/register_dmi'.format(self.master_handle), DMILevel)
                    self.register_dmi( self.node_name, level)
                    rospy.on_shutdown(self.close)
                else:
                    rospy.loginfo("DMI registered as non-essential.")


            except rospy.ServiceException as e:
                rospy.logfatal("Service call failed: %s"%e)
                raise Exception

        if dns_check:
            self.dnsmasqIP = self.wait_on_param("dnsmasqIP", "Waiting for dns to be present.")

        rospy.loginfo("DMI init OK. ")

    def register_dnsmasq(self, IP):
        self.dnsmasqIP = IP
        rosparam.set_param("{}/dnsmasqIP".format(self.master_handle), IP)
        rospy.loginfo("Registered dnsmasq server at {}".format(IP))

    def get_ws_volume_by_name(self,name, tries = 10):

        self.master.TubVolumeDic = self.wait_on_param("TubVolumeDic", message = "Volume List is empty or does not contain desired volume. Have you mounted a volume yet?",
                                                        tries = tries, check_if_inside = name)
        return self.master.TubVolumeDic[name]

    def get_ws_host_by_name(self,name, tries = 10):
        self.master.HostDic = self.wait_on_param("HostDic", message = "Volume List is empty or does not contain desired host. Have you initiated a host yet?",
                                                    tries = tries, check_if_inside = name)
        return self.master.HostDic[name]

    def get_master(self):
        tries = 10
        while (tries > 0):
            for node in rosnode.get_node_names():
                if "docker_master" in node:
                    rospy.loginfo("Found master in {}. Waiting for it to finish loading...".format(node))
                    try:
                        if rosparam.get_param("{}/Ready".format(node)):
                            rospy.loginfo("Master alive, continuing.")
                            return node
                    except:
                        tries -= 1
                        rospy.logwarn("Docker Master did not finish initializing. Will retry {} more times.".format(tries))
                        self.rate.sleep()
            else:
                tries -= 1
                rospy.logwarn("Docker Master not found yet. Is it running? Will retry {} more times.".format(tries))
                self.rate.sleep()

        rospy.loginfo("rosnodes are: {}".format(rosnode.get_node_names()))
        rospy.logfatal("Didn't find master. ")
        raise NoMaster()
            #print("hello")
            #self.master = DockerMaster()
            ## let's wait for the dockerMaster services here:

        return None
    def addVolume(self,VolumeName, WsPath):
        rospy.loginfo("Service add volume called.")
        self.safeServiceCall(self.add_vol, (VolumeName, WsPath))
        self.master.TubVolumeDic = rosparam.get_param("{}/TubVolumeDic".format(self.master_handle))

    def rmVolume(self,VolumeName):
        rospy.loginfo("Service rm volume called.")
        self.safeServiceCall(self.rm_vol, (VolumeName))
        self.master.TubVolumeDic = rosparam.get_param("{}/TubVolumeDic".format(self.master_handle))

    def addHost(self,TubName, HostName, IP):
        rospy.loginfo("Service add host called.")
        ## attempt at solving a run issue that happens in the initialization
        #hack
        if "dns" in TubName:
            rospy.logwarn("Attention `dns` is a protected word in tub names and will trigger a different check. This is expected to warn for the dns container, but for no other. If you want to get rid of this warning, either change name or update this checking function!")
            self.wait_on_param("Ready", check_if = True)
        else:
            self.wait_on_param("DNS_Ready", check_if = True)

        self.safeServiceCall(self.add_host, [TubName, HostName, IP])

        self.master.HostDic = rosparam.get_param("{}/HostDic".format(self.master_handle))

    def rmHost(self,TubName, HostName):
        rospy.loginfo("Service rm host called.")
        self.safeServiceCall(self.rm_host, [TubName, HostName])
        #self.rm_host(TubName, HostName)
        self.master.HostDic = rosparam.get_param("{}/HostDic".format(self.master_handle))

    def close_handle(self,req):
          try:
              rospy.signal_shutdown("Indirectly closed.")
          except:
              pass
          return EmptyResponse()

    def close(self):
        try:
            self.signal_death()
        except:
            rospy.logerr("Master already dead! This shouldn't happen")

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
    DockerHosts = {}

    def __init__(self):
        super(DockerMaster, self).__init__()
        self.DnsMasqNodeName = ""
        self.UseDnsMasq = False
        self.keep_alive = True
        self.dnsmasqIP = ""
        self.resolver = dns.resolver.Resolver()
        self.dns_ready = False
        self.DockerHosts = []

    def __enter__(self):
        rospy.init_node('docker_master', anonymous=False, log_level=rospy.DEBUG)
        rospy.set_param("~Alive", True)
        rospy.set_param("~Ready", False)
        rospy.set_param("~DNS_Ready", False)
        self.updateHostName()

        ### I will want to keep a list of volumes, nodes and bridges.
        #OR I can just get them from the docker tools which already do all of that?
        # then again I can maybe have a bunch of stuff running everywhere?
        # occam's razor tells me all of that is bollocks and I should implement when I need it.
        # I am probably going to read this line over and be upset (why didn't I do this sooner?)
        # because you didn't know you were going to need it.

        self.signal_kill_dmis = {}

        self.rate = rospy.Rate(1)

        rospy.set_param("~TubVolumeDic",self.TubVolumeDic)
        rospy.set_param("~dnsmasqIP",self.dnsmasqIP)
        rospy.set_param("~HostDic",self.HostDic)
        rospy.set_param("~docker_hosts",self.DockerHosts)
        ##TODO: cleanup, consistency!
        self.afps("UseDnsMasq","use_dnsmasq")
        rospy.set_param("~UseDnsMasq",self.UseDnsMasq)

        self.afps("DnsMasqNodeName","dnsmasq_node_name")

        self.addVolumeSrv = rospy.Service('~add_volume', addVolume, self.handle_add_volume)
        self.rmVolumeSrv = rospy.Service('~rm_volume', RmVolume, self.handle_rm_volume)
        self.addHostSrv = rospy.Service('~add_host', addDockerMachine, self.handle_add_host)
        self.rmHostSrv = rospy.Service('~rm_host', RmDockerMachine, self.handle_rm_host)
        self.updateHostSrv = rospy.Service('~upd_host', Empty, self.handle_update_host)
        self.DieSrv = rospy.Service('~die', Empty, self.die) ##this is simplistic it should be a list and then call everyone on the list to signal that I died.
        # self.registerDMI = rospy.Service('~register_dmi', GenericString, self.handle_register_dmi)
        self.registerDMI = rospy.Service('~register_dmi', DMILevel, self.handle_register_dmi)

        rospy.set_param("~Ready", True) ### this needs toi be before the setup_dnsmasq or it will enter in a deadlock

        if self.UseDnsMasq:
            self.setup_dnsmasq()

        # TODO: make parameter?
        self.resolver.nameservers=["192.168.0.1"]#[socket.gethostbyname('ns1.cisco.com')]

    def setup_dnsmasq(self):
        rospy.logwarn("dnsmasq set!")
        if not self.DnsMasqNodeName:
            rospy.logfatal("Using dnsmaq but param dnsmasq_node_name is not set! Which dnsmasq should I use?")
            raise Error

        dnsmasq_update_service_string_handle = '/{}/upd_host'.format(self.DnsMasqNodeName) ## this is global for now
        rospy.loginfo("Waiting for {} to be alive.".format(dnsmasq_update_service_string_handle))
        rospy.wait_for_service(dnsmasq_update_service_string_handle)
        self.update_hosts_DNS = rospy.ServiceProxy(dnsmasq_update_service_string_handle, Empty)
        self.dns_ready = True
        rospy.logwarn("dnsmasq finished setting up")
        rospy.set_param("~DNS_Ready", True)

    # def handle_register_dmi(self,req):
    #     signal_kill_dmis_string_handle = '{}/perish'.format(req.MyString)
    #     self.signal_kill_dmis.append( signal_kill_dmis_string_handle)
    #     # self.signal_kill_dmis.append( rospy.ServiceProxy(signal_kill_dmis_string_handle, Empty))
    #     rospy.logdebug("current list of linked DMIs: {}".format(self.signal_kill_dmis))
    #     rospy.set_param("~allDMIs",self.signal_kill_dmis)
    #     return GenericStringResponse()

    def handle_register_dmi(self,req):
        signal_kill_dmis_string_handle = '{}/perish'.format(req.DMIName)
        self.signal_kill_dmis[signal_kill_dmis_string_handle] = req.Level
        # self.signal_kill_dmis.append( rospy.ServiceProxy(signal_kill_dmis_string_handle, Empty))
        rospy.logwarn("current dic of linked DMIs: {}".format(self.signal_kill_dmis))
        rospy.set_param("~allDMIs",self.signal_kill_dmis)
        return DMILevelResponse()

    def handle_update_host(self,req):
        rospy.logwarn("update hosts called")
        self.update_hosts()
        rospy.logwarn("update hosts ended okay")
        return EmptyResponse()

    def update_hosts(self):
        HostList = {}
        rospy.logwarn("update host inner loop called")
        rospy.logwarn(self.HostDic)
        for hostName, tubIpDic in self.HostDic.iteritems():
            HostList[hostName] = self.resolver.query(hostName)[0].to_text()
            rospy.logdebug(HostList)
            #this fails often
            #HostList[hostName] = socket.gethostbyname(hostName)
        self.DockerHosts = HostList
        rospy.set_param("~docker_hosts", self.DockerHosts)
        if self.UseDnsMasq and self.dns_ready:
            try:
                self.update_hosts_DNS()
            except rospy.ServiceException as e:
                rospy.logdebug(e)
                rospy.logerr("Could not update the DNS. Is rosmasqdns working?")
                ## this happens when the DNS is already closed and we are trying to update the server.
        rospy.loginfo("Current known list of hosts: {}".format(self.DockerHosts))

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
        self.update_hosts()
        return addDockerMachineResponse()

    def handle_rm_host(self,req):
        rospy.loginfo("Removing Ros Docker Host (tub) {} from {} list".format(req.TubName, req.HostName))
        self.HostDic[req.HostName].pop(req.TubName)
        rospy.set_param("~HostDic",self.HostDic)
        self.update_hosts()
        return RmDockerMachineResponse()

    def die(self,req):
        rospy.logwarn("Die called. Shutting down sequence initiated")
        self.keep_alive = False
        return EmptyResponse()

    def kill_everything(self):
        rospy.logwarn("Kill everything called. Shutting down every docker node")
        better_dmi_kill_vec, level_list = parse_dic_into_tree(self.signal_kill_dmis)
        for level in level_list:
            for dieSrv in better_dmi_kill_vec[level]:
                try:
                    rospy.ServiceProxy(dieSrv, Empty)()
                except:
                    ##maybe already closed. I am not going to be too precise here, or I will be solving this problem forever.
                    rospy.logdebug("Could not end: {}\n\tIs it already finished?".format(dieSrv))

    def  __exit__(self, *exc):
        rospy.loginfo("Shutting down. Waiting for other processes to close") ## this is a lie and it will fail if anything takes less than N seconds to close. I need to actually do this, hook them and then get the closure notice
        self.keep_alive = False
        rospy.set_param("~Alive", False)

def parse_dic_into_tree(dic_dic):
    currlevel = 0
    tree = {}
    currlevel_list = []
    while dic_dic:
        while currlevel >=0:
            for item,level in dic_dic.iteritems():
                currlevel = max([currlevel,level])
            currlevel_list.append(currlevel)
            for item,level in dic_dic.iteritems():
                #print(tree)
                if level == currlevel:
                    thislevel = dic_dic.pop(item)
                    if thislevel in tree:
                        ##already has element in this level, then append to list
                        tree[thislevel].append(item)
                    else: ## create a list with one element
                        tree[thislevel] = [item]
                    currlevel=0
                    break
            break
    level_list = list(set(currlevel_list))
    level_list.reverse()
    return tree, level_list

#aa = {"a":1,"b":2,"c":2,"d":3,"e":1}
#parse_dic_into_tree(aa)

if __name__ == '__main__':
    master = DockerMaster()
    #with DockerMaster() as master:
    with master:
        while(True):
            master.rate.sleep()
            if not master.keep_alive:
                master.kill_everything()
                master.rate.sleep()
                break
