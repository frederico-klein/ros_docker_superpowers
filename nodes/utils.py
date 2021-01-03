#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

#import re

def attribute_from_param_setter(owner, private_name, classatrname):
  default_attribute = getattr(owner,  classatrname)
  setattr(owner, classatrname, rospy.get_param('~{}'.format(private_name), default = default_attribute))#
  updated_param = getattr(owner,  classatrname)
  rospy.logdebug('Parameter %s has value %s', rospy.resolve_name('~{}'.format(private_name)), updated_param)
  # I want to keep this updated because I will use these attributes for interprocess talk. it's just easier like this.
  rospy.set_param('~{}'.format(private_name), updated_param)
