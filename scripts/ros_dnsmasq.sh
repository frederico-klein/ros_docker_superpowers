#!/usr/bin/env bash

docker run \
	--name rosdnsmasq \
	-d \
	-p 10053:53/udp \
	-v /tmp/ros_dnsmasq.d/dnsmasq.conf:/etc/dnsmasq.conf \
  ros_dnsmasq
