#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
from __future__ import print_function,division

import pyaudio

import rospy

from tone_controller.msg import CmdTone


class ToneController(object):
    def __init__(self,*args,**kwargs):
        rospy.loginfo('Initializing tone_controller_node...')
        self._initialized = False

        self._cmd_current_sub = rospy.Subscriber('~cmd_current',CmdCurrent,self._cmd_current_callback)
        self._cmd_off_sub = rospy.Subscriber('~cmd_off',CmdChannel,self._cmd_off_callback)

        current_max = rospy.get_param('~current_max')
        self._dev = ToneDevice()
        self._channel_count = self._dev.get_channel_count()
        for channel in range(self._channel_count):
            channel += 1
            self._dev.set_normal_parameters(channel,current_max,1)
        rospy.loginfo('tone_controller_node initialized!')
        self._initialized = True

    def _cmd_current_callback(self,data):
        if self._initialized:
            channel = data.channel
            current = data.current
            if (channel >= 1) and (channel <= self._channel_count):
                if current > 0:
                    self._dev.set_normal_current(channel,current)
                    self._dev.set_mode_normal(channel)
                else:
                    self._dev.set_mode_disable(channel)

    def _cmd_off_callback(self,data):
        if self._initialized:
            channel = data.channel
            if (channel >= 1) and (channel <= self._channel_count):
                self._dev.set_mode_disable(channel)


if __name__ == '__main__':
    try:
        rospy.init_node('tone_controller_node')
        tc = ToneController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
