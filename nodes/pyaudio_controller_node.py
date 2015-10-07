#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
from __future__ import print_function,division

import math
import pyaudio

import rospy

from pyaudio_controller.msg import Tone


class PyaudioController(object):
    def __init__(self,*args,**kwargs):
        rospy.loginfo('Initializing pyaudio_controller_node...')
        self._initialized = False

        self._cmd_pyaudio_sub = rospy.Subscriber('~play_tone',Tone,self._play_tone_callback)

        rospy.on_shutdown(self._shutdown)
        self._pyaudio = pyaudio.PyAudio()
        self._bitrate = 16000 #number of frames per second/frameset.
        self._stream =  self._pyaudio.open(format=self._pyaudio.get_format_from_width(1),
                                           channels=1,
                                           rate=self._bitrate,
                                           output=True)
        rospy.loginfo('pyaudio_controller_node initialized!')
        self._initialized = True

    def _shutdown():
        self._stream.stop_stream()
        self._stream.close()
        self._pyaudio.terminate()

    def _play_tone_callback(self,data):
        if self._initialized:
            frequency = data.frequency
            duration = data.duration
            length = duration/1000
            frame_count = int(self._bitrate*length)
            restframes = frame_count % self._bitrate
            wavedata = ''

            for x in xrange(frame_count):
                wavedata = wavedata+chr(int(math.sin(x/((self._bitrate/frequency)/math.pi))*127+128))

            #fill remainder of frameset with silence
            for x in xrange(restframes):
                wavedata = wavedata+chr(128)

            self._stream.write(wavedata)


if __name__ == '__main__':
    try:
        rospy.init_node('pyaudio_controller_node')
        tc = PyaudioController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
