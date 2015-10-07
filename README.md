#pyaudio_controller_ros

Authors:

    Peter Polidoro <polidorop@janelia.hhmi.org>

License:

    BSD


##Running

Connect speakers.

```shell
roslaunch pyaudio_controller pyaudio_controller.launch
```

```shell
rostopic pub -1 /pyaudio_controller_node/play_tone pyaudio_controller/Tone -- 3000 500
```
