tone_controller_ros
===================

Authors:

    Peter Polidoro <polidorop@janelia.hhmi.org>

License:

    BSD


##Running

Connect speakers.

```shell
roslaunch tone_controller tone_controller.launch
```

```shell
rostopic pub -1 /tone_controller_node/cmd_tone tone_controller/CmdTone -- 3000 500
```
