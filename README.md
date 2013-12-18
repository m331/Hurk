Watch out for the Hurk!
======

To compile Hurk, run the following commands:
```
rosmake base
```
```
rosmake hurk
```

To run the base and the PS3 controller:
```
sudo bash
roslaunch hurk hurk.launch

```

Then run in another terminal:
```
rosrun base ps3_conv
```



To launch the Kinect & laserscan, run the following:
```
roslaunch openni_launch openni.launch
rosrun rqt_reconfigure rqt_reconfigure
```
And select /camera/driver from the drop-down menu. Enable the depth_registration checkbox. 

Then you can run:
```
rosrun depthimage_to_laserscan depthimage_to_laserscan image:=/camera/depth_registered/image_raw
```

To run the sensors and the arduino:
TO DO
