This device definition also lies in RobWork/data/trunk/devices/hands/Robotiq-2-finger-85. The two models should remain identical if possible.

There are two models of the Robotiq hand, that allows controlling it in different ways:
With the RobotiqFingerControl model, it is possible to control the coupled motor joint and the two passive links individually.
The RobotiqDistanceControl model is slightly more complex, but makes it possible to control the distance between the fingers.
This is somewhat similar to how the physical control works. Besides controlling the distance, it is also possible to control the angle of each finger.
The distance control is modelled with a serial chain between the two fingers, consisting of a revolute, prismatic and revolute joint.