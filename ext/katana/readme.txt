Katana Native Interface
-----------------------
(c) 2004 Neuronics AG

Katana Native Interface is a C++ library for programmers who would like to
write their own programs, but don't want to implement the protocol and
device stuff katana is using.

Please also read INSTALL.txt before starting.

- doc:
------

	Contains the KNI, Kinematics and Linear Movements manuals (pdf format), as
	well as the function/class-documentation.

- lib:
------

	All the static and dinamic libraries are placed here (Win32/Linux).


- demo:
-------

	Contains a few sample demo programs, which show how the library
	should be used to develop applications - it's the best place to learn
	quickly how to use the libraries.

	- interlink:	    establishes a link to the katana
	- perfo:	    measures the connection speed between the katana
			    and the host computer; it takes a measure of
			    most of the commands the firmware is supporting
	- calib:	    demonstrates the auto-calibration option of the
			    library with the katana
	- sensor:	    demonstrates how to use the library to get sensor
			    values from the katana; this program can also be
			    used to detect defect sensors.
	- motors: 	    allows the user to move each of the motors using
			    the keyboard.
	- kinematics: 	    shows how to steer the robot using cartesian 
			    coordinates for the end-effector.
	- LinearMovements:  shows how to use steer the robot describing 
			    linear trajectories. 


Please have a look at the "KNI Manual" and the CLASS-Documentation in the 'doc'-folder.


Neuronics AG
Software Development
<softdev@neuronics.ch>
Last Update: 21.06.2005
