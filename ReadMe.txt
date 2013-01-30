
The RobWorkProject consist of several sub projects. These each have their own cmake 

build structure and can be build individually (beware of the dependencies of RobWork).

The cmake file in this root dir is a convinience file that enables the user to compile 
all selected projects in one go. 

Options to enable projects are:
WITH_RWS: enable RobWorkStudio
WITH_RWSIM: enable RobWorkSim
WITH_RWHW: enable RobWorkHardware

As default RobWork, RobWorkStudio and RobWorkSim are enabled.

example: 

Compilation, Linux GCC example

- Create a sub folder say "release": 
	mkdir release
- Goto that folder: 
	cd release
- call cmake in Release mode and disable RobWorkSim and enable RobWorkHardware: 
	cmake -DWITH_RWSIM=0 -DWITH_RWHW=1 -DCMAKE_BUILD_TYPE.. 
- call make using several threads (unly if you have more than 3G ram, else use 1):
	make -j3  
- if you want to install:
	sudo make install
