Many of the packages within RobWorkHardware only compiles on specific platforms. When in need of a device 
go to the src folder and edit the CMakeLists.txt file to compile the devices our your choice.

To compile RobWorkHardware goto the build folder and type

mkdir Release

cd Release

cmake -DCMAKE_BUILD_TYPE=Release ../../

make


If you wish to compile in Debug simple replace Release with Debug in the lines above.