
RWSim is a dynamic simulator package for for RobWork. It depends on the RobWork core library
and somewhat on RobWorkStudio for GUI and plugins. 

The project is setup into different folders:
src - files under src folder is compiled into libaries and plugins
test - The test folder contain testsrc that will link with the main library
example - The example folder contains example files that show how to use RobWorkSim 

To include extra libraries or includedirectories to all source add it to CMakeLists.txt
in the root folder. Else add it to the specific CMakeLists.txt folder under the folder 
src, test, example or plugin.

RobWorkSim is installed using cmake

example: 

Compilation, Win32 MinGW example
- Copy 'config.cmake.template' to 'config.cmake' and make your configuration 
  if default is not good enough 
- You should make sure to create the build tree from within a subfolder
-- create "release" and go to "release"
-- execute 'cmake -G "MinGW Makefiles" -D CMAKE_BUILD_TYPE=Release ../..'
-- execute cmakesetup to configure the CACHED variables if needed 
-- execute make all

Compilation, Linux GCC example
- Copy 'config.cmake.template' to 'config.cmake' and make your configuration  
  if default is not good enough 
- You should make sure to create the build tree from within a subfolder
-- create "release" and go to "release"
-- execute 'cmake -G "Unix Makefiles" -D CMAKE_BUILD_TYPE=Release ../..'
-- execute ccmake to configure the CACHED variables if needed 
-- execute make all

