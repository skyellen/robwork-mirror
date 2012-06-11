
 This template illustrates how to using cmake to include RobWork into your library 
project. That is a project that use RobWork when generating a library.

 Copy the libraryapp dir and rename it to your project name. Edit the 
 cmakelist and rename the project. Add any source files to the cmake file
 and your about good to go. Make sure to read and follow the comments in the 
 cmake file.
 
 
examples:

Building win32 with mingw
  cmake -G "MinGW Makefiles" . 
  mingw32-make all
  
  
