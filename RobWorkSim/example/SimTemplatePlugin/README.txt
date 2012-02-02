
 This example illustrates how to use cmake to create a plugin application
 for RobWorkStudio.
 
 Copy the pluginapp dir and rename it to your project name. Edit the 
 cmakelist and rename the project. Add any source files to the cmake file
 and your about good to go. Make sure to read and follow the comments in the 
 cmake file.
 
 The output will be a dynamically linkable file (win32 its .dll and linux its .so). 
 To use the plugin in your robworkstudio installation add the following line to the 
 RobWorkStudio.ini file in the RobWorkStudio.exe directory:
 
SimTemplatePlugin\DockArea=2
SimTemplatePlugin\Filename=libSimTemplatePlugin
SimTemplatePlugin\Path=../../../RobWorkSim/example/SimTemplatePlugin/libs/Release/
SimTemplatePlugin\Visible=true
 
how to build the example:

Building win32 with mingw
  cmake -G "MinGW Makefiles" . 
  mingw32-make all
  
  
