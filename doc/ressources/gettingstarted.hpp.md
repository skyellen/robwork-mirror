Getting Started  {#page_rw_gettingstarted}
================

[TOC]

## Getting started with Lua ##
TODO: describe how to use lua (embedded vs external) together with RobWork and present some 
usefull scripts

## From the command line ## 
TODO: describe how lua can be used with RobWork  from the commandline 

## RobWorkStudio embedded Lua ## 
TODO: describe how lua can be used with RobWork both from the luaeditor plugin.

# Getting started with Python # 
TODO: describe how to use python together with RobWork and present some usefull scripts 

# Compose a RobWork WorkCell #
TODO: add a simple tutorial on creating a robwork workcell in the XML format

# Command line tools #
TODO: add all command line tools, split them in categories eg. geometry, grasping, planning


# Writing RobWork programs #
This section will present project templates that can be used for different types of project 
purposes. All templates are based on CMake. 

## Plugins ##
TODO: add 

## GUI Plugins ##

The GUI plugin is a Qt plugin for RobWorkStudio and as such is very different from the 
more general plugin structure in RobWork. The GUI plugin must inherit the rws::RobWorkStudioPlugin
class and implement at least some of its functions. In this section we present two common
project templates for creating a GUI plugin. The first template is the simplest and Qt gui
functionality needs to be added in the hpp/cpp plugin file. The second template use the Qt 
designer to create an ui file which describe the layout.  

Common to both templates is how to load the plugin into RobWorkStudio when they have compiled. 
The compiled output will be a dynamically linkable file (win32 its .dll and linux its .so). 
To use the plugin in your robworkstudio installation add the following lines to the 
RobWorkStudio.ini file in the RobWorkStudio.exe directory:
 
	SamplePlugin\DockArea=2
	SamplePlugin\Filename=libSamplePlugin
	SamplePlugin\Path=PATH_TO_PLUGIN
	SamplePlugin\Visible=true  

### Not using Qt designer ###
This template can be found in the folder RobWorkStudio/example/pluginapp . The files include:

- CMakeLists.txt : the cmake project file
- SamplePlugin.cpp : the source file of the plugin
- SamplePlugin.hpp : the header file of the plugin
- pa_icon.png : a sample icon used in the toolbar of RobWorkStudio
- resources.qrc : a Qt resource file that enables adding images directly in exe/dll/so  

Lets start with CMakeLists.txt

\include pluginapp/CMakeLists.txt

\include pluginapp/SamplePlugin.hpp

\include pluginapp/SamplePlugin.cpp

### Using Qt designer ###

TODO: add the robworkstudio template projects

## Programs ##

TODO: add template c++ projects that link to robwork libraries




