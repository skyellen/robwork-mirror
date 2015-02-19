RobWorkStudio/GUI plugins     {#pageRobWorkStudioPlugins} 
===========================

[TOC]

# Introduction #
These tutorials will walk through the creation and possibillities of a RobWorkStudio plugin. The topics will
include creation, basic RobWorkStudio event usage,

# Tutorial 1 – Your first RobWorkStudio plugin # {#sec_rws_first_plugin}
 This tutorial will go through the creation of a RobWorkStudio plugin using cmake. The plugin is very simple
 and for illustrative purposes includes two buttons with event handling.
 Compared to tutorial 1, we need a bit more work. Specifically we need to create a class that implements
 the RobWorkStudioPlugin class interface. Let us start by defining the CMakeLists.txt file
 
 ~~~
 CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)
 # The name of the project.
 PROJECT(SamplePluginApp)
 # Used to resolve absolute path names
 SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})
 # Now set the RW/RWS root (edit this if necessary)
 SET(RW_ROOT "${ROOT}/../../../RobWork")
 SET(RWSTUDIO_ROOT "${ROOT}/../../../RobWorkStudio")
 # We use the settings that robwork studio uses
 SET(CMAKE_BUILD_TYPE Release)
 #Include default settings for constructing a robwork dependent project
 INCLUDE(${RW_ROOT}/build/FindRobWork.cmake)
 #Include default settings for constructing a robworkstudio dependent project
 INCLUDE(${RWSTUDIO_ROOT}/build/FindRobWorkStudio.cmake)
 INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIR} ${ROBWORKSTUDIO_INCLUDE_DIR} )
 LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} ${ROBWORKSTUDIO_LIBRARY_DIRS} )
 #########################
 From here we add the plugins
 QT4_WRAP_CPP(MocSrcFiles SamplePlugin.hpp)
 QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)
 # The shared library to build:
 ADD_LIBRARY(SamplePlugin MODULE SamplePlugin.cpp ${MocSrcFiles} ${RccSrcFiles})
 TARGET_LINK_LIBRARIES(SamplePlugin ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})
 ~~~

 As can be seen we need to make three files.
 resources.qrc: Is a QT resource file. We will not use this right now but later it will be used for embedding
 images/icons in your plugins and exefiles.
~~~{.xml}
 <!DOCTYPE RCC><RCC version="1.0">
 <qresource>
 <!--file>pa_icon.png</file -->
 </qresource>
 </RCC>
~~~

 SamplePlugin.hpp: The plugin header file declares the basic functionalities that we inherit from the
 RobWorkStudioPlugin interface. Beside the inherited interface we add two QPushButton pointers.

 ~~~{.cpp}
 #ifndef SAMPLEPLUGIN_HPP
 #define SAMPLEPLUGIN_HPP
 #include <rw/rw.hpp>
 #include <rws/RobWorkStudioPlugin.hpp>
 class SamplePlugin: public rws::RobWorkStudioPlugin
 {
 Q_OBJECT
 Q_INTERFACES( rws::RobWorkStudioPlugin )
 public:
 SamplePlugin();
 virtual ~SamplePlugin();
 // functions inherited from RobworkStudioPlugin, are typically used but can be optional
 virtual void open(rw::models::WorkCell* workcell);
 virtual void close();
 virtual void initialize();
 private slots:
 void clickEvent();
 void stateChangedListener(const rw::kinematics::State& state);
 private:
 QPushButton* _btn0,*_btn1;
 };
 #endif // SAMPLEPLUGIN_HPP
 ~~~

 SamplePlugin.cpp: The plugin source file implements the basic functionality.

~~~{.cpp}
 #include "SamplePlugin.hpp"
 #include <QPushButton>
 #include <RobWorkStudio.hpp>
 USE_ROBWORK_NAMESPACE
 using namespace robwork;
 using namespace rws;
 SamplePlugin::SamplePlugin():
 RobWorkStudioPlugin("SamplePluginName", QIcon(":/pa_icon.png"))
 {
 QWidget* base = new QWidget(this);
 QGridLayout* pLayout = new QGridLayout(base);
 base->setLayout(pLayout);
 this->setWidget(base);
 int row = 0;
 _btn0 = new QPushButton("Button0");
 pLayout->addWidget(_btn0, row++, 0);
 connect(_btn0, SIGNAL(clicked()), this, SLOT(clickEvent()));
 _btn1 = new QPushButton("Button1");
 pLayout->addWidget(_btn1, row++, 0);
 connect(_btn1, SIGNAL(clicked()), this, SLOT(clickEvent()));
 pLayout->setRowStretch(row,1);
 }
 SamplePlugin::~SamplePlugin(){
 // deallocate used memory
 }
 void SamplePlugin::open(WorkCell* workcell){ //do something when workcell is openned
 }
 void SamplePlugin::close() { //do something when the workcell is closed
 }
 void SamplePlugin::initialize() {
 // do something when plugin is initialized
 getRobWorkStudio()->stateChangedEvent().add(
 boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);
 }
 void SamplePlugin::stateChangedListener(const State& state) {
 log().info() << "State changed!";
 }
 void SamplePlugin::clickEvent() {
 QObject *obj = sender();
 if(obj == _btn0){
 log().info() << "Button 0 pressed!\n";
 } else if(obj == _btn1){
 log().info() << "Button 1 pressed!\n";
 }
 }
 Q_EXPORT_PLUGIN(SamplePlugin);
~~~

## Excercise 1 - Updating the states ## {#rw_updating_states}
 Extend the plugin such that any device in scene will add 0.01 to all its joints when _btn0 is pushed and
 substract 0.01 each time _btn1 is pushed. Remember to update the RobWorkStudio state after changing it.
 In this exercise you should look at setState/getState on the RobWorkStudio interface and
 getRobWorkStudio() on RobWorkStudioPlugin interface.

## Exercise 2 – Saving the state ## {#rw_saving_states}
 Extend the plugin example with two new buttons. One that saves the current state in a member variable of
 the plugin, and a second button that

