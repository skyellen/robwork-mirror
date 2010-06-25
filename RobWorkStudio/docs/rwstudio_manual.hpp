// -*- latex -*-

/**

\page page_rwstudio_manual RobWorkStudio manual

- \ref sec_rws_manual_intro
- \ref sec_rws_plugins
	- \ref sec_rws_plugins_jog
	- \ref sec_rws_plugins_log The Log plugin
	- \ref sec_rws_plugins_treeview The TreeView plugin
	- \ref sec_rws_plugins_lua The Lua plugin
	- \ref sec_rws_plugins_planning The planning plugin
	- \ref sec_rws_plugins_propertyview The propertyview plugin
	- \ref sec_rws_plugins_playback The playback plugin
	- \ref sec_rws_plugins_sensor The sensors plugin
- \ref sec_rws_user_plugins
- \ref sec_rws_user_tips
- \ref sec_rws_properties
- \ref sec_rws_examples
	- \ref subsec_rws_examples_adding_collision
	- \ref subsec_rws_examples_getting_drawables_of_a_frame

\section sec_rws_manual_intro Introduction

\section sec_rws_plugins Default RobWorkStudio plugins

\subsection sec_rws_plugins_jog The Jog plugin
Provides functionality for jogging around the robots in a workcell. 

\subsection sec_rws_plugins_log The Log plugin
Displayes the default log in RobWorkStudio

\subsection sec_rws_plugins_treeview The TreeView plugin
Shows the frame structure of the workcell.

\subsection sec_rws_plugins_lua The Lua plugin
Provides a simple editor for writing and executing lua scripts.

\subsection sec_rws_plugins_planning The planning plugin
Enables the user call motion planners and plan paths.

\subsection sec_rws_plugins_propertyview The propertyview plugin
The propertyview can be used to display and edit properties associated to frames in the workcell.

\subsection sec_rws_plugins_playback The playback plugin
This plugin enables recording and playback of TimedStatePaths.

\subsection sec_rws_plugins_sensor The Sensors plugin
This plugin can display output from simulated camera and range scanners in the workcell.



\section sec_rws_user_plugins Creating your own plugin
To create your own plugin copy one of the example plugins which can be found within the example
directory under RobWorkStudio. The pluginUIapp provides an example in which QT designer (GUI editor)
is used to design the user interface. The pluginapp provides a simple example without the dependency 
of a GUI building tool.

To compile the plugin you need to perform the following steps

- Edit the CMakeLists.txt file to ensure that the variables \verb"RW_ROOT" and \verb"RWSTUDIO_ROOT" points to you RobWork and RobWorkStudio directories.
- call \verb"cmake ." to generate build files
- call \verb"make" to build the plugin.

Once the plugin is build you need to tell RobWorkStudio to load it. This is done by editing the RobWorkStudio.ini file. If the RobWorkStudio.ini file does not exist you can copy the RobWorkStudio.ini.template from the bin directory. Within the template file you may have to remove the existing plugins and add the following

\verbatim
MyPlugin\DockArea=1
MyPlugin\Filename=libmyplugin
MyPlugin\Path=../../MyPlugin/
MyPlugin\Visible=false 
\endverbatim

Be sure that the MyPlugin\Path points to where your library has been generated and that MyPlugin\Filename is correct. You should not add any file extension (this is resolved automatically).

When you start RobWorkStudio it will load your plugin.


\section sec_rws_user_tips Tips 

Here are some small usefull examples that can be used from a plugin

Get the collision detector that is currently used in your robwork studio
instance
\code
CollisionDetector *detector = getRobWorkStudio()->getCollisionDetector();
\endcode


\subsection sec_rws_plugin_communication Communicating between plugins
RobWorkStudio has a number of event which can be used by the plugins. A plugin can register for an event, for example by

\code
getRobWorkStudio()->stateChangedEvent().add(boost::bind(&MyPlugin::stateChangedListener, this,_1), this);
\endcode
which binds the stateChangedListener method of MyPlugin to listen for state changed event.

To see more information about the different event please consult the RobWorkStudio api-doc.


\section sec_rws_properties RobWorkStudio specific frame properties

Through generic properties in the XML and TUL workcell file format, RobWork allows
for adding user specific information to frames. In this section RobWorkStudio specific
properties will be listed. Meaning properties that only makes sence for RobWorkStudio and
not RobWork.

\subsection sec_rwstudio_camera_property Camera property
A property describing a camera pinhole model can be added to a frame. The camera view can
then be visualized in RobWorkStudio. The property string looks like this:
\verbatim
"<Field of view Y> <width> <height>"
\endverbatim
example:
\verbatim
<Property name="Camera">60 640 480</Property>
\endverbatim

You can currently only change views between cameras using the key [1-9], were 1 is the default
3rd person view.

\b Important!
- Multiple cameras are supported but only one camera property per frame!
- The width and height has no real dimension its the proportion between them that matters
- The camera looks in the negative Z-axis direction of the frame
- Field of view is in degree and is defined in the Y-axis

\section sec_rws_examples Usefull examples

\subsection subsec_rws_examples_adding_new_frames Adding new frames to the workcell from a plugin
 This example describe how one can add his own frames to the workcell through
 a user plugin.

 Adding frames to the workcell is possible through the StateStructure instance that
 is located in a WorkCell. It is important to understand that adding frames to the
 state structure will change the static state structure of the workcell (the dynamic state is that
 which is located in the State object). Changing the static structure will not directly influence
 State objects, that is they are still valid for all frames except the newly added frames.
 There exist two methods of making an old state valid for new frames. One is to just assign
 the old state with a new one. Though, this will also overwrite any state information that was
 saved in the old state, say the configuration of tour robot. If you want to preserve the information
 in the old state and still make it valid for newly added frames you would need to upgrade it. You
 can upgrade a state \b oldstate using either StateStructure instance \b stateStruct or another
 state \b newstate. Following is an example of how:
\code
    // using another state to upgrade
    oldstate.upgradeTo(newstate); // oldstate is upgraded to the structure of the newstate
    // using state structure to upgrade
    oldstate = stateStruct.upgrade( oldstate );
\endcode

Following is an example of how to add a new frame to the workcell from your own plugin
\code
    State oldState; // this is your old state
    Frame *newFrame = make_new_frame(); // create your frame
    getRobWorkStudio()->getWorkCell()->getStructure()->addFrame(newFrame,parentFrame);
    // now update the oldState with the new state
    oldState = getRobWorkStudio()->getWorkCell()->getStructure()->upgradeState(oldState);
    // now this is VERY important, remember to update the RobWorkStudio state
    getRobWorkStudio()->setState(oldState);
\endcode


\subsection subsec_rws_examples_adding_drawable Adding drawables from a plugin
This example describe how one can add his own drawable to the robwork scene graph, from
his own robworkstudio plugin.
First we need to create the drawable, next we need to find the frame we want to
connect it too, and lastly add it to the WorkCellGLDrawer of RWStudio. The following
code snippet show the creation of a user specified render which is used to construct
a drawable. One could also use the DrawableFactory to create a drawable from either
file or a primitive string (Cube,Box,Cylinder etc.). Next a frame called "myFrame" is
searched for in the workcell. If the frame is found then a mapping from myFrame to
the user drawable is created in the WorkCellGLDrawer (SceneGraph).

\code
    MyRender *renderObj = new MyRender( .. );
    Drawable *drawableObj = new Drawable(boost::shared_ptr<Render>(renderObj));
    Frame *myFrame = getRobWorkStudio()->getWorkCell()->findFrame("myFrame");
    if(drawableFrame != NULL)
        getRobWorkStudio()->getWorkCellGLDrawer()->addDrawableToFrame(myFrame, drawableObj);
\endcode

\subsection subsec_rws_examples_adding_collision Adding collision models from a plugin

\code
    double scale = 1.0; // set a scale, actually not used in RobWork yet
    Transform3D<> transform = makeMyTransform();
    CollisionModelInfo info("myname", transform, scale);

    Accessor::collisionModelInfo().get(*myFrame).push_back(info);
\endcode

\subsection subsec_rws_examples_getting_drawables_of_a_frame Getting drawables from a frame

This code snippet will copy all drawables associated with the frame \b frameWithDrawables
into the vector \b drawables.
\code
	std::vector<Drawable*> drawables;
	Frame *frameWithDrawables; // specify the frame where your drawables are placed
	getWorkCellGLDrawer()->getAllDrawables(state, frameWithDrawables, drawables);
\endcode

The next code snippet will copy all drawables associated to any frame in the workcell
into the vector \b drawables.
\code
	std::vector<Drawable*> drawables;
	getWorkCellGLDrawer()->getAllDrawables(state, getWorkCell(), drawables);
\endcode


*/
