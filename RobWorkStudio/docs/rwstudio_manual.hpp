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
- \ref sec_rws_user_plugins
- \ref sec_rws_properties
- \ref sec_rws_examples
	- \ref subsec_rws_examples_adding_collision
	- \ref subsec_rws_examples_getting_drawables_of_a_frame

\section sec_rws_manual_intro Introduction

\section sec_rws_plugins Default RWS plugins

\subsection sec_rws_plugins_jog The JOG plugin
\subsection sec_rws_plugins_log The Log plugin
\subsection sec_rws_plugins_treeview The TreeView plugin
\subsection sec_rws_plugins_lua The Lua plugin
\subsection sec_rws_plugins_planning The planning plugin
\subsection sec_rws_plugins_propertyview The propertyview plugin
\subsection sec_rws_plugins_playback The playback plugin
This plugin enables recording and playback of TimedStatePaths.

\section sec_rws_user_plugins Creating your own plugin
Here are some small usefull examples that can be used from a plugin

Get the collision detector that is currently used in your robwork studio
instance
\code
CollisionDetector *detector = getRobWorkStudio()->getCollisionDetector();
\endcode



\subsection sec_rws_plugin_trix

\subsection sec_rws_plugin_communication Communicating between plugins
 There are a few ways of communicating between several plugins both user and default
RobWorkStudio plugins.



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
