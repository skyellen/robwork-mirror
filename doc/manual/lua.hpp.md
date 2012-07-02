# Lua scripting language interface # {#page_lua}

[TOC]

# Introduction # {#sec_lua_intro} 

RobWork has a small (experimental/beta) interface to the 
<a href="http://www.lua.org/">Lua</a> scripting language. This section will present examples 
and general use patterns for using Lua in RobWork.

In general most functionality in the Lua interface is generated using Tolua++ and most apidoc 
from \code rwlibs::lua \endcode should be directly applicable in a lua script. 
E.g. the function from \code rwlibs::lua::gripFrame() \endcode 
can be called in a script using \verbatim rwlibs.lua.gripFrame(...) \endverbatim.

# Lua interface basics # {#sec_lua_basics}
As described above the Lua interface is generated with ToLua++ and wrapper classes for most robwork
classes has been created to seperate lua stuff from the c++ classes. This means that 
\code rwlibs::lua::Vector3D \endcode is
a wrapper to \code rw::math::Vector3D<> \endcode where most functions are wrapped but not all. This
design enables the lua interface to have a more simple script like nature. Whereas the C++ interface 
often has a more object oriented design and descriptive design. E.g. in C++ a function for calculating 
world to frame transform is names \code rw::kinematics::Kinematics::worldTframe \endcode whereas in lua
its \code rwlibs::lua::wTf \endcode.

All wrapper classes implement a basic set of functions:
- \c get - gets the robwork class that is wrapped.
- \c __tostring - enables output through the lua \c print function.

## Creating objects with constructors ## {#subsec_lua_basics_new}
Creating a new object using the Lua interface is straight forward. An Object is created using one
of its constructors. By default there are two ways to do this: \b global where you handle object destruction 
yourself and \b local where the tolua garbage collector destructs the object for you.
Global:
~~~~~~{.lua}
q = rwlibs.lua.Q:new(2,{1,2})
-- dostuff
q:delete() 
~~~~~~

and the local method

~~~~~~{.lua}
q = rwlibs.lua.Q(2,{1,2})
-- dostuff
~~~~~~

The lua interface is under expansion and this section will contain the preliminary interface
that is wanted from the different rw packages.

Heres a list of the packages and what content that is most important

- *math* - All math types (Vector3D, Transform3D, RPY ...) in this package is
important since they are basic building blocks in all of robwork. Besides these basic types
some of the more popular utillity functions need be available (SVD, conversion, ran, min, max, clamp, metrics)
- *common* - Most of the stuff in common is utility functions and are not that important
in script interface. Though some filefunctions, timerfunctions and log functions could come inhandy.
- *geometry* - factory methods and


# Building a Lua interpreter # {#sec_lua_interpreter}

rwlibs::lua implements a Lua module named \c rw that can be loaded
into a Lua interpreter with rwlibs::lua::RobWork::open(). This program
loads the \c rw module into a Lua interpreter and runs a Lua script
given on the command line:

\include ex-lua-interpreter.cpp

# Lua program example # {#sec_lua_script}

This example showcases most of the functionality of the Lua script
interface:

\include ex-simple.lua

The \c workcell.wu file loaded in the script is described in section
\ref sec_tul_workcell.

The output of running the script in the interpreter (see section \ref
sec_lua_interpreter) is as follows:

\include ex-simple-output.txt

# Executing Lua from commandline using rw-lua tool # {#sec_lua_robworkstudio}

# Executing Lua code in RobWorkStudio # {#sec_lua_robworkstudio}

Lua code can be executed in RobWorkStudio via the Lua plugin
interface. The Lua plugin has some additional Lua functions in the \c
rw package to access variables of the RobWorkStudio environment.

\c rw.getWorkCell() returns the currently loaded workcell:


	w = rw.getWorkCell(); print(w)
	--
	WorkCell[D:/movebots/FanucSchunk/scene.wu]

\c rw.getDevice(name) retrieves the device of the given name:

	d = rw.getDevice("Robot"); print(d)
	--
	Device[Robot]

\c rw.getState() returns a copy of the current workcell state of
RobWorkStudio:

	s = rw.getState(); print(d:getQ(s))
	--
	Q[6]{0, 0, 0, 0, 0, 0}

The workcell state can be written back to RobWorkStudio with \c
rw.setState():

	q = rw.Q{0.5, 0, 0, 0, 0, 0}; d:setQ(q, s); rw.setState(s)

This will update the position of the device as displayed in
RobWorkStudio.

# Other Lua functions and methods # {#sec_lua_misc}

The interface has some additional experimental functions and methods
that will be documented and expanded as RobWork matures.

	device = rw.CompositeDevice(devices, state, options)
	detector = rw.getCollisionDetector(options)
	collision = detector:inCollision(state)
	strategy = rw.getCollisionStrategy(options)
	planner = rw.getPathPlanner(device,
	     { tcp = frame, state = state, detector = detector })
	path = planner:query(from, to)
	path = rw.Path(states)
	path1 + path2
	rw.storeStatePath(workcell, path, file)
	rw.getOutput()

# Reference # {#sec_lua_reference}

## Classes ## {#sec_lua_classes}
Most classes accessible from lua is wrapper classes. The object which an
wrapper class wraps can allways be returned using the \b get() function.
