// -*- latex -*-

/**

\page page_lua Lua scripting language interface

- \ref sec_lua_intro
- \ref sec_lua_interpreter
- \ref sec_lua_script
- \ref sec_lua_robworkstudio
- \ref sec_lua_misc
.

\section sec_lua_intro Introduction

RobWork has a small (experimental) interface to the 
<a href="http://www.lua.org/">Lua</a> scripting language.

\section sec_lua_interpreter Building a Lua interpreter

rwlibs::lua implements a Lua module named \c rw that can be loaded
into a Lua interpreter with rwlibs::lua::RobWork::open(). This program
loads the \c rw module into a Lua interpreter and runs a Lua script
given on the command line:

\include ex-lua-interpreter.cpp

\section sec_lua_script Lua program example

This example showcases most of the functionality of the Lua script
interface:

\include ex-simple.lua

The \c workcell.wu file loaded in the script is described in section
\ref sec_tul_workcell.

The output of running the script in the interpreter (see section \ref
sec_lua_interpreter) is as follows:

\include ex-simple-output.txt

\section sec_lua_robworkstudio Executing Lua code in RobWorkStudio

Lua code can be executed in RobWorkStudio via the Lua plugin
interface. The Lua plugin has some additional Lua functions in the \c
rw package to access variables of the RobWorkStudio environment.

\c rw.getWorkCell() returns the currently loaded workcell:

\verbatim
w = rw.getWorkCell(); print(w)
--
WorkCell[D:/movebots/FanucSchunk/scene.wu]
\endverbatim

\c rw.getDevice(name) retrieves the device of the given name:

\verbatim
d = rw.getDevice("Robot"); print(d)
--
Device[Robot]
\endverbatim

\c rw.getState() returns a copy of the current workcell state of
RobWorkStudio:

\verbatim
s = rw.getState(); print(d:getQ(s))
--
Q[6]{0, 0, 0, 0, 0, 0}
\endverbatim

The workcell state can be written back to RobWorkStudio with \c
rw.setState():

\verbatim
q = rw.Q{0.5, 0, 0, 0, 0, 0}; d:setQ(q, s); rw.setState(s)
\endverbatim

This will update the position of the device as displayed in
RobWorkStudio.

\section sec_lua_misc Other Lua functions and methods

The interface has some additional experimental functions and methods
that will be documented and expanded as RobWork matures.

\verbatim
-- device = rw.CompositeDevice(devices, state, options)
-- planner = rw.getPathPlanner(workcell, device, state, { tcp = frame })
-- path = planner:query(from, to)
-- path = rw.Path(states)
-- path1 + path2
-- rw.storeStatePath(workcell, path, file)
-- rw.getOutput()
\endverbatim

*/
