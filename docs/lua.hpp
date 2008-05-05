// -*- latex -*-

/**

\page page_lua Lua scripting language interface

- \ref sec_lua_intro
- \ref sec_lua_interpreter
- \ref sec_lua_script
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

When running the script in the interpreter the output is as follows:

\include ex-simple-output.txt

The interface has some additional functions and methods that are still
somewhat experimental:

\verbatim
-- rw.CompositeDevice(devices, state, options)
-- path1 + path2
-- rw.storeStatePath(workcell, path, file)
-- rw.getPathPlanner(workcell, device, tcp, state)
-- planner:query(from, to)
-- rw.Path(states)
-- rw.getWorkCell()
-- rw.setState(state)
-- rw.getState()
-- rw.print()
-- rw.getOutput()
\endverbatim

*/
