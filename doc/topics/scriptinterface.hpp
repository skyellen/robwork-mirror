// -*- latex -*-

/**

\page page_rw_scriptinterface Script interface
- \ref sec_rw_sinterface_intro
- \ref sec_robwork_python
    - \ref sec_robwork_python_communicate_plugins
\section sec_rw_sinterface_intro Introduction


\section sec_robwork_python Python interface

Make sure you have both swig and python installed. You can check this when running
cmake. There should be lines like: "RobWork: python bindings ENABLED" and
"RobWorkStudio: python bindings ENABLED"

When compiling RobWork, RobWorkStudio and RobWorkSim the python interfaces will be
generated. These consist of two files per project and they are placed in the
libs/Release|Debug folder.

Linux example:
\verbatim
RobWork/libs/Debug/_rw.so \n
RobWork/libs/Debug/rw.py \n
RobWorkStudio/libs/Debug/_rws.so \n
RobWorkStudio/libs/Debug/rws.py \n
RobWorkSim/libs/Debug/_rwsim.so \n
RobWorkSim/libs/Debug/rwsim.py \n
\endverbatim

For the above example loading of the modules in the python interpreter could look like this:
\code
import sys
sys.path.append('/home/jimali/rw/RobWork/libs/Debug')
sys.path.append('/home/jimali/rw/RobWorkSim/libs/Debug')
sys.path.append('/home/jimali/rw/RobWorkStudio/libs/Debug')
import rw, rws, rwsim

// now we can use all robwork python bindings
\endcode

Now all RobWork types, that have bindings, should be available. For some purposes it might
be useful to start an instance of RobWorkStudio:

\code
rwstudio = rws.getRobWorkStudio()
// now load a workcell
rwstudio.loadWorkCell('some/workcell.wc.xml')
// lets get the workcell
wc = rwstudio.getWorkCell()
print wc.getName()
\endcode

\subsection sec_robwork_python_communicate_plugins
It is often necessary to send messages or data to one or more plugins. For this the
generic event methods on RobWorkStudio is used. These are wrapped in utils such that
the current send methods can be used in python
\code
rwstudio.send("someStr")
rwstudio.send("msgId", "some string")
rwstudio.send("msgId", 0.45235)
rwstudio.send("msgId", rw.Q(4, 0.1, 0.2, 0.3, 0.5) )
rwstudio.send("msgId", somePropertyMap )
\endcode

The first send method use RobWorkStudio::genericEvent the next 4 use RobWorkStudio::genericAnyEvent.

*/
