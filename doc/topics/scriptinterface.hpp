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

\subsection sec_robwork_python_communicate_plugins Communicating with plugins
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
Please take a look in rws/RobWorkStudio.hpp to get an example on using these events in your plugin.

\subsection sec_robwork_python_planning Using path planners in python
RobWork has several path planners which might be used from python. If we assume that a workcell with a
6 DOF robot named ``UR1'' has been loaded then a planner can be executed as follows:
\code
// we need the workcell to get a handle to the robot
wc = rwstudio.getWorkCell()
dev = wc.findDevice("UR1")
state = rwstudio.getState()
cd = rwstudio.getCollisionDetector()
planner = rw.QToQPlanner_makeRRT(cd,dev,state)

// now the planner is ready to be used. We define the configurations
// in which the robot should start and end
q_from = rw.Q(6,0,-1,0,0,0,0)
q_to = rw.Q(6, 3,0.2,1,-1,0,0)
result = planner.query(q_from,q_to) 
\endcode

We could also chose a query with a timeout or with some other rw::pathplanning::StopCriteria
\code
result = planner.query(q_from,q_to, 10.0) 
stopCriteria = rw.StopCriteria_stopCnt(100)
result = planner.query(q_from,q_to,stopCriteria) 
\endcode



*/
