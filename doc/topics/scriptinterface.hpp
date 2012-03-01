// -*- latex -*-

/**

\page page_rw_scriptinterface Script interface
- \ref sec_rw_sinterface_intro
- \ref sec_robwork_python
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

Now all RobWork types available through the bindings should be available. For some purposes it might
be usefull to start an instance of RobWorkStudio:

\code
rwstudio = rws.getRobWorkStudio()
// now load a workcell
rwstudio.loadWorkCell('some/workcell.wc.xml')
// lets get the workcell
wc = rwstudio.getWorkCell()
print wc.getName()
\endcode

*/
