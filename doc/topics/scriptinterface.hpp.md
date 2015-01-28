Script interface	{#page_rw_scriptinterface}
================

[TOC]

# Introduction # 	{#sec_rw_sinterface_intro}
The script interface is generated using [SWIG](http://www.swig.org/) which makes it possible to
interface using many different languages. See http://www.swig.org 
for a complete list of supported languages. 

By default Lua, Python and Java interfaces are generated for RobWork. In this document the use of 
these interfaces will be introduced.  SWIG can not yet generate interfaces for MATLAB, but fortunately
MATLAB provides good support for Java.

# Lua interface # 	{#sec_robwork_lua}

# Python interface # 	{#sec_robwork_python}

Make sure you have both swig and python installed. If you are using Ubuntu, make sure to install python-dev package as well.
You can check if Python interfaces for RobWork are generated when running cmake. There should be lines like: "RobWork: python bindings ENABLED" and
"RobWorkStudio: python bindings ENABLED".

When compiling RobWork, RobWorkStudio and RobWorkSim the python interfaces will be
generated. These consist of two files per project and they are placed in the
libs/Release|Debug folder.

Linux example:

	RobWork/libs/Debug/_rw.so 
	RobWork/libs/Debug/rw.py 
	RobWorkStudio/libs/Debug/_rws.so 
	RobWorkStudio/libs/Debug/rws.py 
	RobWorkSim/libs/Debug/_rwsim.so 
	RobWorkSim/libs/Debug/rwsim.py 

For_ the above example loading of the modules in the python interpreter could look like this:

~~~~{.py}
import sys
sys.path.append('/home/jimali/rw/RobWork/libs/Debug')
sys.path.append('/home/jimali/rw/RobWorkSim/libs/Debug')
sys.path.append('/home/jimali/rw/RobWorkStudio/libs/Debug')
import rw, rws, rwsim
// now we can use all robwork python bindings
~~~~

Now all RobWork types, that have bindings, should be available. For some purposes it might
be useful to start an instance of RobWorkStudio:

~~~~{.py}
rwstudio = rws.getRobWorkStudio()
// now load a workcell
rwstudio.openWorkCell('some/workcell.wc.xml')
// lets get the workcell
wc = rwstudio.getWorkCell()
print wc.getName()
~~~~

## Communicating with plugins ## 	{#sec_robwork_python_communicate_plugins}
It is often necessary to send messages or data to one or more plugins. For this the
generic event methods on RobWorkStudio is used. These are wrapped in utils such that
the current send methods can be used in python

~~~~{.py}
rwstudio.send("someStr")
rwstudio.send("msgId", "some string")
rwstudio.send("msgId", 0.45235)
rwstudio.send("msgId", rw.Q(4, 0.1, 0.2, 0.3, 0.5) )
rwstudio.send("msgId", somePropertyMap )
~~~~

The first send method use RobWorkStudio::genericEvent the next 4 use RobWorkStudio::genericAnyEvent.
Please take a look in rws/RobWorkStudio.hpp to get an example on using these events in your plugin.

## Using path planners in python ## {#sec_robwork_python_planning}
RobWork has several path planners which might be used from python. If we assume that a workcell with a
6 DOF robot named ``UR1'' has been loaded then a planner can be executed as follows:

~~~~{.py}
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
~~~~

We could also chose a query with a timeout or with some other rw::pathplanning::StopCriteria

~~~~{.py}
result = planner.query(q_from,q_to, 10.0) 
stopCriteria = rw.StopCriteria_stopCnt(100)
result = planner.query(q_from,q_to,stopCriteria) 
~~~~

# Java interface # 	{#sec_robwork_java}
The java interface is automatically enabled when running cmake if a Java SDK with Java Native Interface (JNI)
is found. For details on SWIG for Java, see the
<a href="http://www.swig.org/Doc2.0/SWIGDocumentation.html#Java">SWIG 2.0 Java documentation</a> which
also contains a good description of the more advanced issues when using JNI to call C++ code from Java.

Look for the line "RobWork: Java bindings ENABLED!" in the cmake output to be sure that Java interfaces are enabled.
The interface is generated for both RobWork, RobWorkStudio and RobWorkSim, and consists of two files per project.

Linux example (for a Debug build):

	RobWork/libs/debug/rw_jni.so 
	RobWork/libs/debug/rw_java.jar 
	RobWorkStudio/libs/debug/rws_jni.so 
	RobWorkStudio/libs/debug/rws_java.jar 
	RobWorkSim/libs/debug/rwsim_jni.so 
	RobWorkSim/libs/debug/rwsim_java.jar 

Furthermore javadoc is generated and can be launched from:

	RobWork/libs/debug/javadoc/index.html 
	RobWorkStudio/libs/debug/javadoc/index.html 
	RobWorkSim/libs/debug/javadoc/index.html 

Unfortunately SWIG is not yet able to copy comments to javadoc, meaning that the generated
javadoc has no descriptions for classes and methods. Instead it is 
recommended that equivalent classes and methods are found in the ordinary apidoc.
Note that names of classes and methods will often be different in the Java interface.

## Compiling a Java program ## 	{#sec_robwork_java_compiling}
To compile a basic program that can utilize the RobWork Java API, consider the following small example.

Main.java

~~~~{.java}
import dk.robwork.*;
public class Main {
	public static void main(String[] args){
		LoaderRW.load();
		LoaderRWS.load();
		LoaderRWSim.load();
	}
}
~~~~

To compile this piece of Java code the classpath must be set up such that the jar files can be found.
The compile command should be similar to:

	javac
		-classpath .:/home/user/RobWork/libs/debug/rw_java.jar:/home/user/RobWorkStudio/libs/debug/rws_java.jar:/home/user/RobWorkSim/libs/debug/rwsim_java.jar
		Main.java

In the Eclipse IDE it is enough to add the jar files to the build path for the Java project.
Similar can be done for other IDEs.

Note that all generated Java classes will be located in the dk.robwork Java package.
Before calling any other method on the interface, it is important that the loader functions has been called first
(always try to call these three lines as the first thing in your program).

Now to actually run the program use the same classpath as before, and set the java.library.path:

	java
		-classpath .:/home/user/RobWork/libs/debug/rw_java.jar:/home/user/RobWorkStudio/libs/debug/rws_java.jar:/home/user/RobWorkSim/libs/debug/rwsim_java.jar
		-Djava.library.path=/home/user/RobWork/libs/debug:/home/user/RobWorkStudio/libs/debug:/home/user/RobWorkSim/libs/debug
		Main

Note that you need to use the -Djava.library.path option to set the library path. This is the path where
Java will search for the JNI .so (Linux) or .dll (Windows) files. In the Eclipse IDE this would be set in
the Run Configuration under arguments to the Virtual Machine.

There is an alternative to set the -Djava.library.path. If this is not specified at runtime, it can be hard-coded.
This is done by explicitly defining the path where the .so or .dll files are located when running the load methods.

		LoaderRW.load("/home/user/RobWork/libs/debug");
		LoaderRWS.load("/home/user/RobWorkStudio/libs/debug");
		LoaderRWSim.load("/home/user/RobWorkSim/libs/debug");

## Examples ## 	{#sec_robwork_java_examples}
To see examples of how the RobWork interface is used in Java, please look in the examples folder
for the different projects. For example look in the folders: 

	RobWorkStudio/example/scripts/java
	RobWorkSim/example/scripts/java
<!--
## Naming Conventions ## 	{#sec_robwork_java_naming}
Java has no concept of operator overloading which is used extensively in the C++ API.
To solve the problem of operator overloading in Java, the following naming conventions are used:

| C++              | Java                |
| ---------------- | ------------------- |
| `operator-()`    | `negate()`          |
| `operator*()`    | `multiply()`        |
| `operator/()`    | `divide()`          |
| `operator==()`   | `equals()`          |

-->

## Memory, Pointers, Arrays & References ## 	{#sec_robwork_java_memory}
In C++ there is a distinction between pass and return by reference, pointer or value.
This is not the case for Java. The Java object proxy is technically always the equivalent of a C++ pointer.

The Java objects can own the corresponding C++ object in the native interface. If it owns the native object
is will call the C++ destructor once the Java object is Garbage Collected. The C++ destructor can also be
called explicitly with the delete() function. In this case the Java object will be invalid, and it is up
to the user not to call methods on a object where delete has been called. If a object is returned from a C++ function
by value, the equivalent method in Java will return a Java object that owns the underlying C++ object. Similar
goes for objects constructed in Java by use of the new keyword.

The Java object does not need to be the owner of the C++ object. If a object is returned as a
reference or pointer in C++, the equivalent method in Java will return the same Java type object as before, but
this time it will not be the owner of the underlying C++ object. This distinction between return by value and
return by reference/pointer is in many ways what one would expect from standard C++ behaviour.

For input arguments to methods called in Java, every Java object passed can be considered a pointer. If the C++ function takes a value the object will
be copied, and if the C++ function takes a reference or pointer it will be passed by reference while being owned
by Java.

Now consider the following small example of creating a smart pointer:
~~~~{.java}
PathTimedStatePtr path = new PathTimedStatePtr(new PathTimedState());
~~~~
When the new PathTimedState() constructor is called an owned object is constructed.
Note however that this is done anonymously and that there is no reference to the newly created object.
Clearly this object might be deleted by the Garbage Collector right after creation. As it is owned
it will also destruct the underlying C++ object.

For smart pointers (all objects in java ending with Ptr), the constructor will take ownership of the
object it is created from. In this example the PathTimedStatePtr changes the ownership of the anonymous
PathTimedState object to false. This way only the Java object will be Garbage Collected, but the C++
object will remain. The smart pointer will make sure that there is still a way to access the object.
Note that the PathTimedStatePtr object is always owned no matter what. This is important as the native
smart pointer object must follow the lifetime of the smart pointer in Java in order to maintain the correct
reference count.

Note that using the above code snippet is always fine as long as Ptr types are constructed. Care must in general
be taken when anonymous objects are created, or owned objects might go out of scope. Garbage Collector issues
might be difficult to debug as it is unknown when the Garbage Collector might run, and might cause weird issues in
the program.

A good advice is to always prefer the smart pointer objects. They will always keep the C++ objects alive while
there still exists references to it (either in native code or in Java). In general one does not need to call the
C++ destructors explicitly with the delete function. If delete is called, consider setting the object to null right
after to avoid calling unavailable methods (these errors might be hard to debug).

## Callbacks ## 	{#sec_robwork_java_callback}
Callbacks are typically required when doing simulations. Java does not support function pointers in the same sense
as C++ does, so instead the callback can either be implemented by implementing a callback interface, or by
creating a listener for callback events.

First consider the callback interface for the ThreadSimulator:

~~~~{.java}
package dk.robwork;

public interface ThreadSimulatorStepCallbackHandler {
	public void callback(ThreadSimulator simulator, State state);
}
~~~~

To register a callback on the simulator a implementation of the interface must be provided by the user.

The native ThreadSimulator expects a boost function object in the setStepCallBack function.
Looking at the ThreadSimulator Java API, there are two available setStepCallBack methods that takes on of the
following types as input:

   * ThreadSimulatorStepCallback\n
   Represents a boost function and is equivalent to the native StepCallback typedef in ThreadSimulator.
   It is not possible to create this type from Java, but if predefined callback implementations are provided
   in native code this will be the type to use.
   
   * ThreadSimulatorStepCallbackEnv\n
   Represents an extended boost function. To make callbacks to Java, information about the Java environment must be stored.
   The extended type allows this. All callbacks defined in Java is created from this type, and this type is not compatible with the ThreadSimulatorStepCallback type.\n

The following three lines of code sets the callback method on the ThreadSimulator (tsim): 

~~~~{.java}
ThreadSimulatorStepCallbackHandler cb = new Callback();
ThreadSimulatorStepCallbackEnv fct = new ThreadSimulatorStepCallbackEnv(cb);
tsim.setStepCallBack(fct);
~~~~
where Callback is the implementation of the ThreadSimulatorStepCallbackHandler interface.

Now consider the memory mangement aspect. First consider the ThreadSimulatorStepCallbackHandler object.
When creating the ThreadSimulatorStepCallbackEnv object, the native code keeps a reference to the
ThreadSimulatorStepCallbackHandle object (as it need to call this back asynchronously).
Hence the JVM will not do Garbage Collection on this object, and we are safe.

Next consider the ThreadSimulatorStepCallbackEnv object. Clearly this object is a candidate for Garbage Collection.
Note however that the native C++ function setStepCallBack specifies that arguments are passed by value. Therefore
the ThreadSimulatorStepCallbackEnv object is copied, and Garbage Collection can safely delete the object afterwards.
As the ThreadSimulatorStepCallbackEnv object is owned, the underlying C++ object is also removed. This if fine as it
has already been copied.

A second approach for creating callbacks, is by implementing event handlers. This is mainly implemented
for use in MATLAB applications, but might make sense in Java applications as well. Basically this method
wraps the first method mentioned.

Again the callback is implemented by implementing the following interface. The simulator and state is
stored inside a ThredSimulatorStepEvent.

~~~~{.java}
package dk.robwork;

public interface ThreadSimulatorStepEventListener extends EventListener {
	void stepEvent(ThreadSimulatorStepEvent event);
}
~~~~

Next the event listener can be added to the simulator with the following few lines of code:

~~~~{.java}
ThreadSimulatorStepEventDispatcher dispatcher = new ThreadSimulatorStepEventDispatcher();
ThreadSimulatorStepEventListener listener = new Listener();
dispatcher.addThreadSimulatorStepEventListener(listener);
ThreadSimulatorStepCallbackEnv fct = new ThreadSimulatorStepCallbackEnv(dispatcher);
tsim.setStepCallBack(fct);
~~~~

where Listener is the implementation of ThreadSimulatorStepEventListener.

# MATLAB interface # 	{#sec_robwork_matlab}
As MATLAB has good Java support, it is possible to interface RobWork from MATLAB via Java.
In general it is recommended that a basic Java program is first compiled and tested before
trying to use the Java interface from MATLAB. This is due to the fact that there might be conflicts
between the libraries bundled with MATLAB, and the dependencies of the native JNI libraries.

## Launching MATLAB ## 	{#sec_robwork_matlab_launch}
When MATLAB is to be used together with the RobWork JNI libraries, MATLAB should be launched with some
environment variables set in order to control the loading of dependent libraries.

The following table gives an overview of some of the version numbers for different MATLAB versions (Linux):

Version     | Name   | JVM      | GCC/G++ | Boost  | Best Ubuntu Match
----------- | ------ | -------- | ------- | ------ | -----------------
MATLAB 7.2  | R2006a | 1.5.0    | ?       | ?      | -
MATLAB 7.3  | R2006b | 1.5.0    | 3.4     | ?      | -
MATLAB 7.4  | R2007a | 1.6.0    | 4.1     | ?      | -
MATLAB 7.5  | R2007b | 1.6.0    | 4.1     | ?      | -
MATLAB 7.6  | R2008a | 1.6.0_04 | 4.1     | ?      | -
MATLAB 7.7  | R2008b | 1.6.0_04 | 4.1     | ?      | -
MATLAB 7.8  | R2009a | 1.6.0_12 | 4.2     | ?      | -
MATLAB 7.9  | R2009b | 1.6.0_12 | 4.2     | ?      | -
MATLAB 7.10 | R2010a | 1.6.0_12 | 4.2     | ?      | -
MATLAB 7.11 | R2010b | 1.6.0_17 | 4.3     | ?      | -
MATLAB 7.12 | R2011a | 1.6.0_17 | 4.3     | 1.40.0 | 10.04, 10.10
MATLAB 7.13 | R2011b | 1.6.0_17 | 4.3     | 1.44.0 | -
MATLAB 7.14 | R2012a | 1.6.0_17 | 4.4     | 1.44.0 | -
MATLAB 8    | R2012b | 1.6.0_17 | 4.4     | 1.44.0 | -
MATLAB 8.1  | R2013a | 1.6.0_17 | 4.42    | 1.49.0 | 12.10, 13.04
MATLAB 8.2  | R2013b | 1.7.0_11 | ?       | ?      | -

The following approach has been tested with MATLAB R2013a on a Ubuntu 13.04 system. It is expected that
the library resolution issues might be very different for other versions of both OS and MATLAB.
It is also uncertain how this will be handled in a Windows environment.

Looking in the table, the Boost libraries bundled with MATLAB R2013a are version 1.49. This is lucky
as this is also the default version of the libraries in Ubuntu 13.04. Note that Boost libraries of different
version numbers can not in general be expected to have good compatibility, hence mixing them should be avoided.
It might be necessary to compile against the MATLAB Boost libraries instead. This will require downloading the
correct headers for the required version of Boost and setting up new Boost paths before running cmake.

First of all the problem with library mismatches is clearly illustrated when using ldd to resolve the
dependencies of the rw_jni.so library.

Try to execute the following in a terminal:

	ldd /home/user/RobWork/libs/debug/librw_jni.so

Now try to run in MATLAB:

	!ldd /home/user/RobWork/libs/debug/librw_jni.so

Notice the difference between how the library dependencies are resolved. MATLAB comes with its own version of the
system libraries, and these will often be older than the system dependencies that the JNI library was linked
against.

It is adviced that a bash script similar to the following is used to launch MATLAB to force it to use the
system environment.

~~~~{.sh}
#!/bin/bash
export RW_ROOT=/home/user/RobWork
export RW_BUILD=debug

export RW_LIBS=$RW_ROOT/RobWork/libs/$RW_BUILD
export RWStudio_LIBS=$RW_ROOT/RobWorkStudio/libs/$RW_BUILD
export RWSim_LIBS=$RW_ROOT/RobWorkSim/libs/$RW_BUILD

export WORK_DIR=$RW_ROOT/RobWorkSim/example/scripts/matlab
cd $WORK_DIR
echo "$RW_LIBS/rw_java.jar" > javaclasspath.txt
echo "$RWStudio_LIBS/rws_java.jar" >> javaclasspath.txt
echo "$RWSim_LIBS/rwsim_java.jar" >> javaclasspath.txt

export MATLAB_JAVA=$JAVA_HOME/jre
export LD_PRELOAD="$LD_PRELOAD /usr/lib/x86_64-linux-gnu/libstdc++.so.6"

/opt/MATLAB/bin/matlab
~~~~

The RW* variables are set for convenience to allow writing MATLAB scripts that are system independent.

Set the WORK_DIR varible to you project directory. A javaclasspath.txt file will by created in this directory
to set the static Java classpath in MATLAB. MATLAB also allows setting this dynamically from within MATLAB,
but please avoid this (especially if callbacks from Java to MATLAB are required).

The MATLAB_JAVA variable should be set if the .jar files has been compiled to a newer version of Java
than the JVM used by MATLAB. This will make sure that MATLAB uses the current system JVM.
This line might be uncommented if the versions already match.

The LD_PRELOAD variable forces MATLAB to use newer system libraries for libstdc++.so.6 instead of the
libraries that comes with MATLAB. This should be safe as the libstdc++ library is designed to be backwards
compatible.

Note that overriding the libraries that MATLAB use and changing the JVM is a drastic change that might
give other issues in MATLAB. Depending on the system it might not always be a requirement to set these variables.
Always try setting as few variables first and then add MATLAB_JAVA and LD_PRELOAD if required.
It might also be necessary to add even more libraries than shown here.

The following MATLAB code should run without errors before the RobWork API can be used from MATLAB.

~~~~{.m}
RW_LIBS=getenv('RW_LIBS');
RWSim_LIBS=getenv('RWSim_LIBS');
RWStudio_LIBS=getenv('RWStudio_LIBS');

% Import the java API
javaaddpath(strcat(RW_LIBS,'/rw_java.jar'));
javaaddpath(strcat(RWSim_LIBS,'/rwsim_java.jar'));
javaaddpath(strcat(RWStudio_LIBS,'/rws_java.jar'));
import dk.robwork.*;

% Load the native libraries
LoaderRW.load(RW_LIBS)
LoaderRWSIM.load(RWSim_LIBS)
LoaderRWS.load(RWStudio_LIBS)
~~~~

## Typical Errors ## 	{#sec_robwork_matlab_errors}
It can be difficult to get the MATLAB interface to run. The following is a list of known errors and
possible solutions.

	>> LoaderRW.load(RW_LIBS)
	Java exception occurred:
	java.lang.UnsatisfiedLinkError: /home/user/RobWork/RobWork/libs/debug/librw_jni.so:
	/opt/MATLAB/bin/glnxa64/../../sys/os/glnxa64/libstdc++.so.6: version `GLIBCXX_3.4.15' not found (required by /home/user/RobWork/RobWork/libs/debug/librw_jni.so)

This error is caused by librw_jni.so as it is dependent on a newer version of the standard C++ library than
the library provided and used by MATLAB. To solve this issue set the LD_PRELOAD:

	export LD_PRELOAD="$LD_PRELOAD /usr/lib/x86_64-linux-gnu/libstdc++.so.6"

If classes can not be found:

	>> javaaddpath(strcat(RW_LIBS,'/rw_java.jar'));
	>> import dk.robwork.*;
	>> LoaderRW.load(RW_LIBS)
	Undefined variable "LoaderRW" or class "LoaderRW.load".

Make sure that the .jar file is at the given path, and that the .jar actually contains a LoaderRW class.
If this is the case, the reason for MATLAB not finding the class can be that there is a mismatch between
the .jar Java version and the MATLAB JVM.

There can be two solutions. First one is to use another JVM version in MATLAB.

	export MATLAB_JAVA=$JAVA_HOME/jre

Secondly the source can be manually compiled using a different Java compiler version. The generated source
is located in the build folder under src/rwlibs/swig/java_src and similar for the other packages. See the Java
version used by MATLAB with

	version -java

## Callbacks ## 	{#sec_robwork_matlab_callback}
When running simulations, callbacks might be required from C++ code to MATLAB. In MATLAB the implementation of
such callbacks can be a bit tricky. Callbacks are implemented as a listener to the corresponding Java event. 

First consider a MATALB function handling the callback event:

~~~~{.m}
function StepCallBack( dispatcherObject, event, rwstudio )
    tsim=event.getThreadSimulator();
    state=event.getState();
    
    userdata=getappdata(dispatcherObject,'UserData');
    counter = userdata(1);
    
    % Set state in RobWorkStudio and print time for each 20 steps
    if mod(counter,20) == 0
        rwstudio.setState(state);
        display(num2str(tsim.getTime()));
    end

    setappdata(dispatcherObject,'UserData',[counter+1]);
end
~~~~

The function takes at least two arguments, namely the dispatcher object itself and the event.
In the example it is also illustrated how to append additional arguments in MATLAB (here used for
passing a pointer to the RobWorkStudio instance).

First two lines of the functions extracts the data stored in the event, which is a pointer to the
ThreadSimulator and the state. To illustrate a second way of storing additional data, a counter is
stored on the dispatcherObject under UserData (managed internally by MATLAB). On the last line the counter is
incremented. For each 20 callbacks, the state is updated in RobWorkStudio and the current simulated time
is displayed.

Now a callback can be added by using the following few lines of code (very similar to the Java example):

~~~~{.m}
dispatcher = ThreadSimulatorStepEventDispatcher();
setappdata(dispatcher,'UserData',[0]); % counter
set(dispatcher,'StepEventCallback',{@StepCallBack,rwstudio});
fct = ThreadSimulatorStepCallbackEnv(dispatcher);
tsim.setStepCallBack(fct);
~~~~

The third line set the callback with a MATLAB function reference to the StepCallBack function. Notice
how the rwstudio argument is added. It is possible to add an arbitrary number of static arguments this way.

## Examples ## 	{#sec_robwork_matlab_examples}
To see examples of how the RobWork Java interface can be used in MATLAB, please look in the examples folder
for the different projects: 

	RobWorkStudio/example/scripts/matlab
	RobWorkSim/example/scripts/matlab
