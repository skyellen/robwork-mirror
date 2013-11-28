%% Initialize
clear;
clc;

% Please see the following page for a description of how MatLab should be launched:
% http://www.robwork.dk/apidoc/nightly/rw/page_rw_scriptinterface.html

% Setup the paths (from the environment variables)
RW_LIBS=getenv('RW_LIBS');
RWSim_LIBS=getenv('RWSim_LIBS');
RWStudio_LIBS=getenv('RWStudio_LIBS');

% Import the java API
import dk.robwork.*;

% Load the native libraries
LoaderRW.load(RW_LIBS)
LoaderRWSim.load(RWSim_LIBS)
LoaderRWS.load(RWStudio_LIBS)

% Try to open RobWorkStudio and load the workcell
rwstudio = rws.getRobWorkStudioInstance();
dwc = DynamicWorkCellLoader.load('DynamicWorkCell.dwc.xml');
wc = dwc.getWorkcell();
state = wc.getDefaultState();
rwstudio.postWorkCell(wc);

% Create simulator
cd = ContactDetectorPtr(ContactDetector(wc));
cd.setDefaultStrategies();
simulator = PhysicsEnginePtr(ODESimulator(dwc, cd));
simulator.initPhysics(state);
dsim = DynamicSimulatorPtr(DynamicSimulator(dwc, simulator));
tsim = ThreadSimulatorPtr(ThreadSimulator(dsim, state));
tsim.setTimeStep(0.001);
tsim.setRealTimeScale(1);

% Set target configuration for robot
controller = dwc.findPDController('UR1Controller');
qTarget = Q(6,-1.666,-2.332,-0.413,-1.283,0.793,-0.219);
controller.setTargetPos(qTarget);

%% Create callback and run Simulation
dispatcher = ThreadSimulatorStepEventDispatcher();
setappdata(dispatcher,'UserData',[0]); % set step counter
% Set the callback (rwstudio pointer is passed as additional argument)
set(dispatcher,'StepEventCallback',{@StepCallBack,rwstudio});
% Manually invoke callback (just a test)
dispatcher.callback(tsim.dereference(),state);

% Add to simulator and start
fct = ThreadSimulatorStepCallbackEnv(dispatcher);
tsim.setStepCallBack(fct);
tsim.start();

%% Exit RobWorkStudio
if (tsim.isRunning())
    tsim.stop();
end
rwstudio.postExit();

