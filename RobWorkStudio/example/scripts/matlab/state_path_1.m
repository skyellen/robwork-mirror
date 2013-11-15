%% Initialize
clear;
clc;

% Setup the paths (from the environment variables)
% Please se the following page for a description of how MatLab should be
% launched with these paths set:
% http://www.robwork.dk/apidoc/nightly/rw/page_rw_scriptinterface.html
RW_LIBS=getenv('RW_LIBS');
RWStudio_LIBS=getenv('RWStudio_LIBS');

% Import the java API
javaaddpath(strcat(RW_LIBS,'/rw_java.jar'));
javaaddpath(strcat(RWStudio_LIBS,'/rws_java.jar'));
import dk.robwork.*;

% Load the native libraries
LoaderRW.load(RW_LIBS)
LoaderRWS.load(RWStudio_LIBS)

% Try to open RobWorkStudio and load the workcell
rwstudio = rws.getRobWorkStudioInstance();
wcFile = '/home/thomas/Hentede filer/rob_scenes/PA10WorkCell/Scene.wc.xml';
wc = WorkCellFactory.load(wcFile);
rwstudio.postWorkCell(wc);

%% Main Section
% Find the device and relevant frames
arm = wc.findDevice('PA10');
gripper = wc.findFrame('Tool');
object = wc.findFrame('Bottle');
table = wc.findFrame('Table');

% Get the current state from RobWorkStudio
state = rwstudio.getState();
q = arm.getQ(state);

% Create PathTimedState
path = PathTimedState();
path.clear();

% Add configurations
addQ(path, 1, toQ([0 0.1 0 0 0 0 0]), arm, state);
addQ(path, 1, toQ([0 0.1 0 0 0 0 0]), arm, state);
addQ(path, 2, toQ([0 0.2 0 0 0 0 0]), arm, state);
addQ(path, 3, toQ([0 0.3 0 0 0 0 0]), arm, state);
addQ(path, 4, toQ([0 0.4 0 0 0 0 0]), arm, state);
addQ(path, 5, toQ([0 0.5 0 0 0 0 0]), arm, state);
addQ(path, 6, toQ([0 0.6 0 0 0 0 0]), arm, state);
addQ(path, 7, toQ([0 0.7 0 0 0 0 0]), arm, state);

% Grasp the object
graspObject(path, object, gripper, state);

% Add configurations
addQ(path, 8, toQ([0 0.7 0 0 0 0 0]), arm, state);
addQ(path, 9, toQ([0 0.6 0 0 0 0 0]), arm, state);
addQ(path, 10, toQ([0 0.5 0 0 0 0 0]), arm, state);
addQ(path, 11, toQ([0 0.4 0 0 0 0 0]), arm, state);
addQ(path, 12, toQ([0 0.3 0 0 0 0 0]), arm, state);
addQ(path, 13, toQ([0 0.2 0 0 0 0 0]), arm, state);
addQ(path, 14, toQ([0 0.1 0 0 0 0 0]), arm, state);

% Set object on table
graspObject( path, object, table, state );

% Add configurations
addQ(path, 15, toQ([0 0.1 0 0 0 0 0]), arm, state);
addQ(path, 16, toQ([0 0.1 0 0 0 0 0]), arm, state);
addQ(path, 18, toQ([0 0.2 0 0 0 0 0]), arm, state);
addQ(path, 19, toQ([0 0.3 0 0 0 0 0]), arm, state);

% Set path in RobWorkStudio
% Use the PlayBack module to execute
rwstudio.setTimedStatePath(PathTimedStatePtr(path));

%% Exit RobWorkStudio
rwstudio.postExit();