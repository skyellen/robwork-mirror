function graspObject( pathTimedState, object, tool, state )
%graspObject Attach a RobWork frame to another frame and add to PathTimedStatePath
    import dk.robwork.rw;
    import dk.robwork.TimedState;
    rw.gripFrame(object, tool, state);
    time = pathTimedState.elem(pathTimedState.size()-1).getTime();
    pathTimedState.add(TimedState(time, state));
end