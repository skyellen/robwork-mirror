function addQ( pathTimedState, time, q, device, state )
%addQ Add a RobWork Q vector to a RobWork PathTimedState
    import dk.robwork.Q;
    import dk.robwork.TimedState;
    if (strcmp(class(q),'dk.robwork.Q'))
        device.setQ(q, state);
    else
        qRW = Q(length(q),toJavaDoubleArray(q).cast());
        device.setQ(qRW,state);
    end
    pathTimedState.add(TimedState(time, state));
end