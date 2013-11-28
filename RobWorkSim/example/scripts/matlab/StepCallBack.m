function StepCallBack( dispatcherObject, event, rwstudio )
    import dk.robwork.rw;
    
    % Extract the event data
    tsim=event.getThreadSimulator();
    state=event.getState();
    
    % Extract user data stored on dispatcher object
    userdata=getappdata(dispatcherObject,'UserData');
    counter = userdata(1);
    
    % Set state in RobWorkStudio and print time for each 20 steps
    if mod(counter,20) == 0
        rwstudio.setState(state);
        display(num2str(tsim.getTime()));
        rw.writelog(strcat(num2str(tsim.getTime()),'\\r\\n'));
    end

    % Update step counter
    setappdata(dispatcherObject,'UserData',[counter+1]);
end