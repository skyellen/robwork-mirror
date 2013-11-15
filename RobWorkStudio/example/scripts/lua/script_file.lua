-- EXAMPLE HEADER STARTS HERE

-- helper function - adds a configuration to the state path
function addQ(tpath, time, arr)
 --arm:setQ( rw.Q(#arr,arr), state) -- set the device configuration in state
 --tpath:add( time, state)
end

-- helper function - add a grasp to the state function
function graspObject(tpath, obj, tool)
 rw.gripFrame(state, obj, tool)
 time = tpath[tpath:size()-1]:getTime()
 tpath:add(time, state)
end

-- load workcell
wc = rwstudio:getWorkCell()
state = wc:getDefaultState() -- the state

-- EXAMPLE BODY STARTS HERE

-- load device, gripper, object and state
arm = wc:findDevice("KukaKr16") -- robot arm


--gripper = wc:findFrame("PG70.TCP") -- the end effector frame which is the gripper
--object = wc:findFrame("Bottle") -- the object which is to be grasped
--table = wc:findFrame("Table") -- the object which is to be grasped

print(arm)
-- we store everything in a timed state path
tpath = rw.TimedStatePath:new()
print(tpath)
reflect(tpath)
addQ( tpath, 1, {0,0.1,0,0,0,0} )
addQ( tpath, 2, {0,0.2,0,0,0,0} )
addQ( tpath, 3, {0,0.3,0,0,0,0} )
addQ( tpath, 4, {0,0.4,0,0,0,0} )
addQ( tpath, 5, {0,0.5,0,0,0,0} )
addQ( tpath, 6, {0,0.6,0,0,0,0} )
addQ( tpath, 7, {0,0.7,0,0,0,0} )
-- grasp the object
-- graspObject( tpath, object, gripper)
-- samples are int the form add(time, Q)
addQ( tpath,  8, {0,0.7,0,0,0,0} )
addQ( tpath,  9, {0,0.6,0,0,0,0} )
addQ( tpath,  10, {0,0.5,0,0,0,0} )
addQ( tpath,  11, {0,0.4,0,0,0,0} )
addQ( tpath,  12, {0,0.3,0,0,0,0} )
addQ( tpath,  13, {0,0.2,0,0,0,0} )
addQ( tpath, 14, {0,0.1,0,0,0,0} )
 
-- graspObject( tpath, object, table)

addQ( tpath, 15, {0,0.1,0,0,0,0} )
addQ( tpath, 16, {0,0.1,0,0,0,0} )
addQ( tpath, 18, {0,0.2,0,0,0,0} )
addQ( tpath, 19, {0,0.3,0,0,0,0} )

rwstudio:setTimedStatePath( tpath )  
} )
addQ( tpath, 16, {0,0.1,0,0,0,0} )
addQ( tpath, 18, {0,0.2,0,0,0,0} )
addQ( tpath, 19, {0,0.3,0,0,0,0} )

rwstudio:setTimedStatePath( tpath )  
