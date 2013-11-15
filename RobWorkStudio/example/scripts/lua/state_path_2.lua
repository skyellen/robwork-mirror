-- define namespace shortcut
rw = rwlibs.lua -- robwork namespace
rws = rws.lua.rwstudio -- robworkstudio namespace
rwstudio = rws.getRobWorkStudio()

-- load workcell
wc = rwstudio:getWorkcell()

-- load device, gripper, object and state
arm = wc:findDevice("PA10") -- robot arm
gripper = wc:findFrame("Tool") -- the end effector frame which is the gripper
object = wc:findFrame("Bottle") -- the object which is to be grasped
state = wc:getDefaultState() -- the state

-- function for creating timed state path
function add_to_state_path(path, dev, state, tpath)
	t_last = path[0]:getTime()
	for qidx=0, path:size()-1 do
		q = path[qidx]:getValue() -- get the next configuration
		t = path[qidx]:getTime() -- get the next configuration
		dev:setQ(q, state) -- set the device configuration in state
		tpath:add( t, state)
	end
end

-- we store everything in a timed state path
timedstatepath = rw.TimedStatePath:new()

-- create a time q path and add samples in it, this could be put in a file and called with: dofile "pathfile.lua"
path = rw.TimedQPath:new()
-- samples are int the form add(time, Q)
path:add( 1, {0,0.1,0,0,0,0,0} )
path:add( 2, {0,0.2,0,0,0,0,0} )
path:add( 3, {0,0.3,0,0,0,0,0} )
path:add( 4, {0,0.4,0,0,0,0,0} )
path:add( 5, {0,0.5,0,0,0,0,0} )
path:add( 6, {0,0.6,0,0,0,0,0} )
path:add( 7, {0,0.7,0,0,0,0,0} )

-- now move the robot from start to end of path
add_to_state_path(path, arm, state, timedstatepath) 

-- now pick up the object
rw.gripFrame(state, object, gripper)
timedstatepath:add(path[path:size()-1]:getTime(), state)

path = rw.TimedQPath:new()
-- samples are int the form add(time, Q)
path:add( 8, {0,0.7,0,0,0,0,0} )
path:add( 9, {0,0.6,0,0,0,0,0} )
path:add( 10, {0,0.5,0,0,0,0,0} )
path:add( 11, {0,0.4,0,0,0,0,0} )
path:add( 12, {0,0.3,0,0,0,0,0} )
path:add( 13, {0,0.2,0,0,0,0,0} )
path:add( 14, {0,0.1,0,0,0,0,0} )

add_to_state_path(path, arm, state, timedstatepath) 

rwstudio:setTimedStatePath( timedstatepath )

