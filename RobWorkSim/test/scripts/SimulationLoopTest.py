--
-- scene should have two bodies one fixed (FBody) and one RigidBody (RBody) that are constrained by a motorized joint
-- 

-- first load robwork lua interface
rwroot = getenv("RW_ROOT")
rw_init=loadlib(rwroot .. "/libs/release/rwlua.so","luaopen_rw")
assert(rw_init)
rws_init=loadlib(rwroot .. "../RobWorkStudio/libs/release/rwslua.so","luaopen_rws")
assert(rws_init)

rw_init();
rws_init();

-- next 




