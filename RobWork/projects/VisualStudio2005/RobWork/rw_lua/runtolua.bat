set ROOT=%1

%ROOT%\tolua -o ../../../../src/rwlibs/lua/LuaMathStub.cpp -H ../../../../src/rwlibs/lua/LuaMathStub.hpp -n LuaMath ../../../../src/rwlibs/lua/LuaMath.pkg
%ROOT%\tolua -o ../../../../src/rwlibs/lua/LuaKinematicsStub.cpp -H ../../../../src/rwlibs/lua/LuaKinematicsStub.hpp -n LuaKinematics ../../../../src/rwlibs/lua/LuaKinematics.pkg
%ROOT%\tolua -o ../../../../src/rwlibs/lua/LuaLoadersStub.cpp -H ../../../../src/rwlibs/lua/LuaLoadersStub.hpp -n LuaLoaders ../../../../src/rwlibs/lua/LuaLoaders.pkg
%ROOT%\tolua -o ../../../../src/rwlibs/lua/LuaModelsStub.cpp -H ../../../../src/rwlibs/lua/LuaModelsStub.hpp -n LuaModels ../../../../src/rwlibs/lua/LuaModels.pkg
%ROOT%\tolua -o ../../../../src/rwlibs/lua/LuaSensorStub.cpp -H ../../../../src/rwlibs/lua/LuaSensorStub.hpp -n LuaSensor ../../../../src/rwlibs/lua/LuaSensor.pkg
%ROOT%\tolua -o ../../../../src/rwlibs/lua/LuaTrajectoryStub.cpp -H ../../../../src/rwlibs/lua/LuaTrajectoryStub.hpp -n LuaTrajectory ../../../../src/rwlibs/lua/LuaTrajectory.pkg
%ROOT%\tolua -o ../../../../src/rwlibs/lua/LuaProximityStub.cpp -H ../../../../src/rwlibs/lua/LuaProximityStub.hpp -n LuaProximity ../../../../src/rwlibs/lua/LuaProximity.pkg
%ROOT%\tolua -o ../../../../src/rwlibs/lua/LuaCommonStub.cpp -H ../../../../src/rwlibs/lua/LuaCommonStub.hpp -n LuaCommon ../../../../src/rwlibs/lua/LuaCommon.pkg
%ROOT%\tolua -o ../../../../src/rwlibs/lua/LuaPathPlanningStub.cpp -H ../../../../src/rwlibs/lua/LuaPathPlanningStub.hpp -n LuaPathPlanning ../../../../src/rwlibs/lua/LuaPathPlanning.pkg