
/**
 * We need to specify the wrapper classes,
 */
#include "LuaMath.hpp"
#include <rw/trajectory.hpp>
#include <rw/pathplanning.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rwlibs/pathplanners/prm/PRMPlanner.hpp>
#include <rwlibs/pathoptimization/clearance/ClearanceOptimizer.hpp>
#include <rwlibs/pathoptimization/clearance/MinimumClearanceCalculator.hpp>
#include <rwlibs/pathoptimization/pathlength/PathLengthOptimizer.hpp>

#ifndef RWLIBS_LUA_PATHPLANNER_HPP
#define RWLIBS_LUA_PATHPLANNER_HPP


namespace rwlua {
namespace rw {

    //! @addtogroup lua
    // @{
    //static QToQPlanner makeDefaultQToQPlanner(Device, CollisionDetector, state);
    //static QToQPlanner::Ptr makeDefaultQToQPlanner(DeviceList, CollisionDetector, state);

    typedef ::rw::pathplanning::StopCriteria StopCriteria;
    typedef ::rw::pathplanning::QToQPlanner QToQPlanner;

	// @}

}}


#endif
