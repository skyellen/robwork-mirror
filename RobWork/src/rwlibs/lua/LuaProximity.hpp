
/**
 * We need to specify the wrapper classes,
 */
#include "LuaMath.hpp"
#include "LuaKinematics.hpp"
#include <rw/proximity.hpp>

#ifndef RWLIBS_LUA_PROXIMITY_HPP
#define RWLIBS_LUA_PROXIMITY_HPP


namespace rwlua {
namespace rw {

    //! @addtogroup lua
    // @{

    /**
     * @brief lua wrapper class for rw::proximity::CollisionSetup
     */
    typedef ::rw::proximity::CollisionSetup CollisionSetup;
    typedef ::rw::proximity::ProximityPair ProximityPair;
    typedef ::rw::proximity::ProximityPair ProximityPairList;

	// @}
}}


#endif
