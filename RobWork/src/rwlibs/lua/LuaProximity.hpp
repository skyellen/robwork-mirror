
/**
 * We need to specify the wrapper classes,
 */
#include "LuaMath.hpp"
#include <rw/proximity.hpp>

#ifndef RWLIBS_LUA_PROXIMITY_HPP
#define RWLIBS_LUA_PROXIMITY_HPP


namespace rwlibs {
namespace lua {

    //! @addtogroup lua
    // @{

    /**
     * @brief lua wrapper class for rw::proximity::CollisionSetup
     */
	class CollisionSetup: public rw::proximity::CollisionSetup
	{
	public:
	    //! @brief copy-constructor
		CollisionSetup(const rw::proximity::CollisionSetup& setup);
	};

	// @}
}}


#endif
