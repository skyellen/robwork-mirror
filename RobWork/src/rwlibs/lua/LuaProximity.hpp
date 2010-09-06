
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

	class CollisionSetup: public rw::proximity::CollisionSetup
	{
	public:
		CollisionSetup(const rw::proximity::CollisionSetup& setup);
	};

	// @}
}}


#endif
