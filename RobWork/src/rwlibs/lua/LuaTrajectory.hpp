
/**
 * We need to specify the wrapper classes,
 */
#include "LuaMath.hpp"
#include <rw/trajectory.hpp>

#ifndef RWLIBS_LUA_TRAJECTORY_HPP
#define RWLIBS_LUA_TRAJECTORY_HPP


namespace rwlibs {
namespace lua {

    //! @addtogroup lua
    // @{

    /**
     * @brief lua wrapper class for rw::trajectory::QPath
     */
	class QPath: public rw::trajectory::QPath
	{
	public:
		QPath(const rw::trajectory::QPath& path);
	};

    /**
     * @brief lua wrapper class for rw::trajectory::StatePath
     */
	class StatePath: public rw::trajectory::StatePath
	{
	public:
		StatePath(const rw::trajectory::StatePath& path);
	};

    /**
     * @brief lua wrapper class for rw::trajectory::TimedStatePath
     */
	class TimedStatePath: public rw::trajectory::TimedStatePath
	{
	public:
		TimedStatePath(const rw::trajectory::TimedStatePath& path);
	};

	// @}

}}


#endif
