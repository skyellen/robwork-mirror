
/**
 * We need to specify the wrapper classes,
 */
#include "LuaMath.hpp"
#include <rw/trajectory.hpp>

#ifndef RWLIBS_LUA_TRAJECTORY_HPP
#define RWLIBS_LUA_TRAJECTORY_HPP


namespace rwlibs {
namespace lua {
namespace trajectory {

	class QPath: public rw::trajectory::QPath
	{
	public:
		QPath(const rw::trajectory::QPath& path);
	};

	class StatePath: public rw::trajectory::StatePath
	{
	public:
		StatePath(const rw::trajectory::StatePath& path);
	};

	class TimedStatePath: public rw::trajectory::TimedStatePath
	{
	public:
		TimedStatePath(const rw::trajectory::TimedStatePath& path);
	};



}}}


#endif
