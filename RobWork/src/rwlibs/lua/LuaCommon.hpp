
/**
 * We need to specify the wrapper classes,
 */
#include "LuaMath.hpp"
#include <rw/common.hpp>

#ifndef RWLIBS_LUA_COMMON_HPP
#define RWLIBS_LUA_COMMON_HPP


namespace rwlibs {
namespace lua {
namespace common {

	void info(const std::string& msg);

	void debug(const std::string& msg);

	void warn(const std::string& msg);

	void error(const std::string& msg);

}}}


#endif
