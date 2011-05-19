
/**
 * We need to specify the wrapper classes,
 */
#include "LuaMath.hpp"
#include <rw/common.hpp>

#ifndef RWLUA_RW_COMMON_HPP
#define RWLUA_RW_COMMON_HPP

namespace rwlua {
namespace rw {
    //! @addtogroup lua
    // @{

    /**
     * @brief write \b msg to info log
     * @param msg [in] string
     */
	void info(const std::string& msg);

    /**
     * @brief write \b msg to debug log
     * @param msg [in] string
     */
	void debug(const std::string& msg);

    /**
     * @brief write \b msg to warn log
     * @param msg [in] string
     */
	void warn(const std::string& msg);

    /**
     * @brief write \b msg to error log
     * @param msg [in] string
     */
	void error(const std::string& msg);

    /**
     * @brief write \b msg to lua log
     * @param msg [in] string
     */
	void lualog(const std::string& msg);

	void setLualog(::rw::common::LogWriter::Ptr writer);

	/**
	 * @brief sleep time \b t in seconds
	 * @param t
	 */
	void sleep(double t);


    // @}
}}

#endif
