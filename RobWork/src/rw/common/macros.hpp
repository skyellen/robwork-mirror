/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#ifndef rw_common_macros_HPP
#define rw_common_macros_HPP

/**
 * @file macros.hpp
 */

#include "Exception.hpp"
#include "Log.hpp"
#include "Message.hpp"
#include "IOUtil.hpp"

#include <sstream>
#include <iostream>

/** @addtogroup common */
/*@{*/

/**
 * @brief Throw an exception with message \b ostreamExpression.
 *
 * \b ostreamExpression is an expression that is fed to an output stream. Example:
 * \code
 *  RW_THROW("The value of x is " << x);
 * \endcode
 *
 * Exception messages can be intercepted via exceptionLog().
 */
#define RW_THROW(ostreamExpression) do { int RW__line = __LINE__;           \
    std::stringstream RW__stream;                                           \
    RW__stream << ostreamExpression;                                        \
    rw::common::Message RW__message(__FILE__, RW__line, RW__stream.str());  \
    rw::common::Log::errorLog().write(RW__message);        \
    throw rw::common::Exception(RW__message);                               \
} while (0)
// We use the weird RW__ names to (hopefully) avoid name crashes.



/**
 * @brief Throw an exception with the specified id and message \b ostreamExpression.
 *
 * \b id is the id of the exception and * \b ostreamExpression is an expression that 
 * is fed to an output stream. Example:
 * \code
 *  RW_THROW("The value of x is " << x);
 * \endcode
 *
 * Exception data can be intercepted via exceptionLog().
 */
#define RW_THROW2(id, ostreamExpression) do { int RW__line = __LINE__;           \
    std::stringstream RW__stream;                                           \
    RW__stream << ostreamExpression;                                        \
    rw::common::Message RW__message(__FILE__, RW__line, RW__stream.str());  \
    rw::common::Exception exp(id, RW__message);                               \
    rw::common::Log::errorLog().write(exp.what());                              \
    throw exp;                                                                  \
} while (0)
// We use the weird RW__ names to (hopefully) avoid name crashes.


/**
 * @brief Emit a warning.
 *
 * \b ostreamExpression is an expression that is fed to an output stream. Example:
\code
RW_WARN("The value of x is " << x << ". x should be less than zero.");
\endcode
 *
 * Warning messages can be intercepted via warningLog().
 */
#define RW_WARN(ostreamExpression) do { int RW__line = __LINE__;            \
    std::stringstream RW__stream;                                           \
    RW__stream << ostreamExpression;                                        \
    rw::common::Message RW__message(__FILE__, RW__line, RW__stream.str());  \
    rw::common::Log::warningLog().write(RW__message);      \
    rw::common::Log::warningLog() << std::endl;\
} while (0)

/**
 * @brief Emit debug message.
 *
 * \b ostreamExpression is an expression that is fed to an output stream. Example:
\code
RW_DEBUG("The value of x is " << x << ". x should be less than zero.");
 *
\endcode
 * Warning messages can be intercepted via debugLog().
 */
#ifdef RW_DEBUG_ENABLE
#define RW_DEBUG(ostreamExpression)                                         \
do { int RW__line = __LINE__;                                               \
    std::stringstream RW__stream;                                           \
    RW__stream << ostreamExpression;                                        \
    rw::common::Message RW__message(__FILE__, RW__line, RW__stream.str());  \
    rw::common::Log::debugLog().write( RW__message);        \
} while (0)
#else
#define RW_DEBUG(ostreamExpression)
#endif

/**
 * @brief For internal use only.
 */
#define RW_ASSERT_IMPL(e, file, line) \
    ((e) ? (void)0 : rw::common::IOUtil::rwAssert(#e, file, line))

/**
 * @brief RobWork assertions.
 *
 * RW_ASSERT() is an assertion macro in the style of assert(). RW_ASSERT() can
 * be enabled by compiling with RW_ENABLE_ASSERT defined. Otherwise RW_ASSERT()
 * is enabled iff NDBUG is not defined.
 *
 * You should prefer RW_ASSERT() to assert() everywhere with a possible
 * exception being (unsafe) access of arrays and a few other places where you
 * are sure that a run time sanity check will be a performance issue.
 */
#ifdef RW_ENABLE_ASSERT
#  define RW_ASSERT(e) RW_ASSERT_IMPL(e, __FILE__, __LINE__)
#else
#  ifdef NDEBUG
#    define RW_ASSERT(e)
#  else
#    define RW_ASSERT(e) RW_ASSERT_IMPL(e, __FILE__, __LINE__)
#  endif
#endif

/**
 * @brief Writes \b ostreamExpression to the log identified by \b id
 *
 * \b ostreamExpression is an expression that is fed to an output stream.
 *
 * Example:
 * \code
 * int x = 1;
 * RW_LOG_TEXT(Log::warningId(), "Warning: The value of x " << x << " is too small");
 * \endcode
 *
 * @param id [in] Identifier for log
 * @param ostreamExpression [in] Stream expression which should be written to the log
 */
#define RW_LOG_TEXT(id, ostreamExpression) do {     \
    std::stringstream RW__stream;                   \
    RW__stream << ostreamExpression;                \
    rw::common::Log::log().write(id, RW__stream.str());   \
} while (0)


/**
 * @brief Writes \b ostreamExpression augmented with file name and line number
 * to the log identified by \b id.
 *
 * \b ostreamExpression is an expression that is fed to an output stream.
 *
 * The example:
 * \code
 * RW_LOG(Log::errorId(), "Invalid input");
 * \endcode
 * will result in an output looking like \b {Filename:Line Invalid Input}
 *
 * @param id [in] Identifier for log
 * @param ostreamExpression [in] Stream expression which should be written to the log
 */
#define RW_LOG(id, ostreamExpression) do {                                      \
    std::stringstream RW__stream;                                               \
    RW__stream << ostreamExpression << "\n";                                    \
    rw::common::Log::log().get(id).write(Messages(RW__stream.str(), __LINE__, __FILE__);   \
} while (0)


/**
 * @brief Writes \b ostreamExpression to \b log.
 *
 * \b log be of type rw::common::LogWriter or have a write(const std::string&) method.
 *
 * \b ostreamExpression is an expression that is fed to an output stream.
 *
 * Example:
 * \code
 * int x = 1;
 * RW_WRITE_LOG(myLogWriter, "Warning: The value of x " << x << " is too small");
 * \endcode
 *
 * @param log [in] LogWriter to write to
 * @param ostreamExpression [in] Stream expression which should be written to the log
 */
#define RW_WRITE_LOG(log, ostreamExpression) do {                               \
    std::stringstream RW__stream;                                               \
    RW__stream << ostreamExpression << "\n";                                    \
    log.write(RW__stream.str());                                                \
} while (0)
/*@}*/

#define USE_ROBWORK_NAMESPACE \
	namespace rw { namespace proximity {}} \
	namespace rw { namespace common {}} \
	namespace rw { namespace control {}} \
	namespace rw { namespace geometry {}} \
	namespace rw { namespace interpolator {}} \
	namespace rw { namespace trajectory {}} \
	namespace rw { namespace invkin {}} \
	namespace rw { namespace kinematics {}} \
	namespace rw { namespace math {}} \
	namespace rw { namespace task {}} \
	namespace rw { namespace models {}} \
	namespace rw { namespace pathplanning {}} \
	namespace rw { namespace sensor {}} \
	namespace rw { namespace loaders {}} \
	namespace robwork \
	{ \
		using namespace rw; \
		using namespace rw::proximity; \
		using namespace rw::common; \
		using namespace rw::control; \
		using namespace rw::geometry; \
		using namespace rw::interpolator; \
		using namespace rw::trajectory; \
		using namespace rw::invkin; \
		using namespace rw::kinematics; \
		using namespace rw::math; \
		using namespace rw::task; \
		using namespace rw::models; \
		using namespace rw::pathplanning; \
		using namespace rw::sensor; \
		using namespace rw::loaders; \
	}


#endif // end include guard
