/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef rw_common_macros_HPP
#define rw_common_macros_HPP

/**
 * @file macros.hpp
 */

#include "Exception.hpp"
#include "Log.hpp"
#include "IOUtil.hpp"

#include <sstream>
#include <iostream>

/** @addtogroup common */
/*@{*/

/** @brief Throw an exception with message \b ostreamExpression.
 *
 * \b ostreamExpression is an expression that is fed to an output stream. Example:
\code
RW_THROW("The value of x is " << x);
\endcode
 *
 * Exception messages can be intercepted via exceptionLog().
 */
#define RW_THROW(ostreamExpression) do { int RW__line = __LINE__;                       \
    std::stringstream RW__stream;                                                       \
    RW__stream << ostreamExpression;                                                    \
    rw::common::Message RW__message(__FILE__, RW__line, RW__stream.str());        \
    rw::common::exceptionLog().get().emitMessage(RW__message);                    \
    throw rw::common::Exception(RW__message);                                     \
    } while (0)
// We use the weird RW__ names to (hopefully) avoid name crashes.

/** @brief Emit a warning.
 *
 * \b ostreamExpression is an expression that is fed to an output stream. Example:
\code
RW_WARN("The value of x is " << x << ". x should be less than zero.");
\endcode
 *
 * Warning messages can be intercepted via warningLog().
 */
#define RW_WARN(ostreamExpression) do { int RW__line = __LINE__;                        \
    std::stringstream RW__stream;                                                       \
    RW__stream << ostreamExpression;                                                    \
    rw::common::Message RW__message(__FILE__, RW__line, RW__stream.str());        \
    rw::common::warningLog().get().emitMessage(RW__message);                      \
} while (0)
// We use the weird RW__ names to (hopefully) avoid name crashes.

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

/*@}*/

#endif // end include guard
