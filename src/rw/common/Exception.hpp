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

#ifndef rw_common_Exception_HPP
#define rw_common_Exception_HPP

/**
 * @file Exception.hpp
 */

#include "Message.hpp"

#include <iostream>
#include <string>

namespace rw { namespace common {

    /** @addtogroup common */
    /*@{*/

    /** @brief Standard exception type of RobWork.
     *
     * All exception thrown within RobWork are of the type Exception.
     *
     * An exception contains a message (of type Message) for the user and
     * nothing else.
     */
    class Exception
    {
    public:
        /** @brief Constructor
         *
         * @param message [in] A message for a user.
         */
        Exception(const Message& message) :
            _message(message)
        {}

        /** @brief The message for the user describing the reason for the error.
         *
         * @return  The message for the user.
         */
        const Message& getMessage() const { return _message; }

    private:
        Message _message;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
