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


#ifndef RW_COMMON_EXCEPTION_HPP
#define RW_COMMON_EXCEPTION_HPP

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


    /**
     * @brief Format to \b out the message of the exception \b exp.
     *
     * The format for the text is
	 *	\code
	 * 	<file>:<line> <message>
	 *	\endcode
     * @return The stream \b out.
     */
    std::ostream& operator<<(std::ostream& out, const Exception& exp);

    /*@}*/
}} // end namespaces

#endif // end include guard
