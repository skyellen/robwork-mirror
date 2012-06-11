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


#ifndef RW_COMMON_MESSAGE_HPP
#define RW_COMMON_MESSAGE_HPP

/**
 * @file Message.hpp
 */

#include <iostream>
#include <string>
#include <sstream>

namespace rw { namespace common {

    /** @addtogroup common */
    /*@{*/

    /**
     * @brief Standard type for user messages of robwork.
     *
     * Messages are used for exception, warnings, and other things that are
     * reported to the user.
     *
     * Message values should contain the source file name and line number so
     * that it is easy to look up the place in the code responsible for the
     * generation of the message.
     *
     * RW_THROW and RW_WARN of macros.hpp have been introduced for the throwing
     * of exceptions and emission of warnings.
     */
    class Message
    {
    public:
        /**
         * @brief Constructor
         *
         * Messages of RobWork are all annotated by the originating file name,
         * the originating line number, and a message text for the user.
         *
         * Supplying all the file, line, and message parameters can be a little
         * painfull, so a utility for creating messages is available from the
         * file macros.hpp.
         *
         * @param file [in] The source file name.
         *
         * @param line [in] The source file line number.
         *
         * @param message [in] A message for a user.
         */
        Message(const std::string& file,
                int line,
                const std::string& message = "");

        /**
         * @brief The name of source file within which the message was
         * constructed.
         *
         * @return The exception file name.
         */
        const std::string& getFile() const { return _file; }

        /**
         * @brief The line number for the file at where the message was
         * constructed.
         *
         * @return The exception line number.
         */
        int getLine() const { return _line; }

        /**
         * @brief The message text meant for the user.
         *
         * @return The message text.
         */
        const std::string& getText() const { return _message; }

        /**
         * @brief Returns a full description of the message containing file, line number and message.
         */
        std::string getFullText() const {
            std::stringstream sstr;
            sstr<<_file<<":"<<_line<<" : "<<_message;
            return sstr.str();
        }

        /**
         * @brief general stream operator
         */
        template< class T>
        Message& operator<<( T t ){
            std::stringstream tmp;
            tmp << t;
			_message = _message + tmp.str();
			return *this;
            //return this->operator<<( tmp.str() );
        }

        ///**
        // * @brief specialized stream operator
        // */
        //Message& operator<<(const std::string& str){
        //	_message = _message + str;
        //	return *this;
        //}


        ///**
        // * @brief specialized stream operator
        // */
        //Message& operator<<(const char* str){
        //	_message = _message + str;
        //	return *this;
        //}

        /**
         * @brief Handle the std::endl and other stream functions.
         */
		//Message& operator<<(std::ostream& (*pf)(std::ostream&)) {
		//	std::ostringstream buf;
		//	buf << pf;
		//	_message = _message + buf.str();			
		//	return *this;
		//}



    private:
        std::string _file;
        int _line;
        std::string _message;
    };

    /** @brief Format to \b out the message \b msg.
     *
     * The format for the exception is
\code
<file>:<line> <message>
\endcode
     * @return The stream \b out.
     */
    std::ostream& operator<<(std::ostream& out, const  Message& msg);

    /*@}*/
}} // end namespaces

#endif // end include guard
