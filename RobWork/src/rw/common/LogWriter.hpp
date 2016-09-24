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


#ifndef RW_COMMON_LOGWRITER_HPP
#define RW_COMMON_LOGWRITER_HPP

#include <string>
#include <sstream>
#include "Message.hpp"
#include "Ptr.hpp"

namespace rw { namespace common {

	/** @addtogroup common */
	/*@{*/

	/**
     * @brief Write interface for Logs
     *
     * LogWriter provides an output strategy for a log.
     */
    class LogWriter
    {
    public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<LogWriter> Ptr;

        /**
         * @brief Descructor
         */
        virtual ~LogWriter();

        /**
         * @brief Flush method
         */
        virtual void flush() = 0;

		/**
		 * @brief Set the tab level
		 */
		virtual void setTabLevel(int tabLevel) = 0;


        /**
         * @brief Writes \b str to the log
         * @param str [in] message to write
         */
        virtual void write(const std::string& str) = 0;

        /**
         * @brief Writes \b msg to the log
         *
         * Default behavior is to use write(const std::string&) for the standard
         * streaming representation of \b msg.
         *
         * @param msg [in] message to write
         */
        virtual void write(const Message& msg);

        /**
         * @brief Writes \b str as a line
         *
         * By default writeln writes \b str followed by a '\\n'. However, logs
         * are free to implement a line change differently.
         */
        virtual void writeln(const std::string& str);

        /**
         * @brief general stream operator
         */
        template< class T>
        LogWriter& operator<<( T t ){
            std::stringstream tmp;
            tmp << t;
            return this->operator<<( tmp.str() );
        }

        /**
         * @brief specialized stream operator
         */
        LogWriter& operator<<(const std::string& str){
        	write(str);
        	return *this;
        }

        /**
         * @brief Write Message to log.
         * @param msg [in] the message.
         * @return a reference to this LogWriter for chaining of stream operators.
         */
        LogWriter& operator<<(const Message& msg){
            write(msg);
            return *this;
        }


        /**
         * @brief specialized stream operator
         */
        LogWriter& operator<<(const char* str){
        	write(str);
        	return *this;
        }

        /**
         * @brief Handle the std::endl and other stream functions.
         */
        LogWriter& operator<<(std::ostream& (*pf)(std::ostream&));

    protected:
        LogWriter() {}

    private:
        LogWriter(const LogWriter&);
        LogWriter& operator=(const LogWriter&);
    };

#ifdef RW_USE_DEPRECATED
    //! deprecated smart pointer type
    typedef rw::common::Ptr<rw::common::LogWriter> LogWriterPtr;
#endif
	/* @} */



}} // end namespaces
#endif /*RW_COMMON_LOGWRITER_HPP*/
