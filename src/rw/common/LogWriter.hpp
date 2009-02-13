/*********************************************************************
 * RobWork Version 0.3
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

#ifndef RW_COMMON_LOGWRITER_HPP
#define RW_COMMON_LOGWRITER_HPP

#include <string>
#include <sstream>
#include "Message.hpp"

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

        /**
         * @brief Descructor
         */
        virtual ~LogWriter();

        /**
         * @brief Flush method
         */
        virtual void flush() = 0;

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

	/* @} */

}} // end namespaces
#endif /*RW_COMMON_LOGWRITER_HPP*/
