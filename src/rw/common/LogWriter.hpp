#ifndef RW_COMMON_LOGWRITER_HPP
#define RW_COMMON_LOGWRITER_HPP

#include <string>

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
         * @brief Writes \b message to the log
         * 
         * Default behavior is to use message.getText() and call write(const std::string&) 
         * 
         * @param message [in] message to write
         */
        virtual void write(const Message& message)
        {
            write(message.getText());
        }
	
        /**
         * @brief Writes \b str as a line
         * 
         * By default writeln writes \b str followed by a '\\n'. However, logs
         * are free to implement a line change differently.
         */
        virtual void writeln(const std::string& str)
        {
            write(str + '\n');        
        }
    };

	/* @} */

}} // end namespaces
#endif /*RW_COMMON_LOGWRITER_HPP*/
