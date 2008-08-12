#ifndef RW_COMMON_LOG_HPP
#define RW_COMMON_LOG_HPP

/**
   @file Log.hpp
*/

#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
#include <sstream>
#include <iostream>

#include "LogWriter.hpp"
#include "Message.hpp"

namespace rw { namespace common {

	/** @addtogroup common */
	/*@{*/

	/**
     * \brief Provides basic log functionality.
     *
     * The Log class owns a number of LogWriters in a static map, which can be accessed
     * using a string identifier. All logs are global.
     *
     * By default the Log class contains an Info, Warning and Error log. These can be accessed as
     * \code
     * Log::get(Log::infoId()).writeln("This is an info message");
     * Log::get(Log::warningId()).writeln("This is an error message");
     * Log::get(Log::errorId()).writeln("This is an error message");
     * \endcode
     * or
     * \code
     * Log::writeln(Log::infoId(), "Another info message");
     * Log::writeln(Log::warningId(), "Another warning message");
     * Log::writeln(Log::infoId(), "Another error message");
     * \endcode
     * or using one one the RW_LOG, RW_LOGLINE or RW_LOG2 macros, e.g.
     * \code
     * RW_LOG(Log::infoId(), "The value of x is "<<x);
     * RW_LOGLINE(Log::infoId(), "The value of x is "<<x);
     * RW_LOG2(Log::infoId(), "The value of x is "<<x);
     * \endcode
     */
    class Log
    {
    public:
        /**
         * @brief Associates a LogWriter with the \b id.
         *
         * SetWriter can either be used to redefine an existing log or to create a new
         * custom output. The class takes ownership of the log.
         *
         * Example:
         * \code
         * Log::SetWriter("MyLog", new LogStreamWriter(std::cout));
         * RW_LOG("MyLog", "Message send to MyLog");
         * \endcode
         *
         * @param id [in] Identifier for the log
         * @param writer [in] LogWriter object to use
         */
        static void setWriter(const std::string& id, LogWriter* writer);

        /**
         * @brief Returns the LogWriter associated with \b id
         *
         * If the \b id is unknown an exception is thrown.
         *
         * @param id [in] Log identifier
         * @return Reference to LogWriter object
         */
        static LogWriter& get(const std::string& id);

        /**
         * @brief Writes \b message to the log
         *
         * If the \b id cannot be found an exception is thrown
         *
         * @param id [in] Log identifier
         * @param message [in] String message to write
         */
        static void write(const std::string& id, const std::string& message);

        /**
         * @brief Writes \b message to the log
         *
         * If the \b id cannot be found an exception is thrown
         *
         * @param id [in] Log identifier
         * @param message [in] Message to write
         */
        static void write(const std::string& id, const Message& message);

        /**
         * @brief Writes \b message followed by a '\\n' to the log
         *
         * If the \b id cannot be found an exception is thrown
         *
         * @param id [in] Log identifier
         * @param message [in] Message to write
         */
        static void writeln(const std::string& id, const std::string& message);

        /**
         * @brief Calls flush on the specified log
         *
         * If the \b id cannot be found an exception is thrown
         *
         * @param id [in] Log identifier
         */
        static void flush(const std::string& id);

        /**
         * @brief Calls flush on all logs
         */
        static void flushAll();

        /**
         * @brief Removes a log
         *
         * If the \b id cannot be found an exception is thrown
         *
         * @param id [in] Log identifier
         */
        static void remove(const std::string& id);

        /**
         * @brief Identifier for the Info Log
         */
        static const std::string& infoId();

        /**
         * @brief Identifier for the Warning Log
         */
        static const std::string& warningId();

        /**
         * @brief Identifier for the Error Log
         */
        static const std::string& errorId();

        /**
         * @brief Identifier for the Debug Log
         */
        static const std::string& debugId();

    private:
        typedef std::map<std::string, boost::shared_ptr<LogWriter> > Map;
        static Map _map;

        Log();
        virtual ~Log();
    };

    /*@}*/
}} // end namespaces
#endif /*LOG_HPP_*/
