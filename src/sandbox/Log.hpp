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

#ifndef RW_COMMON_SANDBOX_LOG_HPP
#define RW_COMMON_SANDBOX_LOG_HPP

/**
   @file Log.hpp
*/

#include <string>
#include <vector>
#include <map>
#include <sstream>
#include <iostream>

#include <rw/common/LogWriter.hpp>
#include <rw/common/Message.hpp>
#include <rw/common/Ptr.hpp>

namespace rw { namespace common {
	typedef rw::common::Ptr<rw::common::LogWriter> LogWriterPtr;
namespace sandbox {

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
     *
     *
     * Log::log() << blabla << std::endl;
     * Log::log(info) << blabla
     *
     *
     *
     */
    class Log
    {
    public:

    	enum LogLevelMask {
    		FatalMask=1, CriticalMask=2,
    		ErrorMask=4, WarningMask=8,
    		InfoMask=16, DebugMask=32,
    		User1Mask=64, User2Mask=128,
    		User3Mask=256, User4Mask=512,
    		User5Mask=1024, User6Mask=2048,
    		User7Mask=4096, User8Mask=8096
    	};

    	enum LogLevel {
    		Fatal=0, Critical=1,
    		Error=2, Warning=3,
    		Info=4, Debug=5,
    		User1=6, User2=7,
    		User3=8, User4=9,
    		User5=10, User6=11,
    		User7=12, User8=13
    	};

    	/**
    	 * @brief convenience function for getting the LogWriter
    	 * that is associated with the info log level
    	 * @return info LogWriter
    	 */
        static LogWriter& infoLog(){ return Log::log().get(Info);};

    	/**
    	 * @brief convenience function for getting the LogWriter
    	 * that is associated with the warning log level
    	 * @return info LogWriter
    	 */
        static LogWriter& warningLog(){ return Log::log().get(Warning);};

    	/**
    	 * @brief convenience function for getting the LogWriter
    	 * that is associated with the error log level
    	 * @return info LogWriter
    	 */
        static LogWriter& errorLog(){ return Log::log().get(Error);};

    	/**
    	 * @brief convenience function for getting the LogWriter
    	 * that is associated with the debug log level
    	 * @return info LogWriter
    	 */
        static LogWriter& debugLog(){ return Log::log().get(Debug);};

    	/**
    	 * @brief returns the global log instance. Global in the sence
    	 * of whatever is linked staticly together.
    	 * @return a Log
    	 */
        static Log& getInstance();

        /**
         * @brief convenience function of getInstance
         * @return a Log
         */
        static Log& log();

        //************************* Here follows the member interface

        /**
         * @brief constructor
         */
        Log();

        /**
         * @brief Destructor
         */
        virtual ~Log();

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
        void setWriter(const std::string& id, LogWriterPtr writer);
        void setWriter(LogLevel id, LogWriterPtr writer);

        /**
         * @brief Returns the LogWriter associated with \b id
         *
         * If the \b id is unknown an exception is thrown.
         *
         * @param id [in] Log identifier
         * @return Reference to LogWriter object
         */
        LogWriter& get(const std::string& id);
        LogWriter& get(LogLevel id);

        /**
         * @brief Writes \b message to the log
         *
         * If the \b id cannot be found an exception is thrown
         *
         * @param id [in] Log identifier
         * @param message [in] String message to write
         */
        void write(LogLevel id, const std::string& message);
        void write(const std::string& id, const std::string& message);

        /**
         * @brief Writes \b message to the log
         *
         * If the \b id cannot be found an exception is thrown
         *
         * @param id [in] Log identifier
         * @param message [in] Message to write
         */
        void write(LogLevel id, const Message& message);
        void write(const std::string& id, const Message& message);

        /**
         * @brief Writes \b message followed by a '\\n' to the log
         *
         * If the \b id cannot be found an exception is thrown
         *
         * @param id [in] Log identifier
         * @param message [in] Message to write
         */
        void writeln(LogLevel id, const std::string& message);
        void writeln(const std::string& id, const std::string& message);

        /**
         * @brief Calls flush on the specified log
         *
         * If the \b id cannot be found an exception is thrown
         *
         * @param id [in] Log identifier
         */
        void flush(LogLevel id);
        void flush(const std::string& id);

        /**
         * @brief Calls flush on all logs
         */
        void flushAll();

        /**
         * @brief Removes a log
         *
         * If the \b id cannot be found an exception is thrown
         *
         * @param id [in] Log identifier
         */
        void remove(const std::string& id);
        void remove(LogLevel id);

    	/**
    	 * @brief convenience function for getting the LogWriter
    	 * that is associated with the info log level
    	 * @return info LogWriter
    	 */
        rw::common::LogWriter& info(){ return get(Info);};

    	/**
    	 * @brief convenience function for getting the LogWriter
    	 * that is associated with the warning log level
    	 * @return info LogWriter
    	 */
        rw::common::LogWriter& warning(){ return get(Warning);};

    	/**
    	 * @brief convenience function for getting the LogWriter
    	 * that is associated with the error log level
    	 * @return info LogWriter
    	 */
        rw::common::LogWriter& error(){ return get(Error);};

    	/**
    	 * @brief convenience function for getting the LogWriter
    	 * that is associated with the debug log level
    	 * @return info LogWriter
    	 */
        rw::common::LogWriter& debug(){ return get(Debug);};

        /**
         * @brief the log level is a runtime handle for enabling/disabling
         * logging to specific log levels.
         * @param loglvl
         */
        void setLogLevelMask( int loglvl ){ _logLevelMask=loglvl; };

        /**
         * @brief get the current log level
         * @return the loglevel
         */
        int getLogLevelMask() const{ return _logLevelMask; };

    private:
    	bool isValidLogLevel(LogLevel id);

        typedef std::map<std::string, rw::common::LogWriterPtr > Map;
		int _logLevelMask;
		std::vector<rw::common::LogWriterPtr> _writers;
		Map _userWriters;
		rw::common::LogWriterPtr _defaultWriter;
    };


    /*@}*/
}}} // end namespaces
#endif /*LOG_HPP_*/
