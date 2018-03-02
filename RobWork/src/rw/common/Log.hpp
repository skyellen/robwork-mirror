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


#ifndef RW_COMMON_LOG_HPP
#define RW_COMMON_LOG_HPP

/**
   @file Log.hpp
*/

#include <string>
#include <vector>

#include <rw/common/LogWriter.hpp>
#include <rw/common/Ptr.hpp>

namespace rw { namespace common {
    class Log;
    class Message;


	/** @addtogroup common */
	/*@{*/
	/**
     * \brief Provides basic log functionality.
     *
     * The Log class owns a number of LogWriters in a static map, which can be accessed
     * using a string identifier. All logs are global.
     *
     * By default the Log class contains a Debug, Info, Warning and Error log. These can be accessed
     * statically as:
     * \code
     * Log::debugLog() <<  "This is an debug message";
     * Log::infoLog() << "This is an info message";
     * Log::warnLog() << "This is an error message";
     * Log::errorLog() << "This is an error message";
     * \endcode
     * or on the log instance
     * \code
     * Log &log = Log::log();
     * log.debug() <<  "This is an debug message";
     * log.info() << "This is an info message";
     * log.warn() << "This is an error message";
     * log.error() << "This is an error message";
     * \endcode
     * or using one one the RW_LOG, RW_LOGLINE or RW_LOG2 macros, e.g.
     * \code
     * RW_LOG_INFO("The value of x is "<<x);
     * RW_LOG_DEBUG("The value of x is "<<x);
     * RW_LOG_ERROR(Log::infoId(), "The value of x is "<<x);
     * \endcode
     *
     * You can control what logs are active both using a loglevel and by using a log mask.
     * The loglevel enables all logs with LogIndex lower or equal to the loglevel. As default
     * loglevel is LogIndex::info which means debug and all user logs are disabled. However,
     * logs can be individually enabled using log masks which will override loglevel setting.
     *
     * Notice that logmasks cannot disable logs that are below or equal to loglevel.
     *
     * change loglevel:
     * \code
     * Log::log().setLevel(Log::Debug);
     * \endcode
     *
     *
     */
    class Log
    {
    public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<Log> Ptr;

        //! @brief loglevel mask
    	enum LogIndexMask {
    		FatalMask=1, CriticalMask=2,
    		ErrorMask=4, WarningMask=8,
    		InfoMask=16, DebugMask=32,
    		User1Mask=64, User2Mask=128,
    		User3Mask=256, User4Mask=512,
    		User5Mask=1024, User6Mask=2048,
    		User7Mask=4096, User8Mask=8096,
			AllMask = 0xFFFF
    	};



    	/**
    	 * @brief Indices for different logs. The loglevel will be Info as default. Everything below the
    	 * loglevel is enabled.
    	 *
    	 */
    	enum LogIndex {
    		Fatal=0, Critical=1,
    		Error=2, Warning=3,
    		Info=4, Debug=5,
    		User1=6, User2=7,
    		User3=8, User4=9,
    		User5=10, User6=11,
    		User7=12, User8=13
    	};

    	/**
    	 * @brief Convert a LogIndex to a mask.
    	 * @param idx [in] the LogIndex.
    	 * @return the mask enabling the given log level.
    	 */
        static LogIndexMask toMask(LogIndex idx){
                LogIndexMask toMaskArr[] = {FatalMask, CriticalMask,
                                          ErrorMask, WarningMask,
                                                    InfoMask, DebugMask,
                                                    User1Mask, User2Mask,
                                                    User3Mask, User4Mask,
                                                    User5Mask, User6Mask,
                                                    User7Mask, User8Mask,
                                                    AllMask};
                return toMaskArr[idx];
            }

    	/**
    	 * @brief convenience function for getting the LogWriter
    	 * that is associated with the info loglevel
    	 * @return info LogWriter
    	 */
        static LogWriter& infoLog() { 
			return Log::log().info();
		};

    	/**
    	 * @brief convenience function for getting the LogWriter
    	 * that is associated with the warning loglevel
    	 * @return warning LogWriter
    	 */
        static LogWriter& warningLog() { 
			return Log::log().warning();
		};


    	/**
    	 * @brief convenience function for getting the LogWriter
    	 * that is associated with the error loglevel
    	 * @return error LogWriter
    	 */
        static LogWriter& errorLog() { 
			return Log::log().error();
		};

    	/**
    	 * @brief convenience function for getting the LogWriter
    	 * that is associated with the debug loglevel
    	 * @return debug LogWriter
    	 */
        static LogWriter& debugLog() { 
			return Log::log().get(Debug);
		};

    	/**
    	 * @brief returns the global log instance. Global in the sence
    	 * of whatever is linked staticly together.
    	 * @return a Log
    	 */
        static Log::Ptr getInstance();

        /**
         * @brief convenience function of getInstance
         * @return a Log
         */
        static Log& log();

        /**
         * @brief sets the instance of the log class
         * @param log [in] the log that will be used through the static log methods.
         */
        static void setLog(Log::Ptr log);

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
         * @brief set the loglevel. Any log with LogIndex equal to or less than
         * loglevel will be enabled. Any log above will be disabled unless an
         * enabled mask is specified for that log
         * @param loglevel [in] the level
         */
        void setLevel(LogIndex loglevel){ _loglevel = loglevel; }


        /**
         * @brief gets the log writer associated to logindex \b id
         * @param id [in] logindex
         * @return log writer
         */
        LogWriter::Ptr getWriter(LogIndex id);

        /**
         * @brief Associates a LogWriter with the LogIndex \b id.
         *
         * SetWriter can either be used to redefine an existing log or to create a new
         * custom output.
         *
         * Example:
         * \code
         * Log::SetWriter(Log::User1, new LogStreamWriter(std::cout));
         * RW_LOG(Log::User1, "Message send to User log 1");
         * \endcode
         *
         * @param id [in] the LogIndex that the logwriter is associated with.
         * @param writer [in] LogWriter object to use
         */
        void setWriter(LogIndex id, LogWriter::Ptr writer);

        /**
         * @brief Associates a LogWriter with the logs specified with \b mask.
         *
         * SetWriter can either be used to redefine an existing log or to create a new
         * custom output.
         *
         * Example:
         * \code
         * log.setWriterForMask(Log::InfoMask | Log::DebugMask, new LogStreamWriter(std::cout));
         * RW_LOG(Log::Info, "Message send to User log 1");
         * \endcode
         *
         * @param mask [in] the LogIndexMask that the logwriter is associated with.
         * @param writer [in] LogWriter object to use
         */
		void setWriterForMask(int mask, LogWriter::Ptr writer);

        /**
         * @brief Returns the LogWriter that is associated with LogIndex \b id
         *
         * If the \b id is unknown an exception is thrown.
         *
         * @param id [in] loglevel
         * @return Reference to LogWriter object
         */
        LogWriter& get(LogIndex id);

        /**
         * @brief Writes \b message to the log
         *
         * If the \b id cannot be found an exception is thrown
         *
         * @param id [in] Log identifier
         * @param message [in] String message to write
         */
        void write(LogIndex id, const std::string& message);

        /**
         * @brief Writes \b message to the logwriter associated with LogIndex \b id
         *
         * If the \b id cannot be found an exception is thrown

         *
         * @param id [in] Log identifier
         * @param message [in] Message to write
         */
        void write(LogIndex id, const Message& message);

        /**
         * @brief Writes \b message followed by a '\\n' to the log
         *
         * If the \b id cannot be found an exception is thrown
         *
         * @param id [in] Log identifier
         * @param message [in] Message to write
         */
        void writeln(LogIndex id, const std::string& message);

        /**
         * @brief Calls flush on the specified log
         *
         * If the \b id cannot be found an exception is thrown
         *
         * @param id [in] loglevel
         */
        void flush(LogIndex id);


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
        void remove(LogIndex id);

		/**
		 * @brief Removes all log writers
		 */
		void removeAll();

		//! @brief Make indentation to make logs easier to read.
		void increaseTabLevel();

		//! @brief Decrease the indentation.
		void decreaseTabLevel();


    	/**
    	 * @brief convenience function for getting the LogWriter
    	 * that is associated with the info loglevel
    	 * @return info LogWriter
    	 */
        rw::common::LogWriter& info(){ 
			if (isLogEnabled(Info))
				return get(Info);
			else
				return *_defaultWriter;
		};

    	/**
    	 * @brief convenience function for getting the LogWriter
    	 * that is associated with the warning loglevel
    	 * @return info LogWriter
    	 */
        rw::common::LogWriter& warning(){ 
			if (isLogEnabled(Warning))
				return get(Warning);
			else
				return *_defaultWriter;
		};

    	/**
    	 * @brief convenience function for getting the LogWriter
    	 * that is associated with the error loglevel
    	 * @return info LogWriter
    	 */
        rw::common::LogWriter& error(){ 
			if (isLogEnabled(Error))
				return get(Error);
			else
				return *_defaultWriter;
		};

    	/**
    	 * @brief convenience function for getting the LogWriter
    	 * that is associated with the debug loglevel
    	 * @return info LogWriter
    	 */
        rw::common::LogWriter& debug(){ 
			if (isLogEnabled(Debug))
				return get(Debug);
			else
				return *_defaultWriter;
			};

        /**
         * @brief the loglevel is a runtime handle for enabling/disabling
         * logging to specific loglevels.
         * @param mask
		 *
		 * @note DEPRECATED. Use setEnable/setDisable instead
         */
        void setLogIndexMask( int mask ){ _logEnabledMask=mask; };


        /**
         * @brief get the current log mask
         * @return the LogIndex
		 * @note DEPRECATED. To be removed
		 */
        int getLogIndexMask() const{ return _logEnabledMask; };


		/**
		 * @brief Enable log(s) given by log mask.
		 * @param mask [in] the mask for the logs to enable.
		 */
		void setEnable(int mask) {
			_logEnabledMask = _logEnabledMask | mask;
		}

       /**
         * @brief Checks if the given LogIndex is enabled. This can be used to
         * determine if a certain log level will be displayed or not.
         * @param idx [in] the level
         */
        bool isEnabled(LogIndex idx) {
            if(idx<=_loglevel)
                return true;
            return (_logEnabledMask & toMask(idx) ) != 0;
        }

		/**
		 * @brief Disable log(s) given by log mask.
		 * @param mask [in] the mask for the logs to disable.
		 */
		void setDisable(int mask) {
			_logEnabledMask = _logEnabledMask & (_logEnabledMask ^ mask);
		}


    private:
    	bool isValidLogIndex(LogIndex id);

		bool isLogEnabled(LogIndexMask mask) {
			return (_logEnabledMask & mask) != 0;
		}

        bool isLogEnabled(LogIndex idx) {
            if(idx<=_loglevel)
                return true;
            return (_logEnabledMask & toMask(idx) ) != 0;
        }

		int _logEnabledMask;
		int _tabLevel;
		LogIndex _loglevel;
		std::vector<rw::common::LogWriter::Ptr> _writers;
		rw::common::LogWriter::Ptr _defaultWriter;


    };

    /*@}*/
}} // end namespaces
#endif /*LOG_HPP_*/
