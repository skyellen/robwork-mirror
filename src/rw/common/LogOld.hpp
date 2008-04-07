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

#ifndef rw_common_Log_HPP
#define rw_common_Log_HPP

/**
 * @file Log.hpp
 */

#include <string>
#include <boost/shared_ptr.hpp>

namespace rw { namespace common {

    /** @addtogroup common */
    /*@{*/

    class Message;

    /** @brief Log is the standard interface for receiving messages for the
     * user.
     *
     * The Log class is NOT Stable
     */
    class Log
    {
    public:
        /** @brief Receive and process a message for the user.
         *
         * Note: We use emitMessage() rather than just emit(), because of the
         * evil QT emit macro.
         *
         * @param message [in] The message to process.
         */
        virtual void emitMessage(const Message& message) = 0;

        virtual ~Log() {}

    private:
        Log(const Log&);
        Log& operator=(const Log&);

    protected:
        Log() {}
    };

    /** @brief RobWorkLog is a mutable container for a log object.
     *
     * RobWork has a number of globally accessible RobWorkLogs to which messages
     * for warnings, exceptions and other user messages are done.
     */
    class RobWorkLog
    {
    public:
        /** @brief Constructor
         *
         * By default, the RobWorkLog contains a log object that does nothing.
         * RobWork logs are given a name in general aid of debugging.
         *
         * @param name [in] The name to use for the log.
         */
        explicit RobWorkLog(const std::string& name);

        /** @brief The log object.
         *
         * @return The log object.
         */
        Log& get();

        /** @brief Set or reset the log object.
         *
         * The log must be non-null, or an exception will be thrown.
         *
         * @param log [in] The new log to use.
         */
        void set(boost::shared_ptr<Log> log);

    private:
        std::string _name;
        boost::shared_ptr<Log> _log;

    private:
        RobWorkLog(const RobWorkLog&);
        RobWorkLog& operator=(const RobWorkLog&);
    };

    /** @brief DEPRECATED The RobWorkLog to which warning messages are emitted.
     *
     * By default, messages are given a header and written to std::cerr.
     *
     * The method is deprecated
     *
     * @return The RobWorkLog to which warning messages are emitted.
     */
    RobWorkLog& warningLog();

    /** @brief DEPRECATED The RobWorkLog to which warning messages are emitted.
     *
     * The method is deprecated
     *
     * By default, messages are given a header and written to std::cerr.
     *
     * If your logger throws an exception, the effectively the assertion will
     * trigger an exception and instead of an abort(). You should however throw
     * an exception which is not of the type (or a subtype of) Exception,
     * because if you do that then you risk silently ignoring fatal errors.
     *
     * @return The RobWorkLog to which warning messages are emitted.
     */
    RobWorkLog& assertionLog();

    /** @brief DEPRECATED The RobWorkLog to which exception messages are emitted.
     *
     * The method is deprecated
     *
     * All messages for exceptions emitted by RobWork are forwarded to this
     * RobWork log. Your log object should do some or no processing of the
     * message and then return. RobWork _may_ depend on code to throw exceptions
     * of type Exception in case of error. Therefore your log object should not
     * itself (in normal operation) throw any exceptions.
     *
     * Be default, messages are given a header and written to std::cerr.
     *
     * @return The RobWorkLog to which exception messages are emitted.
     */
    RobWorkLog& exceptionLog();

    /*@}*/
}} // end namespaces

#endif // end include guard
