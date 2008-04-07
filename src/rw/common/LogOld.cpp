/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
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

#include "Log.hpp"
#include "Message.hpp"
#include "macros.hpp"
#include "StringUtil.hpp"

#define NS rw::common
using namespace NS;

namespace
{
    // A log that does nothing.
    class NoLog : public Log
    {
    public:
        void emitMessage(const Message& message) {}
    };

    // The type of log used for warnings.
    class WarningLog : public Log
    {
    public:
        void emitMessage(const Message& message)
        {
            std::cerr
                << "Warning ("
                << message.getFile()
                << ":"
                << message.getLine()
                << "):\n"
                << message.getText()
                << "\n";
        }
    };

    // The type of log used for warnings.
    class AssertionLog : public Log
    {
    public:
        void emitMessage(const Message& message)
        {
            std::cerr
                << "Assertion ("
                << message.getFile()
                << ":"
                << message.getLine()
                << "):\n"
                << message.getText()
                << "\n";
        }
    };

    // The type of log used for exceptions.
    class ExceptionLog : public Log
    {
    public:
        void emitMessage(const Message& message)
        {
            std::cerr
                << "Exception ("
                << message.getFile()
                << ":"
                << message.getLine()
                << "):\n"
                << message.getText()
                << "\n";
        }
    };
}

RobWorkLog::RobWorkLog(const std::string& name) :
    _name(name),
    _log(boost::shared_ptr<Log>(new NoLog()))
{}

Log& RobWorkLog::get()
{
    return *_log;
}

void RobWorkLog::set(boost::shared_ptr<Log> log)
{
    if (!log.get())
        RW_THROW("Can't assign null-log for RobWork log "
         << StringUtil::Quote(_name));

    _log = log;
}

RobWorkLog& NS::warningLog()
{
    static RobWorkLog log("warningLog");
    return log;
}

RobWorkLog& NS::assertionLog()
{
    static RobWorkLog log("assertionLog");
    return log;
}

RobWorkLog& NS::exceptionLog()
{
    static RobWorkLog log("exceptionLog");
    return log;
}

//----------------------------------------------------------------------
// Log initialization

namespace
{
    int initializeLogs()
    {
        warningLog().set(boost::shared_ptr<Log>(new WarningLog()));
        assertionLog().set(boost::shared_ptr<Log>(new AssertionLog()));
        exceptionLog().set(boost::shared_ptr<Log>(new ExceptionLog()));
        return 0;
    }

    int x = initializeLogs();    
}
