#include "Log.hpp"

#include <rw/common/macros.hpp>
#include <boost/foreach.hpp>

#include "LogStreamWriter.hpp"

using namespace rw::common;


const std::string Log::Info("Info");
const std::string Log::Warning("Warning");
const std::string Log::Error("Error");
Log::Map Log::_map;

namespace {
    static int init() {
        Log::setWriter(Log::Info, new LogStreamWriter(std::cout));
        Log::setWriter(Log::Warning, new LogStreamWriter(std::cerr));
        Log::setWriter(Log::Error, new LogStreamWriter(std::cerr));
        return 0;
    }
    static int x = init();
    
}

Log::Log()
{
}

Log::~Log()
{
}


void Log::setWriter(const std::string& id, LogWriter* writer) {
    _map[id] = boost::shared_ptr<LogWriter>(writer);
}

LogWriter& Log::get(const std::string& id) {
    Map::iterator it = _map.find(id);
    if (it != _map.end())
        return *(*it).second;
    RW_THROW("LogWriter named: "<<id<<" does not exist");
}

void Log::write(const std::string& id, const std::string& message) {
    get(id).write(message);
}

void Log::write(const std::string& id, const Message& message) {
    get(id).write(message);
}


void Log::writeln(const std::string& id, const std::string& message) {
    get(id).write(message + '\n');
}



void Log::flush(const std::string& id) {
    get(id).flush();
}

void Log::flushAll() {
    typedef std::pair<std::string, boost::shared_ptr<LogWriter> > PAIR;
    BOOST_FOREACH(PAIR pair, _map) {
        pair.second->flush();
    }
}

void Log::remove(const std::string& id) {
    _map.erase(id);
}


