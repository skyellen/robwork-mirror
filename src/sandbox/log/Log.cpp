#include "Log.hpp"

#include <rw/common/macros.hpp>
#include <boost/foreach.hpp>

#include "LogStreamWriter.hpp"

using namespace rw::sandbox;


const std::string Log::Info("Info");
const std::string Log::Warning("Warning");
const std::string Log::Error("Error");
Log::Map Log::_map;

namespace {
    static int init() {
        Log::SetWriter(Log::Info, new LogStreamWriter(std::cout));
        Log::SetWriter(Log::Warning, new LogStreamWriter(std::cerr));
        Log::SetWriter(Log::Error, new LogStreamWriter(std::cerr));
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


void Log::SetWriter(const std::string& id, LogWriter* writer) {
    _map[id] = boost::shared_ptr<LogWriter>(writer);
}

LogWriter& Log::Get(const std::string& id) {
    Map::iterator it = _map.find(id);
    if (it != _map.end())
        return *(*it).second;
    RW_THROW("LogWriter named: "<<id<<" does not exist");
}

void Log::Write(const std::string& id, const std::string& message) {
    Get(id).write(message);
}

void Log::WriteLine(const std::string& id, const std::string& message) {
    Get(id).write(message + '\n');
}



void Log::Flush(const std::string& id) {
    Get(id).flush();
}

void Log::FlushAll() {
    typedef std::pair<std::string, boost::shared_ptr<LogWriter> > PAIR;
    BOOST_FOREACH(PAIR pair, _map) {
        pair.second->flush();
    }
}

void Log::Remove(const std::string& id) {
    _map.erase(id);
}


