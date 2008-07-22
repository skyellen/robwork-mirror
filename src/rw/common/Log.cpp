#include "Log.hpp"

#include <rw/common/macros.hpp>
#include <boost/foreach.hpp>

#include "LogStreamWriter.hpp"

using namespace rw::common;

Log::Map Log::_map;

const std::string& Log::warningId()
{
	static const std::string str = "Warning";
	return str;
}

const std::string& Log::errorId()
{
	static const std::string str = "Error";
	return str;
}

const std::string& Log::infoId()
{
	static const std::string info = "Info";
	return info;
}

const std::string& Log::debugId()
{
	static const std::string str = "Debug";
	return str;
}

namespace
{
    int init()
    {
        Log::setWriter(Log::infoId(), new LogStreamWriter(&std::cout));
        Log::setWriter(Log::debugId(), new LogStreamWriter(&std::cout));
        Log::setWriter(Log::warningId(), new LogStreamWriter(&std::cerr));
        Log::setWriter(Log::errorId(), new LogStreamWriter(&std::cerr));
        return 0;
    }

    const int x = init();
}

Log::Log() {}

Log::~Log() {}

void Log::setWriter(const std::string& id, LogWriter* writer)
{
    _map[id] = boost::shared_ptr<LogWriter>(writer);
}

LogWriter& Log::get(const std::string& id)
{
    Map::iterator it = _map.find(id);
    if (it != _map.end())
        return *it->second;

    RW_THROW("LogWriter named: " << id << " does not exist");
}

void Log::write(const std::string& id, const std::string& message)
{
    get(id).write(message);
}

void Log::write(const std::string& id, const Message& message)
{
    get(id).write(message);
}

void Log::writeln(const std::string& id, const std::string& message)
{
    get(id).write(message + '\n');
}

void Log::flush(const std::string& id)
{
    get(id).flush();
}

void Log::flushAll()
{
    typedef std::pair<std::string, boost::shared_ptr<LogWriter> > Pair;
    BOOST_FOREACH(const Pair& pair, _map) {
        pair.second->flush();
    }
}

void Log::remove(const std::string& id)
{
    _map.erase(id);
}
