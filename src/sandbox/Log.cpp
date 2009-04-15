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

#include "Log.hpp"

#include <rw/common/macros.hpp>
#include <boost/foreach.hpp>

#include <rw/common/LogStreamWriter.hpp>

using namespace rw::common::sandbox;
using namespace rw;

namespace {

	class EmptyLogWriter: public rw::common::LogWriter
	{
	public:
        virtual void flush(){};
        virtual void write(const std::string& str){};
        virtual void write(const rw::common::Message& msg){};
        virtual void writeln(const std::string& str){};
	};
	typedef rw::common::Ptr<EmptyLogWriter> EmptyLogWriterPtr;
}

Log _log;

Log& Log::log(){
	return getInstance();
}

Log& Log::getInstance(){
	return _log;
}

Log::Log():
	_writers(32)
{
	_defaultWriter = ownedPtr(new EmptyLogWriter());
    setWriter(Info, new common::LogStreamWriter(&std::cout));
    setWriter(Debug, new common::LogStreamWriter(&std::cout));
    setWriter(Warning, new common::LogStreamWriter(&std::cerr));
    setWriter(Error, new common::LogStreamWriter(&std::cerr));
}

Log::~Log() {}

void Log::setWriter(LogLevel id, rw::common::LogWriterPtr writer)
{
	_writers[id] = writer;
}

void Log::setWriter(const std::string& id, rw::common::LogWriterPtr writer)
{
	_userWriters[id] = writer;
}

rw::common::LogWriter& Log::get(LogLevel id)
{
	if(isValidLogLevel(id))
		return *_writers[id];
	return *_defaultWriter;
	//RW_ASSERT("No such writer");
    //RW_THROW("LogWriter named: " << id << " does not exist");
}

rw::common::LogWriter& Log::get(const std::string& id)
{
    Map::iterator it = _userWriters.find(id);
    if (it != _userWriters.end())
        return *it->second;

    return *_defaultWriter;
    //RW_THROW("LogWriter named: " << id << " does not exist");
}

void Log::write(LogLevel id, const std::string& message){
    get(id).write(message);
}

void Log::write(const std::string& id, const std::string& message){
    get(id).write(message);
}

void Log::write(LogLevel id, const rw::common::Message& message){
    get(id).write(message);
}

void Log::write(const std::string& id, const rw::common::Message& message){
    get(id).write(message);
}

void Log::writeln(LogLevel id, const std::string& message){
    get(id).write(message + '\n');
}

void Log::writeln(const std::string& id, const std::string& message){
    get(id).write(message + '\n');
}

void Log::flush(const std::string& id){
    get(id).flush();
}

void Log::flush(LogLevel id){
    get(id).flush();
}

void Log::flushAll(){
    typedef std::pair<std::string, Ptr<LogWriter> > Pair;
    BOOST_FOREACH(const Pair& pair, _userWriters) {
        pair.second->flush();
    }
    for(size_t i=0;i<_writers.size();i++){
    	if(_writers[i]!=NULL)
    		_writers[i]->flush();
    }
}

void Log::remove(const std::string& id)
{
    _userWriters.erase(id);
}

void Log::remove(LogLevel id)
{
	_writers[id] = NULL;
}

bool Log::isValidLogLevel(LogLevel id){
	if(id<0 || _writers.size()<id)
		return false;
	if(_writers[id]==NULL)
		return false;
	return true;
}
