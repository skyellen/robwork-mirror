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


#include "Log.hpp"

//#include <rw/common/macros.hpp>

#include <rw/common/LogStreamWriter.hpp>
#include <rw/RobWork.hpp>
using namespace rw;
using namespace rw::common;


namespace {

	class EmptyLogWriter: public rw::common::LogWriter
	{
	public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<EmptyLogWriter> Ptr;

        virtual void flush(){};
        virtual void write(const std::string& str){};
        virtual void write(const rw::common::Message& msg){};
        virtual void writeln(const std::string& str){};
		virtual void setTabLevel(int) {};
	};
}


Log& Log::log(){
	return rw::RobWork::getInstance()->getLog();
}


void Log::setLog(Log::Ptr log){
    if(rw::RobWork::getInstance()->getLogPtr()==log)
        return;
    rw::RobWork::getInstance()->setLog(log);
}

Log::Ptr Log::getInstance(){
	return rw::RobWork::getInstance()->getLogPtr();
}

Log::Log():
	_logEnabledMask( 0 ), // enabled logs are completely defined by loglevel
	_tabLevel(0),
	_loglevel(Log::Info),
	_writers(32)
{
	_defaultWriter = ownedPtr(new EmptyLogWriter());
	setWriter(Debug, ownedPtr(new common::LogStreamWriter(&std::cout)) );
	setWriter(Info, ownedPtr(new common::LogStreamWriter(&std::cout)) );
    setWriter(Warning, ownedPtr(new common::LogStreamWriter(&std::cerr)) );
    setWriter(Error, ownedPtr(new common::LogStreamWriter(&std::cerr)) );
	setWriter(Critical, ownedPtr(new common::LogStreamWriter(&std::cerr)) );
	setWriter(Fatal, ownedPtr(new common::LogStreamWriter(&std::cerr)) );
	
}

Log::~Log() {

}

LogWriter::Ptr Log::getWriter(LogIndex id){
    return _writers[id];
}


void Log::setWriter(LogIndex id, rw::common::LogWriter::Ptr writer)
{
	writer->setTabLevel(_tabLevel);
	_writers[id] = writer;	
}

void Log::setWriterForMask(int mask, LogWriter::Ptr writer) 
{
	//The mask and log indices follows each other such that
	//index "i" has mask "2^i"
	int pattern = 1;
	size_t index = 0;
	do {
		if (mask & pattern)
			_writers[index] = writer;
		pattern = pattern << 1;
		index++;
	} while (index < _writers.size());

}

rw::common::LogWriter& Log::get(LogIndex id)
{
    if( !isLogEnabled(id) )
        return *_defaultWriter;
	if(isValidLogIndex(id))
		return *_writers[id];
	return *_defaultWriter;
	//RW_ASSERT("No such writer");
    //RW_THROW("LogWriter named: " << id << " does not exist");
}

void Log::write(LogIndex id, const std::string& message){
    get(id).write(message);
}

void Log::write(LogIndex id, const rw::common::Message& message){
    get(id).write(message);
}

void Log::writeln(LogIndex id, const std::string& message){
    get(id).write(message + '\n');
}

void Log::flush(LogIndex id){
    get(id).flush();
}

void Log::flushAll(){
    for(size_t i=0;i<_writers.size();i++){
    	if(_writers[i]!=NULL)
    		_writers[i]->flush();
    }
}


void Log::remove(LogIndex id)
{
	_writers[id] = NULL;
}

void Log::removeAll() {
	_writers.clear();
	_defaultWriter = ownedPtr(new EmptyLogWriter());
}

void Log::increaseTabLevel() {
	_tabLevel++;
    for(size_t i=0;i<_writers.size();i++){
    	if(_writers[i]!=NULL)
    		_writers[i]->setTabLevel(_tabLevel);
    }
}

void Log::decreaseTabLevel() {
	if (_tabLevel > 0)
		_tabLevel--;

    for(size_t i=0;i<_writers.size();i++){
    	if(_writers[i]!=NULL)
    		_writers[i]->setTabLevel(_tabLevel);
    }
}

bool Log::isValidLogIndex(LogIndex id){
	if(id<0 || _writers.size()<(size_t)id)
		return false;
	if(_writers[id]==NULL)
		return false;
	return true;
}
