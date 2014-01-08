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

#include "BINArchive.hpp"

#include <boost/filesystem.hpp>

using namespace rw::common;

const int BINArchive::MAX_LINE_WIDTH;

BINArchive::~BINArchive(){
	close();
}

void BINArchive::close(){
	if(_fstr!=NULL)
		_fstr->close();
}

void BINArchive::doWriteEnterScope(const std::string& id){
	_scope.push_back(id);
	(*_ofs) << "[" << getScope() << "]\n";
}

void BINArchive::doWriteLeaveScope(const std::string& id){
	if(id!=_scope.back()){
		RW_THROW("Scopes has been messed up!");
	}
	_scope.pop_back();
}

// utils to handle arrays
void BINArchive::doReadEnterScope(const std::string& id){
	_scope.push_back(id);
	_ifs->getline(_line, MAX_LINE_WIDTH);
	//(*_ofs) << "[" << getScope() << "]\n";
}

void BINArchive::doReadLeaveScope(const std::string& id){
	if(id!=_scope.back()){
		RW_THROW("Scopes has been messed up!");
	}
	_scope.pop_back();
}


void BINArchive::doOpenArchive(const std::string& filename){
	if( !boost::filesystem::exists(filename.c_str()) ){
		//create the file

	}

	_fstr = new std::fstream(filename.c_str());
	_iostr = _fstr;
	_ofs = _fstr;
	_ifs = _fstr;
	_isopen =  _fstr->is_open();
}

void BINArchive::doOpenArchive(std::iostream& stream){
	_fstr = NULL;
	_iostr = &stream;
	_ofs = _iostr;
	_ifs = _iostr;
	_isopen =  true;
}

void BINArchive::flush(){
	if(_iostr!=NULL)
		_iostr->flush();
	if(_ofs!=NULL && _ofs!=_iostr)
		_ofs->flush();
}

void BINArchive::doRead(bool& val, const std::string& id){
	int res = readUInt8(id);
	if(res==0)
		val = false;
	else
		val = true;
 }

 void BINArchive::doRead(std::string& val, const std::string& id){
	 _ifs->getline(_line, MAX_LINE_WIDTH);
	 std::pair<std::string,std::string> valname = getNameValue();
	 //std::cout << valname.first << "  " << valname.second << std::endl;
	 val = valname.second;
 }

 void BINArchive::doRead(std::vector<std::string>& val, const std::string& id){
		_ifs->getline(_line,MAX_LINE_WIDTH);
		std::pair<std::string,std::string> valname = getNameValue();
		if(id!=valname.first)
			RW_WARN("mismatched ids: " << id << " ---- " << valname.first);
	    // read from array
		boost::split(val, valname.second, boost::is_any_of("\t "));
 }


