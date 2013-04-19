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

void BINArchive::open(const std::string& filename){
	if( !boost::filesystem::exists(filename.c_str()) ){
		//create the file

	}

	_fstr = new std::fstream(filename.c_str());
	_iostr = _fstr;
	_ofs = _fstr;
	_ifs = _fstr;
	_isopen =  _fstr->is_open();
}

void BINArchive::open(std::iostream& stream){
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

void BINArchive::read(bool& val, const std::string& id){
	int res = readUInt8(id);
	if(res==0)
		val = false;
	else
		val = true;
 }

 void BINArchive::read(std::string& val, const std::string& id){
	 _ifs->getline(_line,500);
	 std::pair<std::string,std::string> valname = getNameValue();
	 //std::cout << valname.first << "  " << valname.second << std::endl;
	 val = valname.second;
 }

 void BINArchive::read(std::vector<std::string>& val, const std::string& id){
		_ifs->getline(_line,500);
		std::pair<std::string,std::string> valname = getNameValue();
		if(id!=valname.first)
			RW_WARN("mismatched ids: " << id << " ---- " << valname.first);
	    // read from array
		boost::split(val, valname.second, boost::is_any_of("\t "));
 }
