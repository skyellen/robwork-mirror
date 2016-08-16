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

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include <fstream>

using namespace rw::common;


BINArchive::~BINArchive(){
	close();
}

void BINArchive::close(){
	if(_fstr != NULL) {
		_fstr->close();
		delete _fstr;
		_fstr = NULL;
	}
}

void BINArchive::doWriteEnterScope(const std::string& id){
	_scope.push_back(id);
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
}

void BINArchive::doReadLeaveScope(const std::string& id){
	if(id!=_scope.back()){
		RW_THROW("Scopes has been messed up!");
	}
	_scope.pop_back();
}


void BINArchive::doOpenArchive(const std::string& filename){
    if( !boost::filesystem::exists(filename) ) {
        _fstr = new std::fstream(filename.c_str(), std::ios::out | std::ios::in | std::ios::trunc | std::ios::binary);
    } else {
        _fstr = new std::fstream(filename.c_str(), std::ios::out | std::ios::in | std::ios::binary);
    }

	_iostr = _fstr;
	_ofs = _fstr;
	_ifs = _fstr;
	_isopen =  _fstr->is_open();
}

void BINArchive::doOpenArchive(std::iostream& stream){
	if(_fstr != NULL) {
		_fstr->close();
		delete _fstr;
		_fstr = NULL;
	}
	_iostr = &stream;
	_ofs = _iostr;
	_ifs = _iostr;
	_isopen =  true;
}

void BINArchive::doOpenOutput(std::ostream& ofs){
	if(_fstr != NULL) {
		_fstr->close();
		delete _fstr;
		_fstr = NULL;
	}
	_iostr = NULL;
	_ofs = &ofs;
	_isopen = true;
}

void BINArchive::doOpenInput(std::istream& ifs){
	if(_fstr != NULL) {
		_fstr->close();
		delete _fstr;
		_fstr = NULL;
	}
	_iostr = NULL;
	_ifs = &ifs;
	_isopen = true;
}

void BINArchive::flush(){
	if(_iostr!=NULL)
		_iostr->flush();
	if(_ofs!=NULL && _ofs!=_iostr)
		_ofs->flush();
}

void BINArchive::doRead(std::vector<bool>& val, const std::string& id){
    boost::uint32_t s = 0;
    _ifs->read((char*)&s, sizeof(boost::uint32_t) );
    //std::cout << "LEN:" << s << std::endl;
    val.resize(s);
    for( boost::uint32_t i=0; i<s;i++){
        uint8_t tmp;
        _ifs->read((char*)& (tmp), sizeof(uint8_t) );
        val[i] = tmp;
    }
}

void BINArchive::doRead(bool& val, const std::string& id){
	int res = readUInt8(id);
	if(res==0)
		val = false;
	else
		val = true;
 }

void BINArchive::doWrite(const std::string& val, const std::string& id){
    boost::uint32_t s = val.size();
     (*_ofs) << s;
     BOOST_FOREACH(const char& rval, val){ (*_ofs) << rval; }
}

void BINArchive::doWrite(const std::vector<std::string>& val, const std::string& id){
    boost::uint32_t s = val.size();
     (*_ofs) << s;
     BOOST_FOREACH(const std::string& str_tmp, val){
         boost::uint32_t str_s = str_tmp.size();
          (*_ofs) << str_s;
          BOOST_FOREACH(const char& rval, str_tmp){ (*_ofs) << rval; }
     }
}

 void BINArchive::doRead(std::string& val, const std::string& id){
     boost::uint32_t s;
     (*_ifs) >> s;
     val.resize(s);
     for( boost::uint32_t i=0; i<s;i++){
         (*_ifs) >> val[i];
     }
 }

 void BINArchive::doRead(std::vector<std::string>& val, const std::string& id){
     boost::uint32_t s,ss;
     (*_ifs) >> s;
     val.resize(s);
     for( boost::uint32_t i=0; i<s;i++){
         (*_ifs) >> ss;
         std::string str_val;
         str_val.resize(ss);
         for( boost::uint32_t j=0; j<ss;j++){
             (*_ifs) >> str_val[j];
         }
     }
 }

 std::string BINArchive::getScope(){
	 if(_scope.size()==0)
		 return "";
	 std::stringstream sstr;
	 for(size_t i=0;i<_scope.size()-1;i++)
		 sstr << _scope[i] << ".";
	 sstr << _scope.back();
	 return sstr.str();
 }
