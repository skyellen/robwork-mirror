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

#include "INIArchive.hpp"

#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
using namespace rw::common;


void INIArchive::close()
{
    //flush();
    if (_fstr != NULL) {
    	_fstr->close();
    	delete _fstr;
    	_fstr = NULL;
    }
}

void INIArchive::doWriteEnterScope(const std::string& id)
{
    _scope.push_back(id);
    (*_ofs) << "[" << getScope() << "]\n";
}

void INIArchive::doWriteLeaveScope(const std::string& id)
{
    if (id != _scope.back()) {
        RW_THROW("Scopes has been messed up! Expected id " << _scope.back() << " got " << id);
    }
    _scope.pop_back();
}

void INIArchive::doReadEnterScope(const std::string& id)
{
    _scope.push_back(id);
    _ifs->getline(_line, MAX_LINE_WIDTH);
    //(*_ofs) << "[" << getScope() << "]\n";
}

void INIArchive::doReadLeaveScope(const std::string& id)
{
    if (id != _scope.back()) {
        RW_THROW("Scopes has been messed up!");
    }
    _scope.pop_back();
}

void INIArchive::doOpenArchive(const std::string& filename)
{
    if( !boost::filesystem::exists(filename) ) {
        _fstr = new std::fstream(filename.c_str(), std::ios::out | std::ios::in | std::ios::trunc );
    } else {
        _fstr = new std::fstream(filename.c_str(), std::ios::out | std::ios::in);
    }

    _iostr = _fstr;
    _ofs = _fstr;
    _ifs = _fstr;
    _isopen = _fstr->is_open();
}

void INIArchive::doOpenArchive(std::iostream& stream)
{
    if (_fstr != NULL) {
    	_fstr->close();
    	delete _fstr;
    	_fstr = NULL;
    }
    _iostr = &stream;
    _ofs = _iostr;
    _ifs = _iostr;
    _isopen = true;
}

void INIArchive::doOpenOutput(std::ostream& ofs){
    if (_fstr != NULL) {
    	_fstr->close();
    	delete _fstr;
    	_fstr = NULL;
    }
	_iostr = NULL;
	_ofs = &ofs;
	_isopen = true;
}

void INIArchive::doOpenInput(std::istream& ifs){
    if (_fstr != NULL) {
    	_fstr->close();
    	delete _fstr;
    	_fstr = NULL;
    }
	_iostr = NULL;
	_ifs = &ifs;
	_isopen = true;
}

void INIArchive::flush()
{
    if (_iostr != NULL) _iostr->flush();
    if (_ofs != NULL && _ofs != _iostr) _ofs->flush();
}

void INIArchive::doWrite(boost::int8_t val, const std::string& id)
{
    //std::cout << "uint stuff b" << (int)val << std::endl;
    //int bum = 4;
    (*_ofs) << id << "=" << (int) val << "\n";
    //std::cout << id << "=" << (int)val << "\n";

    //boost::int32_t tmp=val;std::cout << "uint stuff" << std::endl; writeValue<int>(tmp,id);
}

void INIArchive::doWrite(boost::uint8_t val, const std::string& id)
{
    (*_ofs) << id << "=" << (int) val << "\n";
    //doWrite((boost::uint32_t)val,id);
}

void INIArchive::doRead(bool& val, const std::string& id)
{
    int res = readInt(id);
    if (res == 0) val = false;
    else val = true;
}

void INIArchive::doRead(boost::int8_t& val, const std::string& id)
{
	//8 bits are handled as 16 bit to avoid confusion with char
    boost::int16_t tmp;
    readValue<boost::int16_t>(tmp, id);
    val = static_cast<boost::int8_t>(tmp);
}

void INIArchive::doRead(boost::uint8_t& val, const std::string& id)
{
	//8 bits are handled as 16 bit to avoid confusion with char
    boost::uint16_t tmp;
    readValue<boost::uint16_t>(tmp, id);
    val = static_cast<boost::uint8_t>(tmp);
}

void INIArchive::doRead(std::string& val, const std::string& id)
{
    getLine();
    std::pair<std::string, std::string> valname = getNameValue();
    //std::cout << valname.first << "  " << valname.second << std::endl;
    if (valname.first != id) {
        RW_WARN("Mismatching id when reading INI file. " << valname.first << "!=" << id);
    }
    val = valname.second;
}

void INIArchive::doRead(std::vector<std::string>& val, const std::string& id)
{
    getLine();
    std::pair<std::string, std::string> valname = getNameValue();
    if (valname.first != id) {
        RW_WARN("Mismatching id when reading INI file. " << valname.first << "!=" << id);
    }
    // read from array
    boost::split(val, valname.second, boost::is_any_of("\t "));
}

bool INIArchive::getLine()
{
    if( !_isopen )
        return false;
    bool valid = false;
    while (valid == false) {
        _ifs->getline(_line, MAX_LINE_WIDTH);
        //std::cout << "Line: " << _line << std::endl;
        if (_ifs->eof()) break;
        // test if line has valid input
        static const boost::regex comment("^[ ]*;[.]*$");
        static const boost::regex empty("^[\\h]*$");
        if (boost::regex_match(_line, comment)) continue;
        if (boost::regex_match(_line, empty)) continue;
        valid = true;
    }
    return valid;
}
