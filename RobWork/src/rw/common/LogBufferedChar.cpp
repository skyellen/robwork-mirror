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

 
#include "LogBufferedChar.hpp"
#include <iomanip>
#include <cstring>
 
using namespace rw::common;

LogBufferedChar::LogBufferedChar(size_t size, std::ostream* stream, OverflowPolicy policy):
    _stream(stream),
    _buffer(size),
    _size(size),
    _policy(policy),
	_tabLevel(0)
{
    _buffer[0] = 0;

    _index = 0;
    _overflow = false;

}

LogBufferedChar::~LogBufferedChar()
{
    flush();
}

void LogBufferedChar::doWrite(const std::string& input) {
	std::stringstream sstr;
	sstr << std::setw(_tabLevel)<<std::setfill(' ');
	sstr<<input;

	std::string str = sstr.str();

    //If a single message is large than the entire buffer we skip the last part of the message
    size_t cnt = std::min(str.size(), _size);

    if (_index + cnt >= _size) {
        switch (_policy) {
        case REMOVE_FIRST: {
            int cnt1 = (int)_size - _index; // 6
            int cnt2 = (int)cnt - cnt1; // 0
            memcpy(get(_index), str.c_str(), cnt1);
            _index = 0;
            if(cnt2>0){
            	memcpy(get(_index), &str.c_str()[cnt1], cnt2);
            	_index = cnt2;
            }
            _overflow = true;
            break;
        }
        case REMOVE_LAST:
            memcpy(get(_index), str.c_str(), _size - _index);
            _index = (int)_size;
            break;
        case AUTO_FLUSH:
            flush();
            (*_stream)<<str<<std::endl;
            _index = 0;
            break;
        }
    } else {
        memcpy(get(_index), str.c_str(), cnt);
       	_index += (int)cnt;

        _buffer[_index] = 0;
    }
}

void LogBufferedChar::doFlush() {
    if (_overflow) {
    	_stream->write(get(_index), _size - _index);
        if(_index>0){
        	_stream->write(get(0), _index);
        }
    } else {
        //_stream->write(get(0), _index);
    }

    _index = 0;
    _buffer[_index] = 0;
    _overflow = false;
    _stream->flush();
}


void LogBufferedChar::doSetTabLevel(int tabLevel) {
	_tabLevel = tabLevel;
}
