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

#include "LogBufferedChar.hpp"
#include <iostream>

using namespace rw::common;

LogBufferedChar::LogBufferedChar(size_t size, std::ostream* stream, OverflowPolicy policy):
    _stream(stream),
    _buffer(size),
    _size(size),
    _policy(policy)
{
    _buffer[0] = 0;

    _index = 0;
    _overflow = false;

}

LogBufferedChar::~LogBufferedChar()
{
    flush();
}

void LogBufferedChar::write(const std::string& str) {

    //If a single message is large than the entire buffer we skip the last part of the message
    size_t cnt = std::min(str.size(), _size);

    if (_index + cnt >= _size) {
        switch (_policy) {
        case REMOVE_FIRST: {
            int cnt1 = _size - _index;
            int cnt2 = str.size() - cnt1;
            memcpy(get(_index), str.c_str(), cnt1);
            _index = 0;
            memcpy(get(_index), &str.c_str()[cnt1], cnt2);
            _index = cnt2;
            _overflow = true;
            break;
        }
        case REMOVE_LAST:
            memcpy(get(_index), str.c_str(), _size - _index);
            _index = _size;
            break;
        case AUTO_FLUSH:
            flush();
            (*_stream)<<str<<std::endl;
            _index = 0;
            break;
        }
    } else {
        memcpy(get(_index), str.c_str(), cnt);
        _index += cnt;
        _buffer[_index] = 0;
    }
}

void LogBufferedChar::flush() {
    if (_overflow) {
        _stream->write(get(_index), _size - _index);
        _stream->write(get(0), _index);
    } else {
        _stream->write(get(0), _index);
    }

    _index = 0;
    _buffer[_index] = 0;
    _overflow = false;
    _stream->flush();
}
