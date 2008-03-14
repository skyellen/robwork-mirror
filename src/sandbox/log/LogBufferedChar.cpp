#include "LogBufferedChar.hpp"
#include <iostream>

using namespace rw::sandbox;

LogBufferedChar::LogBufferedChar(size_t size, std::ostream& stream, OverflowPolicy policy):
    _stream(stream),
    _size(size),
    _policy(policy)
{
    _buffer = new char[_size];
    _index = 0;
    _overflow = false;

}

LogBufferedChar::~LogBufferedChar()
{
    flush();
    delete _buffer;
}

void LogBufferedChar::write(const std::string& str) {

    //If a single message is large than the entire buffer we skip the last part of the message
    size_t cnt = std::min(str.size(), _size);

    
    if (_index + cnt > _size) {
        switch (_policy) {
        case REMOVE_FIRST: {
            int cnt1 = _size - _index;
            int cnt2 = str.size() - cnt1;
            memcpy(&(_buffer[_index]), str.c_str(), cnt1);
            _index = 0;
            memcpy(&(_buffer[_index]), &(str.c_str()[cnt1]), cnt2);
            _index = cnt2;
            _overflow = true;
            break;
        }
        case REMOVE_LAST:
            memcpy(&(_buffer[_index]), str.c_str(), _size-_index);
            _index = _size;
            break;
        case AUTO_FLUSH:
            flush();
            _stream<<str<<std::endl;
            _index = 0;
            break;
        }
    } else {
        memcpy(&(_buffer[_index]), str.c_str(), cnt);
        _index += cnt;
        std::cout<<"str = "<<str<<std::endl;
        _buffer[_index] = 0;
        std::cout<<"buffer = "<<_buffer<<std::endl;
        

    }
    
    
}

void LogBufferedChar::flush() {
    std::cout<<"Buffer=";
    std::cout.write(_buffer, _size);
    std::cout<<std::endl;
    std::cout<<"Index = "<<_index<<std::endl;
    if (_overflow) {
        _stream.write(&(_buffer[_index]), _size - _index);
        std::cout<<"Writes 1:";
        std::cout.write(&(_buffer[_index]), _size - _index);
        std::cout<<std::endl;
        _stream.write(_buffer, _index);
        std::cout<<"Writes 2:";
        std::cout.write(_buffer, _index);
        std::cout<<std::endl;
    } else {
        _stream.write(_buffer, _index);
        std::cout<<"Writes 0:";
        std::cout.write(_buffer, _index);
        std::cout<<std::endl;
    }
    _index = 0;
    _buffer[_index] = 0;
    _overflow = false;
    _stream.flush();    
}
