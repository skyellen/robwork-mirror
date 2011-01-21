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

#include "SerialPort.hpp"

#include <rw/common/os.hpp>
#include <rw/common/macros.hpp>

#include <sys/types.h>
#include <sys/stat.h>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include <iostream>
#include <string>

using namespace rwhw;


/**
 * TODO: Make sure the calls to read/write is non-blocking as specified by the interface
 */


// This implies that we can have only one serial port at the time. SerialPort
// should probably either be an abstract class or if it is not then it should
// use the pimpl idiom so that the Linux and Win versions can be implemented
// without the below issue.
namespace
{
    int _ttyS;
    struct termios _oldtio;
    struct termios _newtio;
}

SerialPort::SerialPort()
{
    _ttyS = -1;
}

SerialPort::~SerialPort()
{
    std::cout<<"SerialPort Destructor " << _ttyS<<std::endl;
    if (_ttyS != -1)
        ::close(_ttyS);
}

bool SerialPort::open(
    const std::string& port,
    Baudrate brate,
    DataBits dbits,
    Parity parity,
    StopBits sbits)
{
    std::cout<<"Serial Port Open "<<std::endl;

    // set the user console port up
    _ttyS = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK ); 
    tcgetattr(_ttyS, &_oldtio); // save current port settings

    memset(&_newtio, 0, sizeof(_newtio));
    _newtio.c_cflag |= CLOCAL;
    _newtio.c_cflag |= CREAD;

    speed_t baudrate;
    switch(brate){
    case(Baud1200):   baudrate = B1200; break;
    case(Baud2400):   baudrate = B2400; break;
    case(Baud4800):   baudrate = B4800; break;
    case(Baud9600):   baudrate = B9600; break;
    case(Baud19200):  baudrate = B19200; break; 
    case(Baud38400):  baudrate = B38400; break;
    case(Baud57600):  baudrate = B57600; break;
    case(Baud115200): baudrate = B115200; break;
    case(Baud230400): baudrate = B230400; break;
#ifndef RW_MACOS
    case(Baud460800): baudrate = B460800; break;
    case(Baud921600): baudrate = B921600; break;
#endif
    default:
        RW_WARN("Unsupported baudrate configuration!!");
        return false;
    }
    // baudrate
    cfsetispeed(&_newtio, baudrate);
    cfsetospeed(&_newtio, baudrate);
    _newtio.c_cflag &= ~CRTSCTS; // flow control

    switch(parity){
    case(Even): _newtio.c_cflag |= PARENB; _newtio.c_cflag &= ~PARODD; break;
    case(Odd):  _newtio.c_cflag |= PARENB; _newtio.c_cflag |= PARODD; break;
    case(None): _newtio.c_cflag &= ~PARENB;; break;
    case(Mark):
    case(Space):
    default:
        RW_WARN("Unsupported parity configuration!!!");
        return false;
    }

    switch(sbits){
    case(Stop1_0): _newtio.c_cflag &= ~CSTOPB; break;
    case(Stop2_0): _newtio.c_cflag |= CSTOPB; break;
    case(Stop1_5):;
    default:
        RW_WARN("Unsupported StopBit configuration!!!");
        return false;
    }
    _newtio.c_cflag &= ~CSIZE;

    switch(dbits){
    case(Data5): _newtio.c_cflag |= CS5; break;
    case(Data6): _newtio.c_cflag |= CS6; break;
    case(Data7): _newtio.c_cflag |= CS7; break;
    case(Data8): _newtio.c_cflag |= CS8; break;
    default:
        return false;
    }

    _newtio.c_iflag = IGNPAR; // ignore read bytes with parity error
    _newtio.c_oflag = 0; // no output processing
    _newtio.c_lflag = 0;
    _newtio.c_cc[VMIN] = 1;
    _newtio.c_cc[VTIME] = 0;

    tcflush(_ttyS, TCIFLUSH); // flush stream
    tcsetattr(_ttyS, TCSANOW, &_newtio); // set attributes
    return true;
}

void SerialPort::close()
{
    ::close(_ttyS);
    _ttyS = -1;
}

bool SerialPort::write(const char* buf, int n)
{
	debugPrint("write", buf, n);
    if (::write(_ttyS, buf, n) == -1) {
        return false;
    }
    return true;
}

int SerialPort::read(char* buf, int n)
{
    int b = 0;
    for (int i = 0; i<n; i++) {
        b = ::read(_ttyS, &buf[i], 1);
        if (b<=0) {
//        	printf ("%x", buf);
//        	debugPrint("readhalf", buf, i);
            return i;
        }
    }
    debugPrint("readfull", buf, n);
    return n;
}

void SerialPort::clean()
{
    char ch;
    while (0<(::read(_ttyS, &ch, 1)));
    debugPrint("clean", &ch, 1);
}
