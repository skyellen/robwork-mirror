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

#include <rw/common/macros.hpp>

#include <stdio.h>   /* Standard input/output definitions */
#include <iostream>
#include <io.h>      /* Low-level I/O definitions */
#include <fcntl.h>   /* File control definitions */
#include <windows.h> /* Windows standard function definitions */

#ifndef _WIN32
#include <termios.h> //Header needed to use B38400 etc. to setup baud rate
#endif

using namespace rwhw;

namespace {

std::wstring s2ws(const std::string& s)
{
	 int len;
	 int slength = (int)s.length() + 1;
	 len = MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, 0, 0);
	 wchar_t* buf = new wchar_t[len];
	 MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, buf, len);
	 std::wstring r(buf);
	 delete[] buf;
	 return r;
}
}

SerialPort::SerialPort()
{}

SerialPort::~SerialPort()
{}

bool SerialPort::open(
    const std::string& port,
    Baudrate baudrate,
    DataBits dbits,
    Parity parity, StopBits sbits)
{

	//cfd = CreateFile(TEXT("COM4"), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    cfd = CreateFile(s2ws(port).data(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

    if (cfd == INVALID_HANDLE_VALUE || cfd == NULL) {
    	RW_WARN("Tried to open WIN serial port but failed: INVALID_HANDLE_VALUE");
        return false;
    }

	//TODO: Why did we call this twice?
    close();

	cfd = CreateFile(s2ws(port).data(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	//cfd = CreateFile(TEXT("COM4"), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

    if (cfd == INVALID_HANDLE_VALUE || cfd == NULL) {
    	RW_WARN("Tried to open WIN serial port but failed: INVALID_HANDLE_VALUE");
        return false;
    }

    if (!SetupComm(cfd, 1024, 1024) != 0) {
    	RW_WARN("Tried to setup WIN serial port but failed!");
        return false;
    }

    DCB dcb;
    if (!GetCommState(cfd, &dcb)) {
    	RW_WARN("Could not get state of WIN serialport");
        return false;
    }
    switch(baudrate){
    case(Baud1200):   dcb.BaudRate = CBR_1200; break;
    case(Baud2400):   dcb.BaudRate = CBR_2400; break;
    case(Baud4800):   dcb.BaudRate = CBR_4800; break;
    case(Baud9600):   dcb.BaudRate = CBR_9600; break;
    case(Baud19200):  dcb.BaudRate = CBR_19200; break;
    case(Baud38400):  dcb.BaudRate = CBR_38400; break;
    case(Baud57600):  dcb.BaudRate = CBR_57600; break;
    case(Baud115200): dcb.BaudRate = CBR_115200; break;
    case(Baud230400):
    case(Baud460800):
    case(Baud921600):
    default:
        RW_WARN("Unsupported baudrate configuration!!");
        return false;
    }
    switch(sbits){
    case(Stop1_0): dcb.StopBits = ONESTOPBIT; break;
    case(Stop1_5): dcb.StopBits = ONE5STOPBITS; break;
    case(Stop2_0): dcb.StopBits = TWOSTOPBITS; break;
    default:
        RW_WARN("Unsupported StopBit configuration!!!");
        return false;
    }

    switch(parity){
    case(Even): dcb.Parity = EVENPARITY; break;
    case(Odd): dcb.Parity = ODDPARITY; break;
    case(None): dcb.Parity = NOPARITY; break;
    case(Mark): dcb.Parity = 3; break;
    case(Space): dcb.Parity = 4; break;
    default:
        RW_WARN("Unsupported parity configuration!!!");
        return false;
    }

    switch(dbits){
    case(Data5): dcb.ByteSize = 5; break;
    case(Data6): dcb.ByteSize = 6; break;
    case(Data7): dcb.ByteSize = 7; break;
    case(Data8): dcb.ByteSize = 8; break;
    default:
    	RW_WARN("Unsupported data bit size!");
        return false;
    }
    if (!SetCommState(cfd, &dcb)) {
    	RW_WARN("Tried to setup WIN serial port with desired baudrate but failed!");
        return false;
    }

    COMMTIMEOUTS comTimeOuts;
    GetCommTimeouts(cfd, &comTimeOuts);
    comTimeOuts.ReadTotalTimeoutMultiplier = 3;
    SetCommTimeouts(cfd, &comTimeOuts);

    clean();

    return true;
}

void SerialPort::close() {
    if(cfd==NULL)
        return;
    if(CloseHandle(cfd) == 0){    // Call this function to close port.
        RW_WARN("Port Closeing isn't successed.");
        return;
    }
}

bool SerialPort::write(const char* buf, int len) {
	debugPrint("write", buf, len);
    if(cfd==NULL)
        return false;
    DWORD length;
    DWORD bufsize = len;
    BOOL res = WriteFile(cfd, // handle to file to write to
                         (const void*) buf, // pointer to data to write to file
                         bufsize, // number of bytes to write
                         &length,NULL); // pointer to number of bytes written

    //FlushFileBuffers(cfd);
    //std::cout << "Writing: ";
    //for(int i=0;i<len;i++){
    //	unsigned char val = (unsigned char) buf[i];
    //	std::cout << std::hex << (int)val << " ";
    //}
    //std::cout << std::endl;
    if ( res== 0){
        RW_THROW("Writing of serial communication has problem." << GetLastError() );
        return false;
    }
    return true;
}

int SerialPort::read(char* buf, int len) {
    if(cfd==NULL)
        return 0;
    DWORD length;
    DWORD buffersize = len;
    BOOL res = ReadFile(
        cfd,  // handle of file to read
        (void *)buf, // handle of file to read
        buffersize,  // number of bytes to read
        &length,     // pointer to number of bytes read
        NULL);       // pointer to structure for data
    //std::cout << "SerielPort Read: " << length << std::endl;
    if( res == 0  ){
        RW_THROW("Reading of serial communication has problem. " << GetLastError());

        return length;
    }
	debugPrint("readfull", buf, len);

    return length;
}

void SerialPort::clean() {
    if(cfd==NULL)
        return;
    PurgeComm(cfd, PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR);
    //FlushFileBuffers(cfd);
}
