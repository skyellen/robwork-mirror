/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
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

namespace
{
    HANDLE cfd;
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
    //std::cout<<"Serial Port Open WIN"<<std::endl;
    cfd = CreateFile(port.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL,
                     OPEN_EXISTING, 0, NULL);

    if (cfd == INVALID_HANDLE_VALUE || cfd == NULL) {
    	RW_WARN("Tried to open WIN serial port but failed: INVALID_HANDLE_VALUE");
        return false;
    }
    close();

    cfd = CreateFile(port.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL,
                     OPEN_EXISTING, 0, NULL);

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
    return length;
}

void SerialPort::clean() {
    if(cfd==NULL)
        return;
    PurgeComm(cfd, PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR);
    //FlushFileBuffers(cfd);
}
