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

#if (defined _WIN32) || (defined __CYGWIN__)
#include "WinSerialPort.cpp"
#else
#include "LinuxSerialPort.cpp"
#endif

bool SerialPort::read(
    char* buf,
    const unsigned int n,
    const unsigned int timeout,
    const unsigned int sInterval)
{
    unsigned int index = 0;

    const unsigned long time =
        rw::common::TimerUtil::currentTimeMs() + timeout;

    unsigned long currTime = time;
    do {
        index += read( &(buf[index]), n-index );
        if(index >= n){
            return true;
        }
        rw::common::TimerUtil::sleepMs(sInterval);
        currTime = rw::common::TimerUtil::currentTimeMs();
    } while( currTime < time );
    return false;
}
