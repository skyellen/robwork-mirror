/*********************************************************************
 * RobWork Version 0.3
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

#include "TimerUtil.hpp"

#if !(defined __MINGW32__) && !(defined _WIN32)
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#endif
#include <iostream>
#ifdef _WIN32
#include <windows.h>
#include <time.h>
#endif

#include <cmath>

using namespace rw::common;

inline double round(double d) { return floor(d + 0.5); }

double TimerUtil::currentTime()
{
	return static_cast<double>(currentTimeUs()) / static_cast<double>(1e6);
}

void TimerUtil::sleepMs(int period)
{
#ifdef _WIN32
    Sleep(period);
#else
    usleep(1000 * period);
#endif
}

void TimerUtil::sleepUs(int period)
{
#ifdef _WIN32
    // This is the best we can do, I think.
    Sleep(static_cast<int>(ceil(1e-3 * period)));
#else
    usleep(period);
#endif
}

// We just forward to CurrentTime() here: All the time is spent in the context
// switch of the system call anyway so there is no loss of performance.
// NOTE: (JIMMY) this did not work on some linux computers...

long TimerUtil::currentTimeMs()
{
    /*
    THIS DID NOT WORK CORRECTLY. */
#ifdef _WIN32
    return (long) (clock()* (double(1e3)/CLOCKS_PER_SEC));
#else
    //struct timespec time;
    //clock_gettime(CLOCK_REALTIME, &time);
    //return ((time.tv_nsec/1e6 + time.tv_sec * 1e3));

    timeval current;
    gettimeofday(&current, 0);
    //std::cout << current.tv_sec*1e3 << std::endl;
    //std::cout << ((double)current.tv_usec)/1000.0 << std::endl;
    //std::cout << current.tv_sec*1e3 + ((double)current.tv_usec)/1000.0 << std::endl;
    //return ((time.tv_nsec/1e6 + time.tv_sec * 1e3));
    return current.tv_sec*1e3 + current.tv_usec/1000;

#endif
}

long TimerUtil::currentTimeUs()
{
    return (long) (clock()* (double(1e6)/CLOCKS_PER_SEC));
}
