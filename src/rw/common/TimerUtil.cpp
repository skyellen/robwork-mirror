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

#include "TimerUtil.hpp"

#if !(defined __MINGW32__) && !(defined _WIN32)
#include <sys/time.h>
#include <unistd.h>
#endif

#ifdef _WIN32
#include <windows.h>
#include <time.h>
#endif

#include <cmath>

using namespace rw::common;

inline double round(double d) { return floor(d + 0.5); }

double TimerUtil::CurrentTime()
{
#if !(defined __MINGW32__) && !(defined _WIN32)
    timeval current;
    gettimeofday(&current, 0);
    return (double)current.tv_sec + (double)current.tv_usec / 1e6;
#else
    return static_cast<double>(clock()) / CLOCKS_PER_SEC;
#endif
}

void TimerUtil::SleepMs(int period)
{
#ifdef _WIN32
    Sleep(period);
#else
    usleep(1000 * period);
#endif
}

void TimerUtil::SleepUs(int period)
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

long TimerUtil::CurrentTimeMs()
{
    return (long)round(1e3 * CurrentTime());
}

long TimerUtil::CurrentTimeUs()
{
    return (long)round(1e6 * CurrentTime());
}
