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
#ifdef _WIN32
    return (long) (clock()* (double(1e3)/CLOCKS_PER_SEC));
#else
    timeval current;

    gettimeofday(&current, 0);
    return current.tv_sec * 1000L + current.tv_usec / 1000L;

#endif
}

long TimerUtil::currentTimeUs()
{
#ifdef _WIN32
	return (long) (clock()* (double(1e6)/CLOCKS_PER_SEC));
#else
    timeval current;

    gettimeofday(&current, 0);
    return current.tv_sec * 1000000L + current.tv_usec;
#endif
}
