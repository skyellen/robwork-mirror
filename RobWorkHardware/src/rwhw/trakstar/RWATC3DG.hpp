#ifndef RWHW_RWATC3DG_HPP
#define RWHW_RWATC3DG_HPP

#include <rw/common/os.hpp>

#ifdef RW_WIN32
#include <windows.h>
#include <stdio.h>      // printf
#include <string.h>     // string handling
#include <stdlib.h>     // exit() function
#include <time.h>       // needed for time functions
#endif

#ifdef RW_MAC
#define MAC
#endif


#ifdef RW_LINUX
#define LINUX
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <time.h>
#include <sys/types.h>
#endif

#include <ATC3DG.h>

#endif
