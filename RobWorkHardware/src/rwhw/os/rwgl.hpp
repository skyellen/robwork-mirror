#ifndef RWLIBS_OS_RWGL_HPP
#define RWLIBS_OS_RWGL_HPP

#include <rw/common/os.hpp>


#if defined(RW_CYGWIN)
    #include <windows.h>
    #include <GL/gl.h>
    #include <GL/glu.h> // Header File For The GLu32 Library
#elif defined(RW_WIN32)
    #include <windows.h>
    #include <GL/gl.h>
    #include <GL/glu.h> // Header File For The GLu32 Library
#elif defined(RW_MACOS)
    #include <OpenGL/gl.h>
    #include <OpenGL/glu.h>
#elif defined(RW_LINUX)
    #include <GL/gl.h>
    #include <GL/glu.h> // Header File For The GLu32 Library
#else
    #include <GL/gl.h>
    #include <GL/glu.h> // Header File For The GLu32 Library
#endif

#endif /*RWLIBS_OS_RWGL_HPP*/
