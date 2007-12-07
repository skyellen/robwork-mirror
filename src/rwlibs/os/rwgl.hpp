#include "rwos.hpp"

#ifndef RWGL_HPP_
#define RWGL_HPP_

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

#endif /*RWGL_HPP_*/
