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
