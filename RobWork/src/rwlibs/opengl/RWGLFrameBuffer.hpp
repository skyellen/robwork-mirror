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

#ifndef RWLIBS_SIMULATION_RWGLFRAMEBUFFER_HPP
#define RWLIBS_SIMULATION_RWGLFRAMEBUFFER_HPP

#include <rwlibs/os/rwgl.hpp>
#if defined(RW_CYGWIN)
    #include <GL/glext.h>
#elif defined(RW_WIN32)
    #include <rwlibs/os/glext_win32.h>
#elif defined(RW_MACOS)
    #include <OpenGL/glext.h>
#elif defined(RW_LINUX)
    #include <GL/glx.h>
    #include <GL/glext.h>
    #include <GL/glxext.h>
#else
    #include <GL/glext.h>
#endif

#include <rw/common/LogWriter.hpp>




#define glGenFramebuffersEXT                        pglGenFramebuffersEXT
#define glDeleteFramebuffersEXT                     pglDeleteFramebuffersEXT
#define glBindFramebufferEXT                        pglBindFramebufferEXT
#define glCheckFramebufferStatusEXT                 pglCheckFramebufferStatusEXT
#define glGetFramebufferAttachmentParameterivEXT    pglGetFramebufferAttachmentParameterivEXT
#define glGenerateMipmapEXT                         pglGenerateMipmapEXT
#define glFramebufferTexture2DEXT                   pglFramebufferTexture2DEXT
#define glFramebufferRenderbufferEXT                pglFramebufferRenderbufferEXT

#define glGenRenderbuffersEXT                       pglGenRenderbuffersEXT
#define glDeleteRenderbuffersEXT                    pglDeleteRenderbuffersEXT
#define glBindRenderbufferEXT                       pglBindRenderbufferEXT
#define glRenderbufferStorageEXT                    pglRenderbufferStorageEXT
#define glGetRenderbufferParameterivEXT             pglGetRenderbufferParameterivEXT
#define glIsRenderbufferEXT                         pglIsRenderbufferEXT




namespace rwlibs {
namespace opengl {




class RWGLFrameBuffer
{
public:
    static bool initialize();
    static void test(rw::common::LogWriter& log);

    static bool hasFrameBuffers();
    static bool isFrameBuffersInitialized();

    // Framebuffer object
    static PFNGLGENFRAMEBUFFERSEXTPROC                     pglGenFramebuffersEXT;                      // FBO name generation procedure
    static PFNGLDELETEFRAMEBUFFERSEXTPROC                  pglDeleteFramebuffersEXT;                   // FBO deletion procedure
    static PFNGLBINDFRAMEBUFFEREXTPROC                     pglBindFramebufferEXT;                      // FBO bind procedure
    static PFNGLCHECKFRAMEBUFFERSTATUSEXTPROC              pglCheckFramebufferStatusEXT;               // FBO completeness test procedure
    static PFNGLGETFRAMEBUFFERATTACHMENTPARAMETERIVEXTPROC pglGetFramebufferAttachmentParameterivEXT;  // return various FBO parameters
    static PFNGLGENERATEMIPMAPEXTPROC                      pglGenerateMipmapEXT;                       // FBO automatic mipmap generation procedure
    static PFNGLFRAMEBUFFERTEXTURE2DEXTPROC                pglFramebufferTexture2DEXT;                 // FBO texdture attachement procedure
    static PFNGLFRAMEBUFFERRENDERBUFFEREXTPROC             pglFramebufferRenderbufferEXT;              // FBO renderbuffer attachement procedure
    // Renderbuffer object
    static PFNGLGENRENDERBUFFERSEXTPROC                    pglGenRenderbuffersEXT;                     // renderbuffer generation procedure
    static PFNGLDELETERENDERBUFFERSEXTPROC                 pglDeleteRenderbuffersEXT;                  // renderbuffer deletion procedure
    static PFNGLBINDRENDERBUFFEREXTPROC                    pglBindRenderbufferEXT;                     // renderbuffer bind procedure
    static PFNGLRENDERBUFFERSTORAGEEXTPROC                 pglRenderbufferStorageEXT;                  // renderbuffer memory allocation procedure
    static PFNGLGETRENDERBUFFERPARAMETERIVEXTPROC          pglGetRenderbufferParameterivEXT;           // return various renderbuffer parameters
    static PFNGLISRENDERBUFFEREXTPROC                      pglIsRenderbufferEXT;                       // determine renderbuffer object type



private:
    RWGLFrameBuffer();
    static bool _hasFrameBuffers;
    static bool _frameBuffersInitialized;
};

} //end namespace simulation
} //end namespace rwlibs

#endif /* RWLIBS_SIMULATION_RWGLFRAMEBUFFER_HPP*/
