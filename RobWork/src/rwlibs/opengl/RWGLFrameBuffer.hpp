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



namespace rw { namespace common { class LogWriter; } }


/*

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
*/

#if defined(RW_MACOS)
typedef GLboolean (* PFNGLISRENDERBUFFEREXTPROC) (GLuint renderbuffer);
typedef void (* PFNGLBINDRENDERBUFFEREXTPROC) (GLenum target, GLuint renderbuffer);
typedef void (* PFNGLDELETERENDERBUFFERSEXTPROC) (GLsizei n, const GLuint *renderbuffers);
typedef void (* PFNGLGENRENDERBUFFERSEXTPROC) (GLsizei n, GLuint *renderbuffers);
typedef void (* PFNGLRENDERBUFFERSTORAGEEXTPROC) (GLenum target, GLenum internalformat, GLsizei width, GLsizei height);
typedef void (* PFNGLGETRENDERBUFFERPARAMETERIVEXTPROC) (GLenum target, GLenum pname, GLint *params);
typedef GLboolean (* PFNGLISFRAMEBUFFEREXTPROC) (GLuint framebuffer);
typedef void (* PFNGLBINDFRAMEBUFFEREXTPROC) (GLenum target, GLuint framebuffer);
typedef void (* PFNGLDELETEFRAMEBUFFERSEXTPROC) (GLsizei n, const GLuint *framebuffers);
typedef void (* PFNGLGENFRAMEBUFFERSEXTPROC) (GLsizei n, GLuint *framebuffers);
typedef GLenum (* PFNGLCHECKFRAMEBUFFERSTATUSEXTPROC) (GLenum target);
typedef void (* PFNGLFRAMEBUFFERTEXTURE1DEXTPROC) (GLenum target, GLenum attachment, GLenum textarget, GLuint texture, GLint level);
typedef void (* PFNGLFRAMEBUFFERTEXTURE2DEXTPROC) (GLenum target, GLenum attachment, GLenum textarget, GLuint texture, GLint level);
typedef void (* PFNGLFRAMEBUFFERTEXTURE3DEXTPROC) (GLenum target, GLenum attachment, GLenum textarget, GLuint texture, GLint level, GLint zoffset);
typedef void (* PFNGLFRAMEBUFFERRENDERBUFFEREXTPROC) (GLenum target, GLenum attachment, GLenum renderbuffertarget, GLuint renderbuffer);
typedef void (* PFNGLGETFRAMEBUFFERATTACHMENTPARAMETERIVEXTPROC) (GLenum target, GLenum attachment, GLenum pname, GLint *params);
typedef void (* PFNGLGENERATEMIPMAPEXTPROC) (GLenum target);
#endif




namespace rwlibs {
namespace opengl {

class RWGLFrameBuffer
{
public:
    static bool initialize();
    static void test(rw::common::LogWriter& log);

    static bool hasFrameBuffers();
    static bool isFrameBuffersInitialized();
    static bool testFrameBufferCompleteness();

    // Framebuffer object
    static PFNGLGENFRAMEBUFFERSEXTPROC                     glGenFramebuffersEXT;                      // FBO name generation procedure
    static PFNGLDELETEFRAMEBUFFERSEXTPROC                  glDeleteFramebuffersEXT;                   // FBO deletion procedure
    static PFNGLBINDFRAMEBUFFEREXTPROC                     glBindFramebufferEXT;                      // FBO bind procedure
    static PFNGLCHECKFRAMEBUFFERSTATUSEXTPROC              glCheckFramebufferStatusEXT;               // FBO completeness test procedure
    static PFNGLGETFRAMEBUFFERATTACHMENTPARAMETERIVEXTPROC glGetFramebufferAttachmentParameterivEXT;  // return various FBO parameters
    static PFNGLGENERATEMIPMAPEXTPROC                      glGenerateMipmapEXT;                       // FBO automatic mipmap generation procedure
    static PFNGLFRAMEBUFFERTEXTURE2DEXTPROC                glFramebufferTexture2DEXT;                 // FBO texdture attachement procedure
    static PFNGLFRAMEBUFFERRENDERBUFFEREXTPROC             glFramebufferRenderbufferEXT;              // FBO renderbuffer attachement procedure
    static PFNGLBLITFRAMEBUFFEREXTPROC                     glBlitFrameBufferEXT;

    // Renderbuffer object
    static PFNGLGENRENDERBUFFERSEXTPROC                    glGenRenderbuffersEXT;                     // renderbuffer generation procedure
    static PFNGLDELETERENDERBUFFERSEXTPROC                 glDeleteRenderbuffersEXT;                  // renderbuffer deletion procedure
    static PFNGLBINDRENDERBUFFEREXTPROC                    glBindRenderbufferEXT;                     // renderbuffer bind procedure
    static PFNGLRENDERBUFFERSTORAGEEXTPROC                 glRenderbufferStorageEXT;                  // renderbuffer memory allocation procedure
    static PFNGLGETRENDERBUFFERPARAMETERIVEXTPROC          glGetRenderbufferParameterivEXT;           // return various renderbuffer parameters
    static PFNGLISRENDERBUFFEREXTPROC                      glIsRenderbufferEXT;                       // determine renderbuffer object type
    static PFNGLRENDERBUFFERSTORAGEMULTISAMPLEEXTPROC      glRenderbufferStorageMultisampleEXT;



    static PFNGLTEXIMAGE2DMULTISAMPLEPROC                  glTexImage2DMultisample;


private:
    RWGLFrameBuffer();
    static bool _hasFrameBuffers;
    static bool _frameBuffersInitialized;
};


} //end namespace simulation
} //end namespace rwlibs

#endif /* RWLIBS_SIMULATION_RWGLFRAMEBUFFER_HPP*/
