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

#include "RWGLFrameBuffer.hpp"

#include <iostream>

using namespace rw::common;
using namespace rwlibs::opengl;



bool RWGLFrameBuffer::_hasFrameBuffers = false;
bool RWGLFrameBuffer::_frameBuffersInitialized = false;


// Framebuffer object
PFNGLGENFRAMEBUFFERSEXTPROC                     RWGLFrameBuffer::pglGenFramebuffersEXT = 0;                      // FBO name generation procedure
PFNGLDELETEFRAMEBUFFERSEXTPROC                  RWGLFrameBuffer::pglDeleteFramebuffersEXT = 0;                   // FBO deletion procedure
PFNGLBINDFRAMEBUFFEREXTPROC                     RWGLFrameBuffer::pglBindFramebufferEXT = 0;                      // FBO bind procedure
PFNGLCHECKFRAMEBUFFERSTATUSEXTPROC              RWGLFrameBuffer::pglCheckFramebufferStatusEXT = 0;               // FBO completeness test procedure
PFNGLGETFRAMEBUFFERATTACHMENTPARAMETERIVEXTPROC RWGLFrameBuffer::pglGetFramebufferAttachmentParameterivEXT = 0;  // return various FBO parameters
PFNGLGENERATEMIPMAPEXTPROC                      RWGLFrameBuffer::pglGenerateMipmapEXT = 0;                       // FBO automatic mipmap generation procedure
PFNGLFRAMEBUFFERTEXTURE2DEXTPROC                RWGLFrameBuffer::pglFramebufferTexture2DEXT = 0;                 // FBO texdture attachement procedure
PFNGLFRAMEBUFFERRENDERBUFFEREXTPROC             RWGLFrameBuffer::pglFramebufferRenderbufferEXT = 0;              // FBO renderbuffer attachement procedure
// Renderbuffer object
PFNGLGENRENDERBUFFERSEXTPROC                    RWGLFrameBuffer::pglGenRenderbuffersEXT = 0;                     // renderbuffer generation procedure
PFNGLDELETERENDERBUFFERSEXTPROC                 RWGLFrameBuffer::pglDeleteRenderbuffersEXT = 0;                  // renderbuffer deletion procedure
PFNGLBINDRENDERBUFFEREXTPROC                    RWGLFrameBuffer::pglBindRenderbufferEXT = 0;                     // renderbuffer bind procedure
PFNGLRENDERBUFFERSTORAGEEXTPROC                 RWGLFrameBuffer::pglRenderbufferStorageEXT = 0;                  // renderbuffer memory allocation procedure
PFNGLGETRENDERBUFFERPARAMETERIVEXTPROC          RWGLFrameBuffer::pglGetRenderbufferParameterivEXT = 0;           // return various renderbuffer parameters
PFNGLISRENDERBUFFEREXTPROC                      RWGLFrameBuffer::pglIsRenderbufferEXT = 0;                       // determine renderbuffer object type



bool RWGLFrameBuffer::initialize() {
    if (_frameBuffersInitialized)
        return _hasFrameBuffers;

    // check if FBO is supported by your video card
    //if(glInfo.isExtensionSupported("GL_EXT_framebuffer_object"))
#if defined(RW_WIN32)

    // get pointers to GL functions
    glGenFramebuffersEXT                     = (PFNGLGENFRAMEBUFFERSEXTPROC)wglGetProcAddress("glGenFramebuffersEXT");
    glDeleteFramebuffersEXT                  = (PFNGLDELETEFRAMEBUFFERSEXTPROC)wglGetProcAddress("glDeleteFramebuffersEXT");
    glBindFramebufferEXT                     = (PFNGLBINDFRAMEBUFFEREXTPROC)wglGetProcAddress("glBindFramebufferEXT");
    glCheckFramebufferStatusEXT              = (PFNGLCHECKFRAMEBUFFERSTATUSEXTPROC)wglGetProcAddress("glCheckFramebufferStatusEXT");
    glGetFramebufferAttachmentParameterivEXT = (PFNGLGETFRAMEBUFFERATTACHMENTPARAMETERIVEXTPROC)wglGetProcAddress("glGetFramebufferAttachmentParameterivEXT");
    glGenerateMipmapEXT                      = (PFNGLGENERATEMIPMAPEXTPROC)wglGetProcAddress("glGenerateMipmapEXT");
    glFramebufferTexture2DEXT                = (PFNGLFRAMEBUFFERTEXTURE2DEXTPROC)wglGetProcAddress("glFramebufferTexture2DEXT");
    glFramebufferRenderbufferEXT             = (PFNGLFRAMEBUFFERRENDERBUFFEREXTPROC)wglGetProcAddress("glFramebufferRenderbufferEXT");
    glGenRenderbuffersEXT                    = (PFNGLGENRENDERBUFFERSEXTPROC)wglGetProcAddress("glGenRenderbuffersEXT");
    glDeleteRenderbuffersEXT                 = (PFNGLDELETERENDERBUFFERSEXTPROC)wglGetProcAddress("glDeleteRenderbuffersEXT");
    glBindRenderbufferEXT                    = (PFNGLBINDRENDERBUFFEREXTPROC)wglGetProcAddress("glBindRenderbufferEXT");
    glRenderbufferStorageEXT                 = (PFNGLRENDERBUFFERSTORAGEEXTPROC)wglGetProcAddress("glRenderbufferStorageEXT");
    glGetRenderbufferParameterivEXT          = (PFNGLGETRENDERBUFFERPARAMETERIVEXTPROC)wglGetProcAddress("glGetRenderbufferParameterivEXT");
    glIsRenderbufferEXT                      = (PFNGLISRENDERBUFFEREXTPROC)wglGetProcAddress("glIsRenderbufferEXT");
#else
    // get pointers to GL functions
    glGenFramebuffersEXT                     = (PFNGLGENFRAMEBUFFERSEXTPROC)glXGetProcAddress((GLubyte*)"glGenFramebuffersEXT");
    glDeleteFramebuffersEXT                  = (PFNGLDELETEFRAMEBUFFERSEXTPROC)glXGetProcAddress((GLubyte*)"glDeleteFramebuffersEXT");
    glBindFramebufferEXT                     = (PFNGLBINDFRAMEBUFFEREXTPROC)glXGetProcAddress((GLubyte*)"glBindFramebufferEXT");
    glCheckFramebufferStatusEXT              = (PFNGLCHECKFRAMEBUFFERSTATUSEXTPROC)glXGetProcAddress((GLubyte*)"glCheckFramebufferStatusEXT");
    glGetFramebufferAttachmentParameterivEXT = (PFNGLGETFRAMEBUFFERATTACHMENTPARAMETERIVEXTPROC)glXGetProcAddress((GLubyte*)"glGetFramebufferAttachmentParameterivEXT");
    glGenerateMipmapEXT                      = (PFNGLGENERATEMIPMAPEXTPROC)glXGetProcAddress((GLubyte*)"glGenerateMipmapEXT");
    glFramebufferTexture2DEXT                = (PFNGLFRAMEBUFFERTEXTURE2DEXTPROC)glXGetProcAddress((GLubyte*)"glFramebufferTexture2DEXT");
    glFramebufferRenderbufferEXT             = (PFNGLFRAMEBUFFERRENDERBUFFEREXTPROC)glXGetProcAddress((GLubyte*)"glFramebufferRenderbufferEXT");
    glGenRenderbuffersEXT                    = (PFNGLGENRENDERBUFFERSEXTPROC)glXGetProcAddress((GLubyte*)"glGenRenderbuffersEXT");
    glDeleteRenderbuffersEXT                 = (PFNGLDELETERENDERBUFFERSEXTPROC)glXGetProcAddress((GLubyte*)"glDeleteRenderbuffersEXT");
    glBindRenderbufferEXT                    = (PFNGLBINDRENDERBUFFEREXTPROC)glXGetProcAddress((GLubyte*)"glBindRenderbufferEXT");
    glRenderbufferStorageEXT                 = (PFNGLRENDERBUFFERSTORAGEEXTPROC)glXGetProcAddress((GLubyte*)"glRenderbufferStorageEXT");
    glGetRenderbufferParameterivEXT          = (PFNGLGETRENDERBUFFERPARAMETERIVEXTPROC)glXGetProcAddress((GLubyte*)"glGetRenderbufferParameterivEXT");
    glIsRenderbufferEXT                      = (PFNGLISRENDERBUFFEREXTPROC)glXGetProcAddress((GLubyte*)"glIsRenderbufferEXT");

#endif
    // check once again FBO extension
    if(glGenFramebuffersEXT && glDeleteFramebuffersEXT && glBindFramebufferEXT && glCheckFramebufferStatusEXT &&
       glGetFramebufferAttachmentParameterivEXT && glGenerateMipmapEXT && glFramebufferTexture2DEXT && glFramebufferRenderbufferEXT &&
       glGenRenderbuffersEXT && glDeleteRenderbuffersEXT && glBindRenderbufferEXT && glRenderbufferStorageEXT &&
       glGetRenderbufferParameterivEXT && glIsRenderbufferEXT)
    {
        _hasFrameBuffers = true;
        std::cout << "Video card supports GL_EXT_framebuffer_object." << std::endl;
    }
    else
    {
        _hasFrameBuffers = false;
        std::cout << "Video card does NOT support GL_EXT_framebuffer_object." << std::endl;
    }

    _frameBuffersInitialized = true;
    return _hasFrameBuffers;

}

void RWGLFrameBuffer::test(LogWriter& log) {
    GLint var;
    glGetIntegerv(GL_MAX_RENDERBUFFER_SIZE_EXT, &var);
    (log)<< "GL_MAX_RENDERBUFFER_SIZE_EXT: " << var << std::endl;
    glGetRenderbufferParameterivEXT(GL_RENDERBUFFER_EXT, GL_RENDERBUFFER_WIDTH_EXT, &var);
    (log)<< "GL_RENDERBUFFER_WIDTH_EXT: " << var << std::endl;
    glGetRenderbufferParameterivEXT(GL_RENDERBUFFER_EXT, GL_RENDERBUFFER_HEIGHT_EXT, &var);
    (log)<< "GL_RENDERBUFFER_HEIGHT_EXT: " << var << std::endl;
    glGetRenderbufferParameterivEXT(GL_RENDERBUFFER_EXT, GL_RENDERBUFFER_INTERNAL_FORMAT_EXT, &var);
    (log)<< "GL_RENDERBUFFER_INTERNAL_FORMAT_EXT: " << var << std::endl;
    glGetRenderbufferParameterivEXT(GL_RENDERBUFFER_EXT, GL_RENDERBUFFER_RED_SIZE_EXT, &var);
    (log)<< "GL_RENDERBUFFER_RED_SIZE_EXT: " << var << std::endl;
    glGetRenderbufferParameterivEXT(GL_RENDERBUFFER_EXT, GL_RENDERBUFFER_GREEN_SIZE_EXT, &var);
    (log)<< "GL_RENDERBUFFER_GREEN_SIZE_EXT: " << var << std::endl;
    glGetRenderbufferParameterivEXT(GL_RENDERBUFFER_EXT, GL_RENDERBUFFER_BLUE_SIZE_EXT, &var);
    (log)<< "GL_RENDERBUFFER_BLUE_SIZE_EXT: " << var << std::endl;
    glGetRenderbufferParameterivEXT(GL_RENDERBUFFER_EXT, GL_RENDERBUFFER_ALPHA_SIZE_EXT, &var);
    (log)<< "GL_RENDERBUFFER_ALPHA_SIZE_EXT: " << var << std::endl;
    glGetRenderbufferParameterivEXT(GL_RENDERBUFFER_EXT, GL_RENDERBUFFER_DEPTH_SIZE_EXT, &var);
    (log)<< "GL_RENDERBUFFER_DEPTH_SIZE_EXT: " << var << std::endl;
    glGetRenderbufferParameterivEXT(GL_RENDERBUFFER_EXT, GL_RENDERBUFFER_STENCIL_SIZE_EXT, &var);
    (log)<< "GL_RENDERBUFFER_STENCIL_SIZE_EXT: " << var << std::endl;


}


bool RWGLFrameBuffer::hasFrameBuffers() {
    return _hasFrameBuffers;
}

bool RWGLFrameBuffer::isFrameBuffersInitialized() {
    return _frameBuffersInitialized;
}


RWGLFrameBuffer::RWGLFrameBuffer()
{

}
