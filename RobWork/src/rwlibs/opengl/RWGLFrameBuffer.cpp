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

#include <rw/common/Log.hpp>

using namespace rw::common;
using namespace rwlibs::opengl;



bool RWGLFrameBuffer::_hasFrameBuffers = false;
bool RWGLFrameBuffer::_frameBuffersInitialized = false;


// Framebuffer object
PFNGLGENFRAMEBUFFERSEXTPROC                     RWGLFrameBuffer::glGenFramebuffersEXT = 0;                      // FBO name generation procedure
PFNGLDELETEFRAMEBUFFERSEXTPROC                  RWGLFrameBuffer::glDeleteFramebuffersEXT = 0;                   // FBO deletion procedure
PFNGLBINDFRAMEBUFFEREXTPROC                     RWGLFrameBuffer::glBindFramebufferEXT = 0;                      // FBO bind procedure
PFNGLCHECKFRAMEBUFFERSTATUSEXTPROC              RWGLFrameBuffer::glCheckFramebufferStatusEXT = 0;               // FBO completeness test procedure
PFNGLGETFRAMEBUFFERATTACHMENTPARAMETERIVEXTPROC RWGLFrameBuffer::glGetFramebufferAttachmentParameterivEXT = 0;  // return various FBO parameters
PFNGLGENERATEMIPMAPEXTPROC                      RWGLFrameBuffer::glGenerateMipmapEXT = 0;                       // FBO automatic mipmap generation procedure
PFNGLFRAMEBUFFERTEXTURE2DEXTPROC                RWGLFrameBuffer::glFramebufferTexture2DEXT = 0;                 // FBO texdture attachement procedure
PFNGLFRAMEBUFFERRENDERBUFFEREXTPROC             RWGLFrameBuffer::glFramebufferRenderbufferEXT = 0;              // FBO renderbuffer attachement procedure
// Renderbuffer object
PFNGLGENRENDERBUFFERSEXTPROC                    RWGLFrameBuffer::glGenRenderbuffersEXT = 0;                     // renderbuffer generation procedure
PFNGLDELETERENDERBUFFERSEXTPROC                 RWGLFrameBuffer::glDeleteRenderbuffersEXT = 0;                  // renderbuffer deletion procedure
PFNGLBINDRENDERBUFFEREXTPROC                    RWGLFrameBuffer::glBindRenderbufferEXT = 0;                     // renderbuffer bind procedure
PFNGLRENDERBUFFERSTORAGEEXTPROC                 RWGLFrameBuffer::glRenderbufferStorageEXT = 0;                  // renderbuffer memory allocation procedure
PFNGLGETRENDERBUFFERPARAMETERIVEXTPROC          RWGLFrameBuffer::glGetRenderbufferParameterivEXT = 0;           // return various renderbuffer parameters
PFNGLISRENDERBUFFEREXTPROC                      RWGLFrameBuffer::glIsRenderbufferEXT = 0;                       // determine renderbuffer object type
PFNGLRENDERBUFFERSTORAGEMULTISAMPLEEXTPROC      RWGLFrameBuffer::glRenderbufferStorageMultisampleEXT = 0;
PFNGLTEXIMAGE2DMULTISAMPLEPROC                  RWGLFrameBuffer::glTexImage2DMultisample = 0;
PFNGLBLITFRAMEBUFFEREXTPROC                     RWGLFrameBuffer::glBlitFrameBufferEXT = 0;
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
    glRenderbufferStorageMultisampleEXT      = (PFNGLRENDERBUFFERSTORAGEMULTISAMPLEEXTPROC)wglGetProcAddress("glRenderbufferStorageMultisampleEXT");
    glTexImage2DMultisample                  = (PFNGLTEXIMAGE2DMULTISAMPLEPROC)wglGetProcAddress("glTexImage2DMultisample");
    glBlitFrameBufferEXT                     = (PFNGLBLITFRAMEBUFFEREXTPROC) wglGetProcAddress("glBlitFrameBufferEXT");

#elif defined(RW_MACOS)

    glGenFramebuffersEXT                     = &glGenFramebuffers;
    glDeleteFramebuffersEXT                  = &glDeleteFramebuffers;
    glBindFramebufferEXT                     = &glBindFramebuffer;
    glCheckFramebufferStatusEXT              = &glCheckFramebufferStatus;
    glGetFramebufferAttachmentParameterivEXT = &glGetFramebufferAttachmentParameteriv;
    glGenerateMipmapEXT                      = &glGenerateMipmap;
    glFramebufferTexture2DEXT                = &glFramebufferTexture2D;
    glFramebufferRenderbufferEXT             = &glFramebufferRenderbuffer;
    glGenRenderbuffersEXT                    = &glGenRenderbuffers;
    glDeleteRenderbuffersEXT                 = &glDeleteRenderbuffers;
    glBindRenderbufferEXT                    = &glBindRenderbuffer;
    glRenderbufferStorageEXT                 = &glRenderbufferStorage;
    glGetRenderbufferParameterivEXT          = &glGetRenderbufferParameteriv;
    glIsRenderbufferEXT                      = &glIsRenderbuffer;
    glRenderbufferStorageMultisampleEXT      = &glRenderbufferStorageMultisampleEXT;
    glTexImage2DMultisample                  = &glTexImage2DMultisample;
    glBlitFrameBufferEXT                     = &glBlitFrameBufferEXT;

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
    glRenderbufferStorageMultisampleEXT      = (PFNGLRENDERBUFFERSTORAGEMULTISAMPLEEXTPROC)glXGetProcAddress((GLubyte*)"glRenderbufferStorageMultisampleEXT");
    glTexImage2DMultisample                  = (PFNGLTEXIMAGE2DMULTISAMPLEPROC)glXGetProcAddress((GLubyte*)"glTexImage2DMultisample");
    glBlitFrameBufferEXT                     = (PFNGLBLITFRAMEBUFFEREXTPROC) glXGetProcAddress((GLubyte*)"glBlitFramebufferEXT");

#endif
    // check once again FBO extension
    if(glGenFramebuffersEXT && glDeleteFramebuffersEXT && glBindFramebufferEXT && glCheckFramebufferStatusEXT &&
       glGetFramebufferAttachmentParameterivEXT && glGenerateMipmapEXT && glFramebufferTexture2DEXT && glFramebufferRenderbufferEXT &&
       glGenRenderbuffersEXT && glDeleteRenderbuffersEXT && glBindRenderbufferEXT && glRenderbufferStorageEXT &&
       glGetRenderbufferParameterivEXT && glIsRenderbufferEXT && glTexImage2DMultisample && glRenderbufferStorageMultisampleEXT && glBlitFrameBufferEXT)
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

bool RWGLFrameBuffer::testFrameBufferCompleteness(){

        GLenum status;
        status = RWGLFrameBuffer::glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
        switch (status)
        {
        case GL_FRAMEBUFFER_COMPLETE_EXT:
            return true;
            break;
        case GL_FRAMEBUFFER_UNSUPPORTED_EXT:
            //Choose different formats
            Log::errorLog()
                    << "Framebuffer object format is unsupported by the video hardware. (GL_FRAMEBUFFER_UNSUPPORTED_EXT)(FBO - 820)";
            break;
        case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT:
            Log::errorLog() << "Incomplete attachment. (GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT)(FBO - 820)";
            break;
        case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT:
            Log::errorLog()
                    << "Incomplete missing attachment. (GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT)(FBO - 820)";
            break;
        case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT:
            Log::errorLog() << "Incomplete dimensions. (GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT)(FBO - 820)";
            break;
        case GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT:
            Log::errorLog() << "Incomplete formats. (GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT)(FBO - 820)";
            break;
        case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER_EXT:
            Log::errorLog() << "Incomplete draw buffer. (GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER_EXT)(FBO - 820)";
            break;
        case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER_EXT:
            Log::errorLog() << "Incomplete read buffer. (GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER_EXT)(FBO - 820)";
            break;
        case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE_EXT:
            Log::errorLog()
                    << "Incomplete multisample buffer. (GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE_EXT)(FBO - 820)";
            break;
        default:
            //Programming error; will fail on all hardware
            Log::errorLog()
                    << "Some video driver error or programming error occured. Framebuffer object status is invalid. (FBO - 823)";
        }
        return false;
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
