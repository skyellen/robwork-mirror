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


#ifndef RWLIBS_OS_RWGL_HPP
#define RWLIBS_OS_RWGL_HPP

#include <rw/common/os.hpp>


#if defined(RW_CYGWIN)
    #include <windows.h>
    #include <GL/gl.h>
	#include <GL/glext.h>
    #include <GL/glu.h> // Header File For The GLu32 Library
#elif defined(RW_WIN32)
    #include <windows.h>
    #include <GL/gl.h>
	//#include <GL/glext.h>
	#include "glext_win32.h"
    #include <GL/glu.h> // Header File For The GLu32 Library	
#elif defined(RW_MACOS)
    #include <OpenGL/gl.h>
	#include <OpenGL/glext.h>
	#include <OpenGL/glu.h>
#elif defined(RW_LINUX)
    #include <GL/gl.h>
    #include <GL/glext.h>
    #include <GL/glu.h> // Header File For The GLu32 Library
#else
    #include <GL/gl.h>
    #include <GL/glext.h>
    #include <GL/glu.h> // Header File For The GLu32 Library
#endif

/*
#include <windows.h>
    #include <GL/gl.h>
    #include <GL/glext.h>
    #include <GL/glu.h> // Header File For The GLu32 Library

// function pointers for FBO extension
// Windows needs to get function pointers from ICD OpenGL drivers,
// because opengl32.dll does not support extensions higher than v1.1.
//#ifdef defined(RW_WIN32)

#define GL_FRAMEBUFFER_EXT                     0x8D40

#define GL_RENDERBUFFER_EXT                    0x8D41

#define GL_STENCIL_INDEX_EXT                   0x8D45
#define GL_STENCIL_INDEX1_EXT                  0x8D46
#define GL_STENCIL_INDEX4_EXT                  0x8D47
#define GL_STENCIL_INDEX8_EXT                  0x8D48
#define GL_STENCIL_INDEX16_EXT                 0x8D49

#define GL_RENDERBUFFER_WIDTH_EXT              0x8D42
#define GL_RENDERBUFFER_HEIGHT_EXT             0x8D43
#define GL_RENDERBUFFER_INTERNAL_FORMAT_EXT    0x8D44

#define GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE_EXT            0x8CD0
#define GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME_EXT            0x8CD1
#define GL_FRAMEBUFFER_ATTACHMENT_TEXTURE_LEVEL_EXT          0x8CD2
#define GL_FRAMEBUFFER_ATTACHMENT_TEXTURE_CUBE_MAP_FACE_EXT  0x8CD3
#define GL_FRAMEBUFFER_ATTACHMENT_TEXTURE_3D_ZOFFSET_EXT     0x8CD4

#define GL_COLOR_ATTACHMENT0_EXT                0x8CE0
#define GL_COLOR_ATTACHMENT1_EXT                0x8CE1
#define GL_COLOR_ATTACHMENT2_EXT                0x8CE2
#define GL_COLOR_ATTACHMENT3_EXT                0x8CE3
#define GL_COLOR_ATTACHMENT4_EXT                0x8CE4
#define GL_COLOR_ATTACHMENT5_EXT                0x8CE5
#define GL_COLOR_ATTACHMENT6_EXT                0x8CE6
#define GL_COLOR_ATTACHMENT7_EXT                0x8CE7
#define GL_COLOR_ATTACHMENT8_EXT                0x8CE8
#define GL_COLOR_ATTACHMENT9_EXT                0x8CE9
#define GL_COLOR_ATTACHMENT10_EXT               0x8CEA
#define GL_COLOR_ATTACHMENT11_EXT               0x8CEB
#define GL_COLOR_ATTACHMENT12_EXT               0x8CEC
#define GL_COLOR_ATTACHMENT13_EXT               0x8CED
#define GL_COLOR_ATTACHMENT14_EXT               0x8CEE
#define GL_COLOR_ATTACHMENT15_EXT               0x8CEF
#define GL_DEPTH_ATTACHMENT_EXT                 0x8D00
#define GL_STENCIL_ATTACHMENT_EXT               0x8D20

#define GL_FRAMEBUFFER_COMPLETE_EXT                          0x8CD5
#define GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT             0x8CD6
#define GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT     0x8CD7
#define GL_FRAMEBUFFER_INCOMPLETE_DUPLICATE_ATTACHMENT_EXT   0x8CD8
#define GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT             0x8CD9
#define GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT                0x8CDA
#define GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER_EXT            0x8CDB
#define GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER_EXT            0x8CDC
#define GL_FRAMEBUFFER_UNSUPPORTED_EXT                       0x8CDD
#define GL_FRAMEBUFFER_STATUS_ERROR_EXT                      0x8CDE

#define GL_FRAMEBUFFER_BINDING_EXT             0x8CA6
#define GL_RENDERBUFFER_BINDING_EXT            0x8CA7
#define GL_MAX_COLOR_ATTACHMENTS_EXT           0x8CDF
#define GL_MAX_RENDERBUFFER_SIZE_EXT           0x84E8

#define GL_INVALID_FRAMEBUFFER_OPERATION_EXT   0x0506

#define 	GL_FRAMEBUFFER_BINDING_EXT   0x8CA6


#define 	GL_EXT_framebuffer_object   1
#define 	GL_FRAMEBUFFER_EXT   0x8D40
#define 	GL_RENDERBUFFER_EXT   0x8D41
#define 	GL_STENCIL_INDEX1_EXT   0x8D46
#define 	GL_STENCIL_INDEX4_EXT   0x8D47
#define 	GL_STENCIL_INDEX8_EXT   0x8D48
#define 	GL_STENCIL_INDEX16_EXT   0x8D49
#define 	GL_RENDERBUFFER_WIDTH_EXT   0x8D42
#define 	GL_RENDERBUFFER_HEIGHT_EXT   0x8D43
#define 	GL_RENDERBUFFER_INTERNAL_FORMAT_EXT   0x8D44
#define 	GL_RENDERBUFFER_RED_SIZE_EXT   0x8D50
#define 	GL_RENDERBUFFER_GREEN_SIZE_EXT   0x8D51
#define 	GL_RENDERBUFFER_BLUE_SIZE_EXT   0x8D52
#define 	GL_RENDERBUFFER_ALPHA_SIZE_EXT   0x8D53
#define 	GL_RENDERBUFFER_DEPTH_SIZE_EXT   0x8D54
#define 	GL_RENDERBUFFER_STENCIL_SIZE_EXT   0x8D55
#define 	GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE_EXT   0x8CD0
#define 	GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME_EXT   0x8CD1
#define 	GL_FRAMEBUFFER_ATTACHMENT_TEXTURE_LEVEL_EXT   0x8CD2
#define 	GL_FRAMEBUFFER_ATTACHMENT_TEXTURE_CUBE_MAP_FACE_EXT   0x8CD3
#define 	GL_FRAMEBUFFER_ATTACHMENT_TEXTURE_3D_ZOFFSET_EXT   0x8CD4
#define 	GL_FRAMEBUFFER_ATTACHMENT_TEXTURE_LAYER_EXT   0x8CD4
#define 	GL_COLOR_ATTACHMENT0_EXT   0x8CE0
#define 	GL_COLOR_ATTACHMENT1_EXT   0x8CE1
#define 	GL_COLOR_ATTACHMENT2_EXT   0x8CE2
#define 	GL_COLOR_ATTACHMENT3_EXT   0x8CE3
#define 	GL_COLOR_ATTACHMENT4_EXT   0x8CE4
#define 	GL_COLOR_ATTACHMENT5_EXT   0x8CE5
#define 	GL_COLOR_ATTACHMENT6_EXT   0x8CE6
#define 	GL_COLOR_ATTACHMENT7_EXT   0x8CE7
#define 	GL_COLOR_ATTACHMENT8_EXT   0x8CE8
#define 	GL_COLOR_ATTACHMENT9_EXT   0x8CE9
#define 	GL_COLOR_ATTACHMENT10_EXT   0x8CEA
#define 	GL_COLOR_ATTACHMENT11_EXT   0x8CEB
#define 	GL_COLOR_ATTACHMENT12_EXT   0x8CEC
#define 	GL_COLOR_ATTACHMENT13_EXT   0x8CED
#define 	GL_COLOR_ATTACHMENT14_EXT   0x8CEE
#define 	GL_COLOR_ATTACHMENT15_EXT   0x8CEF
#define 	GL_DEPTH_ATTACHMENT_EXT   0x8D00
#define 	GL_STENCIL_ATTACHMENT_EXT   0x8D20
#define 	GL_FRAMEBUFFER_COMPLETE_EXT   0x8CD5
#define 	GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT   0x8CD6
#define 	GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT   0x8CD7
#define 	GL_FRAMEBUFFER_INCOMPLETE_DUPLICATE_ATTACHMENT_EXT   0x8CD8
#define 	GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT   0x8CD9
#define 	GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT   0x8CDA
#define 	GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER_EXT   0x8CDB
#define 	GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER_EXT   0x8CDC
#define 	GL_FRAMEBUFFER_UNSUPPORTED_EXT   0x8CDD
#define 	GL_FRAMEBUFFER_BINDING_EXT   0x8CA6
#define 	GL_RENDERBUFFER_BINDING_EXT   0x8CA7
#define 	GL_MAX_COLOR_ATTACHMENTS_EXT   0x8CDF
#define GL_MAX_RENDERBUFFER_SIZE_EXT 0x84E8
#define GL_INVALID_FRAMEBUFFER_OPERATION_EXT   0x0506
#define GL_EXT_framebuffer_blit   1
#define GL_DRAW_FRAMEBUFFER_BINDING_EXT   0x8CA6
#define GL_READ_FRAMEBUFFER_EXT   0x8CA8
#define GL_DRAW_FRAMEBUFFER_EXT   0x8CA9
#define GL_READ_FRAMEBUFFER_BINDING_EXT   0x8CAA
#define GL_EXT_framebuffer_multisample   1
#define GL_RENDERBUFFER_SAMPLES_EXT   0x8CAB
#define GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE_EXT   0x8D56
#define GL_MAX_SAMPLES_EXT   0x8D57
#define GL_NV_framebuffer_multisample_coverage   1
#define GL_RENDERBUFFER_COVERAGE_SAMPLES_NV   0x8CAB
#define GL_RENDERBUFFER_COLOR_SAMPLES_NV   0x8E10
#define GL_MAX_MULTISAMPLE_COVERAGE_MODES_NV   0x8E11
#define GL_MULTISAMPLE_COVERAGE_MODES_NV   0x8E12
#define GL_DEPTH_COMPONENT16   0x81A5
#define GL_DEPTH_COMPONENT24   0x81A6
#define GL_DEPTH_COMPONENT32   0x81A7
#define GL_EXT_packed_depth_stencil   1
#define GL_DEPTH_STENCIL_EXT   0x84F9
#define GL_UNSIGNED_INT_24_8_EXT   0x84FA
#define GL_DEPTH24_STENCIL8_EXT   0x88F0
#define GL_TEXTURE_STENCIL_SIZE_EXT   0x88F1



// Framebuffer object
GLboolean APIENTRY glIsRenderbufferEXT(GLuint renderbuffer);
void APIENTRY glBindRenderbufferEXT(GLenum target, GLuint renderbuffer);
void APIENTRY glDeleteRenderbuffersEXT(GLsizei n, const GLuint *renderbuffers);
void APIENTRY glGenRenderbuffersEXT(GLsizei n, GLuint *renderbuffers);

void APIENTRY glRenderbufferStorageEXT(GLenum target, GLenum internalformat,
                                       GLsizei width, GLsizei height);

void APIENTRY glGetRenderbufferParameterivEXT(GLenum target,
                                              GLenum pname, GLint* params);

GLboolean APIENTRY glIsFramebufferEXT(GLuint framebuffer);
void APIENTRY glBindFramebufferEXT(GLenum target, GLuint framebuffer);
void APIENTRY glDeleteFramebuffersEXT(GLsizei n, const GLuint *framebuffers);
void APIENTRY glGenFramebuffersEXT(GLsizei n, GLuint *framebuffers);

GLenum APIENTRY glCheckFramebufferStatusEXT(GLenum target);

void APIENTRY glFramebufferTexture1DEXT(GLenum target, GLenum attachment,
                                        GLenum textarget, GLuint texture,
                                        GLint level);
void APIENTRY glFramebufferTexture2DEXT(GLenum target, GLenum attachment,
                                        GLenum textarget, GLuint texture,
                                        GLint level);
void APIENTRY glFramebufferTexture3DEXT(GLenum target, GLenum attachment,
                                        GLenum textarget, GLuint texture,
                                        GLint level, GLint zoffset);

void APIENTRY glFramebufferRenderbufferEXT(GLenum target, GLenum attachment,
                                           GLenum renderbuffertarget,
                                           GLuint renderbuffer);

void APIENTRY glGetFramebufferAttachmentParameterivEXT(GLenum target,
                                                       GLenum attachment,
                                                       GLenum pname,
                                                       GLint *params);

void APIENTRY glGenerateMipmapEXT(GLenum target);

typedef GLboolean (APIENTRY * PFNGLISRENDERBUFFEREXTPROC) (GLuint renderbuffer);
typedef void (APIENTRY * PFNGLBINDRENDERBUFFEREXTPROC) (GLenum target, GLuint renderbuffer);
typedef void (APIENTRY * PFNGLDELETERENDERBUFFERSEXTPROC) (GLsizei n, const GLuint *renderbuffers);
typedef void (APIENTRY * PFNGLGENRENDERBUFFERSEXTPROC) (GLsizei n, GLuint *renderbuffers);

typedef void (APIENTRY * PFNGLRENDERBUFFERSTORAGEEXTPROC) (GLenum target, GLenum internalformat,
                                       GLsizei width, GLsizei height);

typedef void (APIENTRY * PFNGLGETRENDERBUFFERPARAMETERIVEXTPROC) (GLenum target,
                                              GLenum pname, GLint* params);

typedef GLboolean (APIENTRY * PFNGLISFRAMEBUFFEREXTPROC) (GLuint framebuffer);
typedef void (APIENTRY * PFNGLBINDFRAMEBUFFEREXTPROC) (GLenum target, GLuint framebuffer);
typedef void (APIENTRY * PFNGLDELETEFRAMEBUFFERSEXTPROC) (GLsizei n, const GLuint *framebuffers);
typedef void (APIENTRY * PFNGLGENFRAMEBUFFERSEXTPROC) (GLsizei n, GLuint *framebuffers);

typedef GLenum (APIENTRY * PFNGLCHECKFRAMEBUFFERSTATUSEXTPROC) (GLenum target);

typedef void (APIENTRY * PFNGLFRAMEBUFFERTEXTURE1DEXTPROC) (GLenum target, GLenum attachment,
                                        GLenum textarget, GLuint texture,
                                        GLint level);
typedef void (APIENTRY * PFNGLFRAMEBUFFERTEXTURE2DEXTPROC) (GLenum target, GLenum attachment,
                                        GLenum textarget, GLuint texture,
                                        GLint level);
typedef void (APIENTRY * PFNGLFRAMEBUFFERTEXTURE3DEXTPROC) (GLenum target, GLenum attachment,
                                        GLenum textarget, GLuint texture,
                                        GLint level, GLint zoffset);

typedef void (APIENTRY * PFNGLFRAMEBUFFERRENDERBUFFEREXTPROC) (GLenum target, GLenum attachment,
                                           GLenum renderbuffertarget,
                                           GLuint renderbuffer);

typedef void (APIENTRY * PFNGLGETFRAMEBUFFERATTACHMENTPARAMETERIVEXTPROC) (GLenum target,
                                                       GLenum attachment,
                                                       GLenum pname,
                                                       GLint *params);

typedef void (APIENTRY * PFNGLGENERATEMIPMAPEXTPROC) (GLenum target);
*/
//#endif

#endif /*RWLIBS_OS_RWGL_HPP*/
