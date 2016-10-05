# Open Asset Import Library (assimp)
# ----------------------------------------------------------------------
#
# Copyright (c) 2006-2016, assimp team
# All rights reserved.
#
# Redistribution and use of this software in source and binary forms,
# with or without modification, are permitted provided that the
# following conditions are met:
#
# * Redistributions of source code must retain the above
#   copyright notice, this list of conditions and the
#   following disclaimer.
#
# * Redistributions in binary form must reproduce the above
#   copyright notice, this list of conditions and the
#   following disclaimer in the documentation and/or other
#   materials provided with the distribution.
#
# * Neither the name of the assimp team, nor the names of its
#   contributors may be used to endorse or promote products
#   derived from this software without specific prior
#   written permission of the assimp team.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#----------------------------------------------------------------------
SET(CMAKE_LEGACY_CYGWIN_WIN32 0) # Remove when CMake >= 2.8.4 is required
cmake_minimum_required( VERSION 2.8 )

IF(POLICY CMP0048)
	CMAKE_POLICY(SET CMP0048 NEW)
ENDIF()

PROJECT( Assimp )

OPTION(BUILD_SHARED_LIBS "Build package with shared libraries." ON)
IF(NOT BUILD_SHARED_LIBS)
  SET(LINK_SEARCH_START_STATIC TRUE)
ENDIF(NOT BUILD_SHARED_LIBS)

# Define here the needed parameters
SET (ASSIMP_VERSION_MAJOR 3)
SET (ASSIMP_VERSION_MINOR 3)
SET (ASSIMP_VERSION_PATCH 1) # subversion revision?
SET (ASSIMP_VERSION ${ASSIMP_VERSION_MAJOR}.${ASSIMP_VERSION_MINOR}.${ASSIMP_VERSION_PATCH})
SET (ASSIMP_SOVERSION 3)
#SET (PROJECT_VERSION "${ASSIMP_VERSION}") # not allowed in CMake 3.0 and higher - policy CMP0048

SET(ASSIMP_PACKAGE_VERSION "0" CACHE STRING "the package-specific version used for uploading the sources")

# Needed for openddl_parser config, no use of c++11 at this moment
add_definitions( -DOPENDDL_NO_USE_CPP11 )
#set_property( GLOBAL PROPERTY CXX_STANDARD 11 )

include_directories(
    ./
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Workaround to be able to deal with compiler bug "Too many sections" with mingw.
IF( CMAKE_COMPILER_IS_MINGW )
  ADD_DEFINITIONS(-DASSIMP_BUILD_NO_IFC_IMPORTER )
ENDIF()

ADD_DEFINITIONS(-DASSIMP_BUILD_NO_OWN_ZLIB)
ADD_DEFINITIONS( -DASSIMP_BUILD_NO_C4D_IMPORTER )

INCLUDE_DIRECTORIES( include )

ADD_SUBDIRECTORY( code/ )