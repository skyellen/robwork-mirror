#=======================================================================
#//! \file
#//! \section sdhlibrary_cpp_makefile_sdh_general General file information
#//!   \author    Dirk Osswald 
#//!   \date      2007-05-15
#//!  
#//! \brief  
#//!   Makefile for SDH SDHLibrary C project - library part.
#//!
#//!   This makefile generates the C library itself
#//!   
#//!   For a general description of the project see \ref sdhlibrary_cpp_dox_general "general project information".
#//!
#//! \section sdhlibrary_cpp_makefile_sdh_variables Makefile variables
#//!   The variables defined here state project specific settings which are
#//!   then used by the goals and/or by the included, more generic sub makefiles
#//!   like:
#//!   - \ref Makefile-common "Makefile-common"
#//!   - \ref Makefile-rules  "Makefile-rules"
#//!   - \ref Makefile-cygwin "Makefile-cygwin"
#//! 
#//! \section sdhlibrary_cpp_makefile_sdh_targets Makefile targets
#//!   - \b \c all : generate everything
#//!     - \b \c build  : generate library
#//!     - \b \c build_lib : generate library
#//!   - \b \c clean    : clean up generated object and library files
#//!   - \b \c mrproper : clean up generated object and library files
#//!
#//! \section sdhlibrary_cpp_makefile_sdh_links Links
#//!   - The online documentation for \c gnu \c make can be found at
#//!     <a href="http://www.gnu.org/software/make/manual/make.html">
#//!     http://www.gnu.org/software/make/manual/make.html</a>
#//!  
#//! \section sdhlibrary_cpp_makefile_sdh_copyright Copyright
#//!
#//!  Copyright (c) 2007 SCHUNK GmbH & Co. KG
#//!
#//!  <HR>
#//!  \internal
#//!
#//!    \subsection sdhlibrary_cpp_makefile_sdh_details SVN related, detailed file specific information:
#//!      $LastChangedBy: Osswald2 $
#//!      $LastChangedDate: 2009-08-31 15:46:47 +0200 (Mo, 31 Aug 2009) $
#//!      \par SVN file revision:
#//!        $Id: Makefile 4766 2009-08-31 13:46:47Z Osswald2 $
#//!
#//!  \subsection sdhlibrary_cpp_makefile_sdh_changelog Changelog of this file:
#//!      \include Makefile.log
#//!
#=======================================================================
#//! \cond ignore_me   doxygen cannot parse Makefiles, so just ignore it

.DEFAULT_GOAL := all

########################################################################
# 


#-------------------
# Check if all settings are available 
# (You'd better call this Makefile from the top level makefile)

ifndef LIBRARY_TYPE
 ${error LIBRARY_TYPE is not set! Please call make in the top level directory!}
endif

ifndef SDHLIBRARY_NAME
 ${error SDHLIBRARY_NAME is not set! Please call make in the top level directory!}
endif

ifndef INSTALL_DIR_INC
 ${error INSTALL_DIR_INC is not set! Please call make in the top level directory!}
endif

ifndef INSTALL_DIR_LIB
 ${error INSTALL_DIR_LIB is not set! Please call make in the top level directory!}
endif

ifndef OSNAME
 ${error OSNAME is not set! Try <export OSNAME> and retry and please call make in the top level directory!}
endif
#-------------------

## list of header files to install:
INSTALL_HEADER_FILES=${wildcard *.h}

## Path to common stuff like sub-makefiles
COMMON_DIR=${shell if [ -d "../../../../common" ]; then \
	             echo "../../../../common";\
		   else \
	             echo "../" ; \
	           fi}

# Source files directory
SRCDIR = .

# Object files directory
OBJDIR = ../obj

# Binary files directory
BINDIR = ../bin

# Library files directory
LIBDIR = ../bin

# Dependency files directory
DEPDIR = ../.dep

# Optimization level, can be [0, 1, 2, 3, s]. 
#     0 = turn off optimization. s = optimize for size.
OPT = 0
#----------------------------


#---------------------------- 
# now the target files:

# state only the stems (filename without suffix):

TARGET1=${SDHLIBRARY_NAME}

# target file with path and suffix:
ifeq (${OSNAME},linux)
ifeq (${LIBRARY_TYPE},static)
PTARGET1 = ${LIBDIR}/lib${TARGET1}.a
endif
ifeq (${LIBRARY_TYPE},shared)
PTARGET1 = ${LIBDIR}/lib${TARGET1}.so
endif
endif

ifeq (${OSNAME},cygwin)
ifeq (${LIBRARY_TYPE},static)
PTARGET1 = ${LIBDIR}/lib${TARGET1}.a
endif
ifeq (${LIBRARY_TYPE},shared)
PTARGET1 = ${LIBDIR}/${TARGET1}.dll
endif
endif


# All target files:
PTARGETS = ${PTARGET1} 

#----------------------------
# The source files: all *.cpp but rs232-vcc.cpp
ALL_SRCS1 = ${subst rs232-vcc.cpp,,${wildcard *.cpp}}

# exclude the ESD CAN stuff if not desired
ifeq (${WITH_ESD_CAN},1)
  SRCS1 = ${ALL_SRCS1}
else
  SRCS1 = ${subst canserial-esd.cpp,,${ALL_SRCS1}}
endif

# exclude the PEAK CAN stuff if not desired
ifeq (${WITH_PEAK_CAN},1)
  SRCS2 = ${SRCS1}
else
  SRCS2 = ${subst canserial-peak.cpp,,${SRCS1}}
endif
#----------------------------

#----------------------------
# The object files:
OBJS1=${SRCS2:%.cpp=${OBJDIR}/%.o}
#----------------------------

#----------------------------
# explicit dependencies:
${PTARGET1}: ${OBJS1}
#----------------------------

#----------------------------

EXTRALDFLAGS += -L${LIBDIR} -l${TARGET1}

#----------------------------

#
########################################################################




########################################################################
# now the goals


# Default goal: Make all (library)
.PHONY: all
all: build 


# Build the library
.PHONY: build
build: build_lib

.PHONY: build_lib
build_lib: ${PTARGETS} 


# Clean up generated files.
.PHONY: clean
clean:   cygwin_clean 
	rm -f ${PTARGETS}


# Clean up generated files.
.PHONY: mrproper
mrproper: clean 


# install library and header files in designated directories
.PHONY: install
install: build
	install -d                                  ${INSTALL_DIR_INC}
	install -d                                  ${INSTALL_DIR_LIB}
	install --mode=644 ${INSTALL_HEADER_FILES}  ${INSTALL_DIR_INC} 
	install --mode=755 ${PTARGETS}              ${INSTALL_DIR_LIB} 


# Uninstall previously installed stuff
.PHONY: uninstall
uninstall: 
	@for f in  ${PTARGETS} ; do \
	  rm -f  ${INSTALL_DIR_LIB}/$$f ;        \
	done
	@for f in ${INSTALL_HEADER_FILES} ; do   \
	  rm -f  ${INSTALL_DIR_INC}/$$f ;        \
	done
#	# remove include dir if empty
	rmdir --ignore-fail-on-non-empty ${INSTALL_DIR_INC}
#	rmdir --ignore-fail-on-non-empty ${INSTALL_DIR_LIB}


#rules for gnerating library files

%.dll:
	${CPPC} -shared ${CPPFLAGS}  $^  -o $@ 

${PTARGET1}: EXTRALDFLAGS += -L${LIBDIR}
%.so:
	${CPPC} -o $@  -Wl,--export-dynamic -shared $^ 

%.a:
	ar -r $@  $^

#
########################################################################


######################################################################
#

# some common settings and targets are defined separately to keep this
# main Makefile more concise
export RELEASE_FILE=./release.h
include ${COMMON_DIR}/Makefile-common

# Common settings specific for cygwin on pc target platform
include ${COMMON_DIR}/Makefile-cygwin

#
########################################################################

#-----------------------------------------------------------------------
# emacs settings:
# Local Variables:
# mode: Makefile
# End:
#-----------------------------------------------------------------------
#//! \endcond
########################################################################
