#=======================================================================
#//! \file
#//! \section sdhlibrary_cpp_makefile_demo_general General file information
#//!   \author    Dirk Osswald 
#//!   \date      2007-01-03
#//!  
#//! \brief  
#//!   Makefile for SDH SDHLibrary C project - demo programs.
#//!
#//!   This makefile can generate the demo programs using the sdh C++ library.
#//!   
#//!   For a general description of the project see \ref sdhlibrary_cpp_dox_general "general project information".
#//!
#//! \section sdhlibrary_cpp_makefile_demo_variables Makefile variables
#//!   The variables defined here state project specific settings which are
#//!   then used by the goals and/or by the included, more generic sub makefiles
#//!   like:
#//!   - \ref Makefile-common "Makefile-common"
#//!   - \ref Makefile-rules  "Makefile-rules"
#//!   - \ref Makefile-cygwin "Makefile-cygwin"
#//! 
#//! \section sdhlibrary_cpp_makefile_demo_targets Makefile targets
#//!   - \b \c all : generate everything
#//!     - \b \c build  : generate library
#//!   - \b \c clean    : clean up generated program files, but not TAGS or doxygen doc
#//!   - \b \c mrproper : clean up all generated files, including TAGS and doxygen doc
#//!
#//! \section sdhlibrary_cpp_makefile_demo_links Links
#//!   - The online documentation for \c gnu \c make can be found at
#//!     <a href="http://www.gnu.org/software/make/manual/make.html">
#//!     http://www.gnu.org/software/make/manual/make.html</a>
#//!  
#//! \section sdhlibrary_cpp_makefile_demo_copyright Copyright
#//!
#//!  Copyright (c) 2007 SCHUNK GmbH & Co. KG
#//!
#//!  <HR>
#//!  \internal
#//!
#//!    \subsection sdhlibrary_cpp_makefile_demo_details SVN related, detailed file specific information:
#//!      $LastChangedBy: Osswald2 $
#//!      $LastChangedDate: 2008-10-08 10:48:38 +0200 (Mi, 08 Okt 2008) $
#//!      \par SVN file revision:
#//!        $Id: Makefile 3659 2008-10-08 08:48:38Z Osswald2 $
#//!
#//!  \subsection sdhlibrary_cpp_makefile_demo_changelog Changelog of this file:
#//!      \include Makefile.log
#//!
#=======================================================================
#//! \cond ignore_me   doxygen cannot parse Makefiles, so just ignore it

.DEFAULT_GOAL := all

########################################################################
# first some variables

#-------------------
# Check if all settings are available 
# (You'd better call this Makefile from the top level makefile)

ifndef LIBRARY_TYPE
 ${error LIBRARY_TYPE is not set! Please call make in the top level directory!}
endif

ifndef SDHLIBRARY_NAME
 ${error SDHLIBRARY_NAME is not set! Please call make in the top level directory!}
endif

ifndef INSTALL_DIR_BIN
 ${error INSTALL_DIR_INC is not set! Please call make in the top level directory!}
endif

ifndef OSNAME
 ${error OSNAME is not set! Please call make in the top level directory!}
endif
#-------------------


#----------------------------
# some directories:

## Path to common stuff like sub-makefiles
COMMON_DIR=${shell if [ -d "../../../../common" ]; then \
	             echo "../../../../common";\
		   else \
	             echo "../" ; \
	           fi}

## Source files directory
SRCDIR = .

## Object files directory
OBJDIR = ../obj

## Binary files directory
BINDIR = ../bin

## Library files directory
LIBDIR = ../bin

## Dependency files directory
DEPDIR = ../.dep

## Optimization level, can be [0, 1, 2, 3, s]. 
#     0 = turn off optimization. s = optimize for size.
OPT = 0

## Suffix of the executable (if any)
ifeq (${OSNAME},cygwin) 
  EXESUFFIX=.exe
else
  EXESUFFIX=
endif

## (local) library name with path and suffix:
ifeq (${OSNAME},linux)
ifeq (${LIBRARY_TYPE},static)
PSDHLIBRARY_NAME = ${LIBDIR}/lib${SDHLIBRARY_NAME}.a
endif
ifeq (${LIBRARY_TYPE},shared)
PSDHLIBRARY_NAME = ${LIBDIR}/lib${SDHLIBRARY_NAME}.so
endif
endif

ifeq (${OSNAME},cygwin)
ifeq (${LIBRARY_TYPE},static)
PSDHLIBRARY_NAME = ${LIBDIR}/lib${SDHLIBRARY_NAME}.a
endif
ifeq (${LIBRARY_TYPE},shared)
PSDHLIBRARY_NAME = ${LIBDIR}/${SDHLIBRARY_NAME}.dll
endif
endif

#----------------------------

#---------------------------- 
## now the target files:
#
# state only the stems (filename without suffix):
#   (These simple programs consist of one source file only,
#    hence the further dependencies are not stated here explicitly but
#    are created implicitly below)
TARGETS_DEMO=${basename ${wildcard demo-*.cpp}}
ALL_TARGETS_TEST=${basename ${wildcard test-*.cpp}}

ifeq (${SDH_USE_NAMESPACE},1)
    TARGETS_TEST=${ALL_TARGETS_TEST}
else
	# SDH namespace is not used, so the test-*namespace* programs will 
	# not compile properly, so remove them from the list of targets again
	# (order does matter here! The most specific string must be first!)
	ALL2_TARGETS_TEST=$(subst test-sdhnamespace-dsa,,${ALL_TARGETS_TEST})
	ALL3_TARGETS_TEST=$(subst test-sdhnamespace,,${ALL2_TARGETS_TEST})
	TARGETS_TEST=$(subst test-namespaces,,${ALL3_TARGETS_TEST})
endif
	
TARGETS=${TARGETS_DEMO} ${TARGETS_TEST}

ifeq (${WITH_ESD_CAN},1)
TARGETS+=cancat
endif


## target files with path but still without suffix:
PTARGETS=${TARGETS:%=${BINDIR}/%}

#----------------------------
# explicit dependencies:


## Dependency file with the implicitly calculated dependencies:
DEMO_DEPS = ${DEPDIR}/demo_deps

# create the dependency file:
${shell rm -f ${DEMO_DEPS};                                  		\
	for t in ${TARGETS} ; do                                        \
	    echo "${BINDIR}/$${t}: ${OBJDIR}/$${t}.o" >> ${DEMO_DEPS} ; \
	    echo "${OBJDIR}/$${t}.o: $${t}.cpp"       >> ${DEMO_DEPS} ; \
	done}

# and include it:
include ${DEMO_DEPS}

#extra dependencies:
${BINDIR}/demo-GetAxisActualAngle: ${OBJDIR}/sdhoptions.o
${BINDIR}/demo-GetFingerXYZ: ${OBJDIR}/sdhoptions.o
${BINDIR}/demo-temperature: ${OBJDIR}/sdhoptions.o
${BINDIR}/demo-test: ${OBJDIR}/sdhoptions.o
${BINDIR}/demo-simple: ${OBJDIR}/sdhoptions.o
${BINDIR}/demo-simple-withtiming: ${OBJDIR}/sdhoptions.o
${BINDIR}/demo-simple2: ${OBJDIR}/sdhoptions.o
${BINDIR}/demo-simple3: ${OBJDIR}/sdhoptions.o
${BINDIR}/demo-dsa:     ${OBJDIR}/dsaoptions.o

${BINDIR}/cancat:       ${OBJDIR}/sdhoptions.o

${BINDIR}/test-sdhnamespace:     ${OBJDIR}/sdhoptions.o
${BINDIR}/test-sdhnamespace-dsa: ${OBJDIR}/dsaoptions.o

${OBJDIR}/sdhoptions.o: sdhoptions.cpp sdhoptions.h
${OBJDIR}/dsaoptions.o: dsaoptions.cpp dsaoptions.h


# demo programs depend on the library (in the static case):
ifeq (${LIBRARY_TYPE},static)
${PTARGETS}: ${PSDHLIBRARY_NAME}
endif

#----------------------------

## Add needed options for the linker :
EXTRALDFLAGS += -L${LIBDIR} -l${SDHLIBRARY_NAME}
EXTRAINCDIRS += ../
#----------------------------

#
########################################################################




########################################################################
# now the goals


# Default goal: Make all (programs) 
.PHONY: all
all: build 

# Build the library
.PHONY: build
build: ${PTARGETS} 


# Clean up generated files 
.PHONY: clean
clean:   cygwin_clean 


# Clean up generated files 
.PHONY: mrproper
mrproper: clean 


# install demo programs in designated directories
.PHONY: install
install: build
	install -d                       ${INSTALL_DIR_BIN}
	install --mode=755 ${PTARGETS}   ${INSTALL_DIR_BIN} 


# Uninstall previously installed stuff
.PHONY: uninstall
uninstall:
	@for f in  ${PTARGETS} ; do                                       \
	  echo "removing ${INSTALL_DIR_BIN}/`basename $$f`${EXESUFFIX}" ; \
	  rm -f  ${INSTALL_DIR_BIN}/`basename $$f`${EXESUFFIX} ;          \
	done
#	rmdir --ignore-fail-on-non-empty ${INSTALL_DIR_BIN}


#
########################################################################


########################################################################
#

# some common settings and targets are defined separately to keep this
# main Makefile more concise
export RELEASE_FILE=../sdh/release.h
include ${COMMON_DIR}/Makefile-common

# Common settings specific for cygwin on pc target platform
include ${COMMON_DIR}/Makefile-cygwin

#overwrite settings:
CC=g++

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
