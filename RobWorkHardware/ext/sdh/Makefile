#=======================================================================
#//! \file
#//! \section sdhlibrary_cpp_makefile_general General file information
#//!   \author    Dirk Osswald 
#//!   \date      2007-01-03
#//!  
#//! \brief  
#//!   Makefile for SDH SDHLibrary C project.
#//!
#//!   This makefile can generate the C library itself, demo-programs and auxiliary stuff like
#//!   doxygen documentation or generate a distribution for delivery to end users.
#//!   
#//!   For a general description of the project see \ref sdhlibrary_cpp_dox_general "general project information".
#//!
#//! \section sdhlibrary_cpp_makefile_variables Makefile variables
#//!   The variables defined here state project specific settings which are
#//!   then used by the goals and/or by the included, more generic sub makefiles
#//!   like:
#//!   - \ref Makefile-common "Makefile-common"
#//!   - \ref Makefile-doc    "Makefile-doc"
#//!   - \ref Makefile-rules  "Makefile-rules"
#//! 
#//! \section sdhlibrary_cpp_makefile_targets Makefile targets
#//!   - \b \c all : generate everything
#//!     - \b \c build  : generate library and demo programs
#//!     - \b \c doc    : generate all documentation
#//!   - \b \c clean    : clean up generated program files, but not TAGS or doxygen doc
#//!   - \b \c mrproper : clean up all generated files, including TAGS and doxygen doc
#//!   - \b \c tags     : generate emacs TAGS file
#//!   - \b \c dist     : create a distribution
#//!     - \b \c dist_only : create a distribution without regenerating doc or plot
#//!
#//! \section sdhlibrary_cpp_makefile_links Links
#//!   - The online documentation for \c gnu \c make can be found at
#//!     <a href="http://www.gnu.org/software/make/manual/make.html">
#//!     http://www.gnu.org/software/make/manual/make.html</a>
#//!  
#//! \section sdhlibrary_cpp_makefile_copyright Copyright
#//!
#//!  Copyright (c) 2007 SCHUNK GmbH & Co. KG
#//!
#//!  <HR>
#//!  \internal
#//!
#//!    \subsection sdhlibrary_cpp_makefile_details SVN related, detailed file specific information:
#//!      $LastChangedBy: Osswald2 $
#//!      $LastChangedDate: 2008-10-14 16:26:29 +0200 (Di, 14 Okt 2008) $
#//!      \par SVN file revision:
#//!        $Id: Makefile 3702 2008-10-14 14:26:29Z Osswald2 $
#//!
#//!  \subsection sdhlibrary_cpp_makefile_changelog Changelog of this file:
#//!      \include Makefile.log
#//!
#=======================================================================
#//! \cond ignore_me   doxygen cannot parse Makefiles, so just ignore it

.DEFAULT_GOAL := all

########################################################################
# first some variables

#-------------------
# Settings for the end-user: adjust these to your needs

## Choose wether to build a static (*.a) or a shared (*.so/*.dll) library:
export LIBRARY_TYPE=static
#export LIBRARY_TYPE=shared

## Choose wether to generate a SDHLibrary with support for the ESD CAN interface (1) or not (0)
export WITH_ESD_CAN=1

ifeq (${WITH_ESD_CAN},1)
  ## If the ESD CAN library should be used and is not in a standard path then
  ## - add the path to its header (ntcan.h) to EXTRAINCDIRS and
  ## - add the path to its library (ntcan.lib on windows/cygwin, ntcan.a/ntcan.so on linux) to EXTRALDFLAGS
  ## - add the name of the library (usually ntcan) to EXTRALIBS (without -l prefix and .a/.lib suffix)
  export EXTRACPPFLAGS += -D WITH_ESD_CAN=1
  export EXTRAINCDIRS += /cygdrive/c/Programme/ESD/CAN/SDK/include
  export EXTRALDFLAGS += -L/cygdrive/c/Programme/ESD/CAN/SDK/lib/vc/i386
  export EXTRALIBS    += ntcan
else
  export EXTRACPPFLAGS += -D WITH_ESD_CAN=0
endif

## The name of the library to generate 
## (without prefix (like "lib") or suffix (like ".a", ".so", ".dll"))
export SDHLIBRARY_NAME=SDHLibrary-CPP

## Prefix for installation:
export INSTALL_PREFIX=/usr/local

## Choose the directory where to install header files
export INSTALL_DIR_INC=${INSTALL_PREFIX}/include/sdh

## Choose the directory where to install the generated library
export INSTALL_DIR_LIB=${INSTALL_PREFIX}/lib

## Choose the directory where to install the generated (demo) programs
export INSTALL_DIR_BIN=${INSTALL_PREFIX}/bin

## Choose where to install the generated doxygen documentation
## (set to empty to not install doc)
export INSTALL_DIR_DOC=/usr/share/doc/${SDHLIBRARY_NAME}

## Choose where the firmware sources are kept
export FIRMWARE_DIR = ~/SCHUNK/SDH2

#-------------------

########################################################################
# target system specifics

## define a variable OSNAME to hold the name of the Operating System:
#  - cygwin for cygwin/windows
#  - linux for Linux (SuSE, debian, Ubuntu, ...)

ifeq (${OSTYPE},cygwin) 
 export OSNAME=cygwin
 export EXTRACPPFLAGS += -Wno-unknown-pragmas -D OSNAME_CYGWIN=1
endif
# SuSE reports OSTYPE as "linux"
ifeq (${OSTYPE},linux) 
 export OSNAME=linux
endif
# ubuntu/debian reports OSTYPE as "linux-gnu"
ifeq (${OSTYPE},linux-gnu) 
 export OSNAME=linux
endif

ifeq (${OSNAME},linux)
 export EXTRACPPFLAGS += -D OSNAME_LINUX=1
endif

# Sometimes (debian) OSTYPE is set but is not exported and is thus not
# available for make.

ifndef OSTYPE
 ${warning !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!}
 ${warning OSTYPE is not set! }
 ${warning The shell environment variable OSTYPE should be set and exported!}
 ${warning !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!}
 ${error   Please do an "export OSTYPE" and try again}
 export OSTYPE=linux
endif


#
########################################################################


# Path and name of the header file containing the 
# PROJECT_NAME and PROJECT_RELEASE defines (see Makefile-common).
export RELEASE_FILE=./sdh/release.h


# extract the setting for SDH_USE_NAMESPACE from sdhlibrary_settins
export SDH_USE_NAMESPACE=${shell cat ./sdh/sdhlibrary_settings.h | grep -e "\#define *SDH_USE_NAMESPACE" | sed "s/\#define *SDH_USE_NAMESPACE *\([0-9]\) */\1/" }

#----------------------------
# some directories:


ifeq (${DIST_BASE},)
DIST_BASE=../dist
endif

ifeq (${DIST_DIR},)
DIST_DIR=${DIST_BASE}/${PROJECT_NAME}-${PROJECT_RELEASE}
endif


# Path to common stuff like sub-makefiles
export COMMON_DIR=${shell if [ -d "../../../common" ]; then \
	             echo "../../../common";\
		   else \
	             echo "./" ; \
	           fi}

# Object files directory
OBJDIR = ./obj

# Binary files directory
BINDIR = ./bin

# Library files directory
LIBDIR = ./bin

# Dependency files directory
DEPDIR = ./.dep

# Documentation files directory
DOCDIR = ./doc

#
########################################################################




########################################################################
# now the goals

# list of sub directories with code
SUBDIRS = sdh demo


# Default goal: Make all (programs and documentation)
.PHONY: all
all: build doc


# Build the library
.PHONY: build
build: TARGET=build
build: recursive

tstinfo: TARGET=tstinfo
tstinfo: recursive

# Create and install generated files 
.PHONY: install
# let sub-makefiles install their part first:
install: TARGET=install
install: recursive 
#	# then install docu:
	@echo "installing documentation in ${INSTALL_DIR_DOC}"
	@if [ "${INSTALL_DIR_DOC}" != "" ]; then \
	  install -d ${INSTALL_DIR_DOC} ; \
	  cp -a ${DOCDIR}/*.html ${DOCDIR}/${SDHLIBRARY_NAME}-*.pdf ${DOCDIR}/external/ \
	    ${INSTALL_DIR_DOC} ; \
	  if [ -e "${DOCDIR}/internal/" ]; then \
	    cp -a ${DOCDIR}/internal/ \
	      ${INSTALL_DIR_DOC} ; \
	  fi ; \
	fi  

# uninstall previously installed files 
.PHONY: uninstall
# let sub-makefiles uninstall their part first:
uninstall: TARGET=uninstall
uninstall: recursive
#	# then uninstall docu:
	@echo "uninstalling documentation from ${INSTALL_DIR_DOC}"
	@if [ "${INSTALL_DIR_DOC}" != "" ]; then \
	  rm -rf ${INSTALL_DIR_DOC} ; \
	fi  


# Clean up generated files but not doc or tags.
.PHONY: clean
clean: TARGET=clean
clean: recursive


# Clean up generated files including doc and tags.
.PHONY: mrproper
mrproper: TARGET=mrproper
mrproper: clean doc_clean
mrproper: recursive


#---------------------
# Make a distribution


## quick and dirty hack to read the PROJECT_RELEASE for the python library
PYTHON_PROJECT_RELEASE = ${shell pushd ../python/sdh &> /dev/null; python  -c "import release ; print release.PROJECT_RELEASE"; popd &> /dev/null }


.PHONY: dist
dist:	build doc dist_only

# Make distribution of already generated files only 
# (usefull since docu is always regenerated and is rather slow)
dist_only: 
#	# regenerate distribution directory
#	rm -rf ${DIST_DIR}
	mkdir -p ${DIST_DIR}
	mkdir -p ${DIST_DIR}/cpp/
	cp -a README.txt CHANGELOG.txt   ${DIST_DIR}/cpp/

	mkdir -p ${DIST_DIR}/cpp/sdh
	echo -e "This directory contains the C++ source and header files of SDHLibrary-CPP\r" > ${DIST_DIR}/cpp/sdh/README.txt

	mkdir -p ${DIST_DIR}/cpp/demo
	echo -e "This directory contains the C++ source files of demo programs using the SDHLibrary-CPP\r" > ${DIST_DIR}/cpp/demo/README.txt

	mkdir -p ${DIST_DIR}/cpp/bin
	echo -e "This directory contains the user-compiled C++ library and demo programs of SDHLibrary-CPP\r" > ${DIST_DIR}/cpp/bin/README.txt

	mkdir -p ${DIST_DIR}/cpp/doc
	echo -e "This directory contains the pre-generated doxygen documentation\r" > ${DIST_DIR}/cpp/doc/README.txt
	mkdir -p ${DIST_DIR}/cpp/doc/external

##	mkdir -p ${DIST_DIR}/cpp/precompiled
##	echo -e "This directory contains the precompiled C++ library and demo programs of SDHLibrary-CPP\r" > ${DIST_DIR}/cpp/precompiled/README.txt

##	mkdir -p ${DIST_DIR}/doc
##	echo -e "This directory contains the documentation of SDHLibrary-*\r"  > ${DIST_DIR}/doc/README.txt
##	echo -e "  - Pregenerated pdf and html files are located here\r"      >> ${DIST_DIR}/doc/README.txt
##	echo -e "  - Generated pdf files are located here too\r"              >> ${DIST_DIR}/doc/README.txt
##	echo -e "  - Generated html files are located in sub directories\r"   >> ${DIST_DIR}/doc/README.txt

##	mkdir -p ${DIST_DIR}/doc/${PROJECT_NAME}-${PROJECT_RELEASE}/

#	# The release
	echo -e "Name: ${PROJECT_NAME}\r\nRelease: ${PROJECT_RELEASE}\r" > ${DIST_DIR}/cpp/RELEASE-${PROJECT_RELEASE}.txt

#	# The actual library:
##	if [ "${LIBDIR}" != "${BINDIR}" ]; then   \
##	  cp -a ${LIBDIR}/*  ${DIST_DIR}/cpp/precompiled/ ;\
##	fi
##	cp -a ${BINDIR}/*  ${DIST_DIR}/cpp/precompiled/

#	# The sources:
	for d in ${SUBDIRS} ; do                                     \
	  for f in $$d/*.cpp $$d/*.h $$d/Makefile ; do               \
	    if [ -e $$f ]; then   		                     \
	      cp -a --dereference $$f   ${DIST_DIR}/cpp/$$d/ ;       \
	    fi ;                                       		     \
	  done ;                                       		     \
	done
	cp -a Makefile  ${DIST_DIR}/cpp/ 
#	# Additional Makefiles from common/
	cp -a ${COMMON_DIR}/Makefile-common ${COMMON_DIR}/Makefile-doc ${COMMON_DIR}/Makefile-cygwin ${COMMON_DIR}/Makefile-rules ${COMMON_DIR}/Makefile-subdir ${DIST_DIR}/cpp/ 

#       # Doxygen stuff so end user can generate doxygen doku	
##	cp -a *.dox Doxyfile                            ${DIST_DIR}/cpp/ 
	cat doc/index-external.html \
	  | sed "s/\$${PROJECT_DATE}/${PROJECT_DATE}/g" \
	  | sed "s/\$${PROJECT_RELEASE}/${PROJECT_RELEASE}/g" \
	  | sed "s/\$${PYTHON_PROJECT_RELEASE}/${PYTHON_PROJECT_RELEASE}/g" \
	  > ${DIST_DIR}/cpp/doc/index.html

#	# The pregenerated documentation
##	cp -a doc/${PROJECT_NAME}-external.pdf  ${DIST_DIR}/doc/${PROJECT_NAME}-${PROJECT_RELEASE}.pdf
##	cp -a doc/external/html                 ${DIST_DIR}/doc/${PROJECT_NAME}-${PROJECT_RELEASE}/
	cp -a doc/${PROJECT_NAME}-external.pdf  ${DIST_DIR}/cpp/doc/
	cp -a doc/external/html                 ${DIST_DIR}/cpp/doc/external

#	# general documentation:
	mkdir -p ${DIST_DIR}/doc
	cp -a ~/SCHUNK/SDH2/software/doc/SDH-2.0_Datasheet.pdf ${DIST_DIR}/doc/
	cp -a ~/SCHUNK/SDH2/software/doc/SDH2_kinematics.pdf ${DIST_DIR}/doc/
	cp -a ~/SCHUNK/SDH2/software/doc/SDH2_wiring.pdf ${DIST_DIR}/doc/


#       # Visual C++ project stuff
	mkdir -p ${DIST_DIR}/cpp/vcc
	cp -a vcc/*.vcproj vcc/*.sln vcc/README.txt vcc/*.c vcc/*.h ${DIST_DIR}/cpp/vcc
	# FIXME: copying all first and deleting unwanted afterwards is somewhat suboptimal...
	rm -f ${DIST_DIR}/cpp/vcc/*-intern.* 
	 
	

#	# DSA-explorer
	mkdir -p ${DIST_DIR}/tool
	cp -a ~/Dokumentation/Weiss\ Robotics/dsa_explorer-3.1.131.msi ${DIST_DIR}/tool/

#	# firmware
	mkdir -p ${DIST_DIR}/firmware
	cp -a ${FIRMWARE_DIR}/sdhflash/dist/sdhflash-*.win32.exe   ${DIST_DIR}/firmware/
	cp -a ${FIRMWARE_DIR}/sdhflash/dist/sdhflash-*.tar.bz2     ${DIST_DIR}/firmware/

#	# create index.html file for the whole distribution
	cat ./doc/index-overview.html \
	  | sed "s/\$${PROJECT_DATE}/${PROJECT_DATE}/g" \
	  | sed "s/\$${PROJECT_RELEASE}/${PROJECT_RELEASE}/g" \
	  | sed "s/\$${PYTHON_PROJECT_RELEASE}/${PYTHON_PROJECT_RELEASE}/g" \
	  > ${DIST_DIR}/index.html

#	# autostart CD that shows the index.html file automatically
#	#  sources: http://aktuell.de.selfhtml.org/artikel/programmiertechnik/autorun/index.htm
#	#           http://www.pro-support.de/htmlauto.shtml
	cp -a doc/AUTORUN.INF doc/shelexec.exe doc/SCHUNK.ICO  ${DIST_DIR}/
	attrib +H ${DIST_DIR}/shelexec.exe

#	# create compressed archive
#	cd ${DIST_BASE} && tar czf ${PROJECT_NAME}-${PROJECT_RELEASE}.tar.gz ${PROJECT_NAME}-${PROJECT_RELEASE}/
#
#
########################################################################

.PHONY:
tst: 
	set > set.eclipse
	which epstopdf
	which perl
	bash -c "which epstopdf"
	bash -c "which perl"
	epstopdf 
######################################################################
#

# some common settings and targets are defined separately to keep this
# main Makefile more concise
include ${COMMON_DIR}/Makefile-common

# The variables and targets to generate documentation are defined
# separately to keep this main Makefile clearly laid out.
include ${COMMON_DIR}/Makefile-doc

# The variables and targets to call make recursively are defined
# separately to keep this main Makefile clearly laid out.
include ${COMMON_DIR}/Makefile-subdir

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
