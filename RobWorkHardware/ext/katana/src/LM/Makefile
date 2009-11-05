#===========================================================================
#
#Makefile --
#
#
#Tiziano Mueller <tiziano.mueller@neuronics.ch>
#   
#   Copyright (c) 2005
#   
#   Neuronics AG <www.neuronics.ch>
#   Switzerland
#
#1.0 / 26. Feb 05 (tm)
#- from scratch
#
#
#===========================================================================

CXX		=	g++
CXXFLAGS	=	-g -O2

MAJOR_VERSION	=	3.3
MINOR_VERSION	=	03

TARGETS		=	alib solib

LIBNAME = libKNI_LM
SOURCES = lmBase.cpp

LIBNAME_A  = $(LIBNAME).a
LIBNAME_SO = $(LIBNAME).so.$(MAJOR_VERSION).$(MINOR_VERSION)

SLCFLAGS=-fPIC
SLLDFLAGS=-shared

INCLUDE_DIR = ../../include
TARGET_DIR  = ../../lib/linux

.SILENT:

.SUFFIXES: .cpp.o

.cpp.o:
	echo -n "Compiling: $@"
	if $(CXX) $(CXXFLAGS) -I$(INCLUDE_DIR) -c $< 2>.clog; \
	then echo " [ok]"; \
	else echo " [failed]"; \
	     cat .clog; \
	fi

all: $(TARGETS)

$(TARGET_DIR)/$(LIBNAME_A): $(SOURCES:.cpp=.o)
	echo -n "Building: $@"
	if $(AR) $(ARFLAGS) $@ $(SOURCES:.cpp=.o) 1>.alog 2>.alog; \
	then echo " [ok]"; \
	else echo " [failed]"; \
	     cat .alog; \
	fi	

$(TARGET_DIR)/$(LIBNAME_SO): $(SOURCES:.cpp=.o)
	echo -n "Building: $@"
	if $(CXX) $(CXXFLAGS) $(SLLDFLAGS) -o $@ $(SOURCES:.cpp=.o) 2>.slog; \
	then echo " [ok]"; \
	     cat .slog; \
	fi

alib: $(TARGET_DIR)/$(LIBNAME_A)

solib: $(TARGET_DIR)/$(LIBNAME_SO)

clean:	
	echo "Cleaning up"
	rm -rf *.o *~ ./.?log $(TARGET_DIR)/$(LIBNAME_A) $(TARGET_DIR)/$(LIBNAME_SO)
