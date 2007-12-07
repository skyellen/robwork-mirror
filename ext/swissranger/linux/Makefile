# Make file for the Swiss Ranger 2, Linux user-side driver.
# 26.2.04 gabriel.gruener@csem.ch
ROOT = ../../..
EXTROOT = $(ROOT)/ext
include $(EXTROOT)/makefile.setup

#CC = gcc
#CFLAGS += -Wall -O3

# Uncomment this if you want extensive debugging info
#CFLAGS += -DSWISSRANGER_DEBUG

NAME = swissranger
SOURCES = ${NAME}.c
OBJECTS = ${SOURCES:.c=.o}
HEADERS = ${SOURCES:.c=.h}
LIB_NAME = lib${NAME}.a


all: ${OBJECTS}
	${CC} -c ${CFLAGS} *.c
	ar crs $(ROOT)/libs/$(LIB_NAME) $(OBJECTS)
