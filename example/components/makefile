# makefile for the src/components library
ROOT = ../../..

include $(ROOT)/src/makefile.setup

# Set library name.
LIBRARY_NAME = rw_components

# Call the makefile in the subdirectories with the same argument.
.PHONY : all
all %: 
	make $(JOBS) -C qpcontroller/ $@
	make $(JOBS) -C velocityprofile/ $@
