# makefile for the src/core library
ROOT=../..
include $(ROOT)/src/makefile.setup

# Call the makefile in the subdirectories with the same argument.
.PHONY : all
all %: 
	make $(JOBS) -C algorithms/ $@
	make $(JOBS) -C drawable/ $@
	make $(JOBS) -C collisionstrategies/ $@
	make $(JOBS) -C devices/ $@
	make $(JOBS) -C io/ $@
	make $(JOBS) -C pathplanners/ $@
	make $(JOBS) -C sensors/ $@
