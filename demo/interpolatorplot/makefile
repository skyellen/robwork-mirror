ROOT = ../..
include $(ROOT)/demo/makefile.setup

#Automatically generate a list with the .cpp files
CPP_FILES += InterpolatorPlotDemo.cpp

#Construct list with .d and .o files
D_FILES = $(CPP_FILES:.cpp=.d)
O_FILES = $(CPP_FILES:.cpp=.o)

LDLIBS += -lglut

.PHONY: all
all : cubicx.png
	
%.png : plot.p cubic.dat line.dat
	gnuplot $<

cubic%.png : cubic.dat
    
line%.png : line.dat

%.dat : InterpolatorPlotDemo
	./InterpolatorPlotDemo

InterpolatorPlotDemo : $(ROOT)/libs/librw.a

.PHONY : clean
clean :
	rm -rf $(D_FILES)
	rm -rf $(O_FILES)
	rm -rf *~
	rm -rf EAA


-include $(D_FILES)
