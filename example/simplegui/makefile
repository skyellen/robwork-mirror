# makefile for example/vworkcell

ROOT = ../..
include $(ROOT)/example/makefile.setup

CPP_FILES = ViewerExample.cpp \
			SimpleGLViewer.cpp \
			EventListener.cpp \
			Menu.cpp \
			MenuItem.cpp \
			ArcBall.cpp 

LDLIBS +=  $(if $(MINGW), -lglut32,-lglut) -lpqp

#Construct list with .d and .o files
D_FILES = $(CPP_FILES:.cpp=.d)
O_FILES = $(CPP_FILES:.cpp=.o)

.PHONY: all
all : ViewerExample

ViewerExample : $(O_FILES)
	$(CC) $(LDFLAGS) -o $@ $^ $(LOADLIBES) $(LDLIBS) 

ViewerExample : $(ROOT)/libs/librw.a

%.d : %.cpp
	$(CPP) -MM $(CPPFLAGS) -MT $*.o $< > $@

.PHONY : clean
clean :
	$(RM) $(D_FILES)
	$(RM) $(O_FILES)
	$(RM) *~

-include $(D_FILES)
