# makefile for the math library

#RobWork Root relative to the current library
ROOT = ../..
#Include makefile with common settings
include $(ROOT)/ext/makefile.setup

# Opcode workaround
CPPFLAGS += -fno-strict-aliasing



#Automatically generate a list with the .cpp files
#CPP_FILES = $(shell find -name "*.cpp")

#Manually edited file list
CPP_FILES = \
			StdAfx.cpp \
			OPC_Common.cpp \
			OPC_MeshInterface.cpp \
			OPC_AABBTree.cpp \
			OPC_AABBCollider.cpp \
			IceUtils.cpp \
			IceContainer.cpp \
			IceRevisitedRadix.cpp \
			IceRandom.cpp \
			IcePoint.cpp \
			IceHPoint.cpp \
			IceMatrix3x3.cpp \
			IceMatrix4x4.cpp \
			IcePlane.cpp \
			IceRay.cpp \
			IceIndexedTriangle.cpp \
			IceTriangle.cpp \
			IceAABB.cpp \
			IceOBB.cpp \
			OPC_BaseModel.cpp \
			OPC_BoxPruning.cpp \
			OPC_Collider.cpp \
			OPC_HybridModel.cpp \
			OPC_LSSCollider.cpp \
			OPC_Model.cpp \
			OPC_OBBCollider.cpp \
			OPC_OptimizedTree.cpp \
			OPC_Picking.cpp \
			OPC_PlanesCollider.cpp \
			OPC_RayCollider.cpp \
			OPC_SphereCollider.cpp \
			OPC_SweepAndPrune.cpp \
			OPC_TreeBuilders.cpp \
			OPC_TreeCollider.cpp \
			OPC_VolumeCollider.cpp \
			OPC_AABBCollider.cpp \
			OPC_AABBTree.cpp

#Construct list with .d and .o files
D_FILES = $(CPP_FILES:.cpp=.d)
O_FILES = $(CPP_FILES:.cpp=.o)

.PHONY: all
all : $(ROOT)/libs/libopcode.a

$(ROOT)/libs/libopcode.a : $(O_FILES)
	ar crs $@ $^

%.d : %.cpp
	$(CPP) -MM $(CPPFLAGS) -MT $*.o $< > $@
 
.PHONY : clean
clean :
	rm -rf $(D_FILES)
	rm -rf $(O_FILES)
	rm -rf *~

-include $(D_FILES)
