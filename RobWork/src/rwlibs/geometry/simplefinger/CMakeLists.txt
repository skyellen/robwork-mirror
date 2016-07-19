ADD_LIBRARY(SimpleFinger SimpleFinger.cpp SimpleFinger.hpp)

ADD_LIBRARY(simplefinger.rwplugin MODULE SimpleFingerPlugin.cpp SimpleFingerPlugin.hpp)
TARGET_LINK_LIBRARIES(simplefinger.rwplugin SimpleFinger rw_csg rw)
