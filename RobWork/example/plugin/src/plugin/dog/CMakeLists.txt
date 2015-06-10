ADD_LIBRARY(Dog Dog.cpp Dog.hpp)

ADD_LIBRARY(dog.rwplugin MODULE DogPlugin.cpp DogPlugin.hpp)
TARGET_LINK_LIBRARIES(dog.rwplugin Dog ${ROBWORK_LIBRARIES})
