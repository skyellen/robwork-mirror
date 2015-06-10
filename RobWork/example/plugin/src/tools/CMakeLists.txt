ADD_EXECUTABLE(test test.cpp)
ADD_DEPENDENCIES(test PluginTest)
TARGET_LINK_LIBRARIES(test PluginTest)
