ADD_EXECUTABLE(test test.cpp)
TARGET_LINK_LIBRARIES(test PluginTest)

# Generate file with path to the plugin
FILE(GENERATE OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/path.hpp" INPUT ${CMAKE_CURRENT_SOURCE_DIR}/path.hpp.in)