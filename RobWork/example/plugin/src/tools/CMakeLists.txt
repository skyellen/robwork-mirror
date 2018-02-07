ADD_EXECUTABLE(Animals_test test.cpp)
TARGET_LINK_LIBRARIES(Animals_test PluginTest)

# Generate file with path to the plugin
FILE(GENERATE OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/path.hpp" INPUT ${CMAKE_CURRENT_SOURCE_DIR}/path.hpp.in)