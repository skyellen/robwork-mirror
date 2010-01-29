SET(SRC_CPP CubePort.cpp Cube.cpp PCubeProtocol.cpp)
SET(SRC_HPP CubePort.hpp Cube.hpp PCubeProtocol.hpp)

ADD_LIBRARY(rwhw_pcube ${SRC_CPP} ${SRC_HPP})
INSTALL(TARGETS rwhw_pcube DESTINATION ${RWHW_LIB_INSTALL_DIR})