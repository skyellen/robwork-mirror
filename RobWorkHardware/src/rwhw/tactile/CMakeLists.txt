SET(SRC_CPP )
SET(SRC_HPP )

LIST(APPEND SRC_CPP DSACON32.cpp TactileMatrix.cpp TactileMaskMatrix.cpp ConvertUtil.cpp)
LIST(APPEND SRC_HPP DSACON32.hpp TactileMatrix.hpp TactileMaskMatrix.hpp ConvertUtil.hpp)

ADD_LIBRARY(rwhw_tactile ${SRC_CPP} ${SRC_HPP})
INSTALL(TARGETS rwhw_tactile DESTINATION ${RWHW_LIB_INSTALL_DIR})