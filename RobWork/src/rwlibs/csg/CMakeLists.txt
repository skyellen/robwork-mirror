SET(SUBSYS_NAME rw_csg)
SET(SUBSYS_DESC "A module for doing constructive solid geometry")
SET(SUBSYS_DEPS rw rw_csgjs)

set(build TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)
    SET(SRC_FILES
      CSGModel.cpp
      CSGModelFactory.cpp
      CSGConvert.cpp
      CSGUtil.cpp
    )
    
    SET(SRC_FILES_HPP
      CSGModel.hpp
      CSGModelFactory.hpp
      CSGConvert.hpp
      CSGUtil.hpp
    )
    
    RW_ADD_LIBRARY(${SUBSYS_NAME} csg ${SRC_FILES} ${SRC_FILES_HPP})
    RW_ADD_INCLUDES(csg "rwlibs/csg" CSGModel.hpp CSGConvert.hpp)
    TARGET_LINK_LIBRARIES(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS})

	IF(CMAKE_VERSION VERSION_GREATER 3.3)
		SET_TARGET_PROPERTIES(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
	ENDIF()
ENDIF()
