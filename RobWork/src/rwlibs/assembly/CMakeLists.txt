SET(SUBSYS_NAME rw_assembly)
set(SUBSYS_DESC "Assembly task and result formats and strategies.")
SET(SUBSYS_DEPS rw rw_task)

SET(DEFAULT TRUE)
SET(REASON)

SET(build TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)
    SET(SRC_FILES
      ./AssemblyControlResponse.cpp
      ./AssemblyControlStrategy.cpp
      ./AssemblyParameterization.cpp
      ./AssemblyRegistry.cpp
      ./AssemblyResult.cpp
      ./AssemblyState.cpp
      ./AssemblyTask.cpp
      ./CircularPiHControlStrategy.cpp
      ./CircularPiHParameterization.cpp
      ./PiHParameterization.cpp
      ./PiHStrategy.cpp
      ./SpiralParameterization.cpp
      ./SpiralStrategy.cpp
    )
    
    SET(SRC_FILES_HPP
      ./AssemblyControlResponse.hpp
      ./AssemblyControlStrategy.hpp
      ./AssemblyParameterization.hpp
      ./AssemblyRegistry.hpp
      ./AssemblyResult.hpp
      ./AssemblyState.hpp
      ./AssemblyTask.hpp
      ./CircularPiHControlStrategy.hpp
      ./CircularPiHParameterization.hpp
      ./PiHParameterization.hpp
      ./PiHStrategy.hpp
      ./SpiralParameterization.hpp
      ./SpiralStrategy.hpp
    )
      	  
    RW_ADD_LIBRARY(${SUBSYS_NAME} assembly ${SRC_FILES} ${SRC_FILES_HPP} ${LOADER_SRC_FILES_HPP})
    RW_ADD_INCLUDES(assembly "rwlibs/assembly" ${SRC_FILES_HPP})
    TARGET_LINK_LIBRARIES(${SUBSYS_NAME} PUBLIC rw PRIVATE rw_task)
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})

	IF(CMAKE_VERSION VERSION_GREATER 3.3)
		SET_TARGET_PROPERTIES(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
	ENDIF()
ENDIF()