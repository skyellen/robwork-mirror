
SET(OptionalFiles)
IF (RW_HAVE_PQP)
  LIST(APPEND OptionalFiles "./ProximityStrategyPQP.cpp")
  LIST(APPEND OptionalFiles_hpp "./ProximityStrategyPQP.hpp")
ELSE ()
  MESSAGE(STATUS "rw_proximitystrategies: Not building PQP interface since PQP is disabled!")
ENDIF ()

IF (RW_HAVE_YAOBI)
  LIST(APPEND OptionalFiles "./ProximityStrategyYaobi.cpp")
  LIST(APPEND OptionalFiles_hpp "./ProximityStrategyYaobi.hpp")
ELSE ()
  MESSAGE(STATUS "rw_proximitystrategies: Not building Yaobi interface since Yaobi is disabled!")
ENDIF ()

SET(SRC_FILES
  ${RW_PROXIMITYSTRATEGIES_USER_FILES}
  ProximityStrategyFactory.cpp
  ${OptionalFiles}
)

SET(SRC_FILES_HPP
  ProximityStrategyFactory.hpp
  ${OptionalFiles_hpp}
)

ADD_LIBRARY(rw_proximitystrategies ${SRC_FILES} ${SRC_FILES_HPP})

INSTALL(TARGETS rw_proximitystrategies DESTINATION ${RW_LIB_INSTALL_DIR})
#INSTALL(FILES ${SRC_FILES_HPP} DESTINATION include/rwlibs/proximitystrategies)
