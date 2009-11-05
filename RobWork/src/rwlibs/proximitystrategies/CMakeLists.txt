
set(OptionalFiles)
if (RW_HAVE_PQP)
  list(APPEND OptionalFiles "./ProximityStrategyPQP.cpp")
  list(APPEND OptionalFiles_hpp "./ProximityStrategyPQP.hpp")
else ()
  message("rw_proximitystrategies: Not building PQP interface.")
endif ()
if (RW_HAVE_YAOBI)
  list(APPEND OptionalFiles "./ProximityStrategyYaobi.cpp")
  list(APPEND OptionalFiles_hpp "./ProximityStrategyYaobi.hpp")
else ()
  message("rw_proximitystrategies: Not building Yaobi interface.")
endif ()

SET(SRC_FILES
  ${RW_PROXIMITYSTRATEGIES_USER_FILES}
  ProximityStrategyFactory.cpp
  ${OptionalFiles}
)

SET(SRC_FILES_HPP
  ProximityStrategyFactory.hpp
  ${OptionalFiles_hpp}
)

add_library(rw_proximitystrategies ${SRC_FILES} ${SRC_FILES_HPP})

INSTALL(TARGETS rw_proximitystrategies DESTINATION ${RW_LIB_INSTALL_DIR})
#INSTALL(FILES ${SRC_FILES_HPP} DESTINATION include/rwlibs/proximitystrategies)
