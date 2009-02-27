
set(OptionalFiles)
if (RW_HAVE_PQP)
  list(APPEND OptionalFiles "./ProximityStrategyPQP.cpp")
else ()
  message("rw_proximitystrategies: Not building PQP interface.")
endif ()
if (RW_HAVE_YAOBI)
  list(APPEND OptionalFiles "./ProximityStrategyYaobi.cpp")
else ()
  message("rw_proximitystrategies: Not building Yaobi interface.")
endif ()

add_library(
  rw_proximitystrategies
  CollisionStrategyFactory.hpp
  ${RW_PROXIMITYSTRATEGIES_USER_FILES}
  ${OptionalFiles}
  )
