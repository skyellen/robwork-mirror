


add_library(
  rw_proximitystrategies
  ${RW_PROXIMITYSTRATEGIES_USER_FILES}
  ./CollisionStrategyFactory.cpp
  ./ProximityStrategyOpcode.cpp
  ./ProximityStrategyPQP.cpp
  ./ProximityStrategyYaobi.cpp
)
