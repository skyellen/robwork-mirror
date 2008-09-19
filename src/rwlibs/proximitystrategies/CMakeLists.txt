
find_library(PQP_LIB pqp)
find_library(OPCODE_LIB opcode)
find_library(YAOBI_LIB yaobi)

add_library(
  rw_proximitystrategies
  ${RW_PROXIMITYSTRATEGIES_USER_FILES}
  ./CollisionStrategyFactory.cpp
  ./ProximityStrategyOpcode.cpp
  ./ProximityStrategyPQP.cpp
  ./ProximityStrategyYaobi.cpp
)
