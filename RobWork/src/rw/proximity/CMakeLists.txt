SET(FILES_CPP
  ProximityCache.cpp
  ProximityData.cpp
  ProximityStrategyData.cpp
  
  CollisionStrategy.cpp
  CollisionSetup.cpp
  CollisionToleranceStrategy.cpp
  ProximityStrategy.cpp
  CollisionDetector.cpp
  DistanceStrategy.cpp
  DistanceMultiStrategy.cpp
  #DistanceThresholdStrategy.cpp
  DistanceCalculator.cpp
  ProximityModel.cpp
  
  ProximityFilterStrategy.cpp
  ProximitySetup.cpp
  ProximitySetupRule.cpp
  BasicFilterStrategy.cpp
  #ProximityStrategyFactory.cpp
  Raycaster.cpp
  
  rwstrategy/BinaryBVTree.cpp
  rwstrategy/BinaryIdxBVTree.cpp
  
  rwstrategy/BVTree.cpp
  rwstrategy/BVTreeFactory.cpp
  rwstrategy/ProximityStrategyRW.cpp
  rwstrategy/BVTreeCollider.cpp
  rwstrategy/BVTreeColliderFactory.cpp
  rwstrategy/BVTreeToleranceCollider.cpp
  rwstrategy/OBVTreeDFSCollider.cpp
)

SET(FILES_HPP
  CollisionStrategy.hpp
  CollisionSetup.hpp
  CollisionToleranceStrategy.hpp
  ProximityStrategy.hpp
  CollisionDetector.hpp
  DistanceStrategy.hpp
  DistanceMultiStrategy.hpp
  DistanceCalculator.hpp
  ProximityModel.hpp
  ProximityFilterStrategy.hpp
  ProximitySetup.hpp
  ProximitySetupRule.hpp
  BasicFilterStrategy.hpp
  #ProximityStrategyFactory.hpp  
  Raycaster.hpp
  rwstrategy/BinaryBVTree.hpp
  rwstrategy/BinaryIdxBVTree.hpp
  rwstrategy/BVTreeCollider.hpp
  rwstrategy/BVTree.hpp
  rwstrategy/BVTreeFactory.hpp
  rwstrategy/ProximityStrategyRW.hpp
  rwstrategy/BVTreeColliderFactory.hpp
  rwstrategy/BVTreeToleranceCollider.hpp
  rwstrategy/OBVTreeDFSCollider.hpp 
)

IF (RW_HAVE_PQP)
  SET(FILES_CPP ${FILES_CPP}
    rwstrategy/TreeDistanceCalc.cpp
  )
  SET(FILES_HPP ${FILES_HPP}
    rwstrategy/TreeDistanceCalc.hpp
  )
ENDIF()

SOURCE_GROUP(proximity FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")

#INSTALL(FILES ${PROXIMITY_FILES_HPP} DESTINATION "include/rw/proximity/")
