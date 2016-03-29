INCLUDE_DIRECTORIES(${GTEST_INCLUDE_DIRS})

########################################################################
# Common
########################################################################
SET(COMMON_TEST_LIBRARIES
 ${GTEST_BOTH_LIBRARIES}
 rw
 ${XERCESC_LIBRARIES}
 ${ASSIMP_LIBRARIES}
 ${QHULL_LIBRARIES}
)

SET(COMMON_TEST_SRC
  common/CommonTest.cpp
)
ADD_EXECUTABLE( rw_common-gtest ${COMMON_TEST_SRC})       
TARGET_LINK_LIBRARIES( rw_common-gtest ${COMMON_TEST_LIBRARIES})
ADD_TEST(rw_common-gtest ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/rw_common-gtest)

########################################################################
# Loaders
########################################################################
SET(LOADERS_TEST_LIBRARIES
 ${GTEST_BOTH_LIBRARIES}
 rw
 ${XERCESC_LIBRARIES}
 ${ASSIMP_LIBRARIES}
 ${QHULL_LIBRARIES}
 ${ROBWORK_LIBRARIES}
 )

SET(LOADERS_TEST_SRC
  loaders/DOMPropertyMap.cpp
)
ADD_EXECUTABLE( rw_loaders-gtest ${LOADERS_TEST_SRC})       
TARGET_LINK_LIBRARIES( rw_loaders-gtest ${LOADERS_TEST_LIBRARIES})
ADD_TEST(rw_loaders-gtest ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/rw_loaders-gtest)
