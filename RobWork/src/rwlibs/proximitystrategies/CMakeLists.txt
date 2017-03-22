SET(SUBSYS_NAME rw_proximitystrategies)
set(SUBSYS_DESC "Collection of proximity strategies!")
SET(SUBSYS_DEPS rw)

SET(build TRUE)
SET(DEFAULT TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT})
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS pqp yaobi)
#RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} )
RW_ADD_DOC(${SUBSYS_NAME})


IF(build)
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

    IF (RW_HAVE_BULLET)
      LIST(APPEND OptionalFiles "./ProximityStrategyBullet.cpp")
      LIST(APPEND OptionalFiles_hpp "./ProximityStrategyBullet.hpp")
    ELSE ()
      MESSAGE(STATUS "rw_proximitystrategies: Not building Bullet interface since Bullet is disabled!")
    ENDIF ()

    IF (RW_HAVE_FCL)
      LIST(APPEND OptionalFiles "./ProximityStrategyFCL.cpp")
      LIST(APPEND OptionalFiles_hpp "./ProximityStrategyFCL.hpp")
    ELSE ()
      MESSAGE(STATUS "rw_proximitystrategies: Not building FCL interface since FCL is disabled!")
    ENDIF ()
    
    SET(SRC_FILES
      ${RW_PROXIMITYSTRATEGIES_USER_FILES}
      ${OptionalFiles}
      ProximityStrategyFactory.cpp
    )
    
    SET(SRC_FILES_HPP
      ${OptionalFiles_hpp}
      ProximityStrategyFactory.hpp
    )
    
    RW_ADD_LIBRARY(${SUBSYS_NAME} proximitystrategies ${SRC_FILES} ${SRC_FILES_HPP})
    RW_ADD_INCLUDES(proximitystrategies "rwlibs/proximitystrategies" ${SRC_FILES_HPP}) 
    TARGET_LINK_LIBRARIES(${SUBSYS_NAME} ${SUBSYS_DEPS} ${YAOBI_LIBRARIES} ${PQP_LIBRARIES} ${BULLET_LIBRARIES} ${FCL_LIBRARIES})
    TARGET_INCLUDE_DIRECTORIES(${SUBSYS_NAME} PRIVATE ${FCL_INCLUDE_DIRS})
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})

    ADD_LIBRARY(${SUBSYS_NAME}.rwplugin MODULE ProximityStrategyPlugin.cpp)
    TARGET_LINK_LIBRARIES(${SUBSYS_NAME}.rwplugin ${SUBSYS_NAME})
ENDIF()
