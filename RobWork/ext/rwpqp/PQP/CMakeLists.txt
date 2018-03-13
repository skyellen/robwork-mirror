cmake_minimum_required(VERSION 2.8)
if(POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif()

project(PQP)

SET(PQP_HPP_FILES
    Build.h
    BV.h
    BVTQ.h
    GetTime.h
    MatVec.h
    OBB_Disjoint.h
    PQP_Compile.h
    PQP_Internal.h
    PQP.h
    RectDist.h
    Tri.h
    TriDist.h    
)


add_library(pqp
  PQP.cpp
  Build.cpp
  BV.cpp
  TriDist.cpp
  )

IF(CMAKE_VERSION VERSION_GREATER 3.3)
	SET_TARGET_PROPERTIES(pqp PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
ENDIF()

  
INSTALL(TARGETS pqp DESTINATION ${LIB_INSTALL_DIR})
INSTALL(FILES ${PQP_HPP_FILES} DESTINATION "${INCLUDE_INSTALL_DIR}/ext/pqp/")
