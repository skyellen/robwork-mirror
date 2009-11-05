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

  
INSTALL(TARGETS pqp DESTINATION ${RW_LIB_INSTALL_DIR})
INSTALL(FILES ${PQP_HPP_FILES} DESTINATION "${RW_EXT_INSTALL_DIR}/pqp/")
  