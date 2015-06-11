SET(SUBSYS_NAME rw_mathematica)
set(SUBSYS_DESC "Mathematica interface for RobWork")
SET(SUBSYS_DEPS rw)

SET(build ON)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${RW_HAVE_MATHEMATICA})
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
RW_ADD_DOC(${SUBSYS_NAME})

IF( build )
    SET(SRC_FILES
      Mathematica.cpp
      
      # Packets
      EnterExpressionPacket.cpp
      EnterTextPacket.cpp
      EvaluatePacket.cpp
      InputNamePacket.cpp
      MessagePacket.cpp
      OutputNamePacket.cpp
      ReturnExpressionPacket.cpp
      ReturnPacket.cpp
      ReturnTextPacket.cpp
      TextPacket.cpp
      
      # Functions
      FactorInteger.cpp
      Image.cpp
      List.cpp
      ListPlot.cpp
      RawArray.cpp
      Rule.cpp
      ToExpression.cpp
    )
    
    SET(SRC_FILES_HPP
      Mathematica.hpp
      
      # Packets
      EnterExpressionPacket.hpp
      EnterTextPacket.hpp
      EvaluatePacket.hpp
      InputNamePacket.hpp
      MessagePacket.hpp
      OutputNamePacket.hpp
      ReturnExpressionPacket.hpp
      ReturnPacket.hpp
      ReturnTextPacket.hpp
      TextPacket.hpp
      
      # Functions
      FactorInteger.hpp
      Image.cpp
      List.hpp
      ListPlot.hpp
      RawArray.hpp
      Rule.hpp
      ToExpression.hpp
    )
    	
    RW_ADD_LIBRARY(${SUBSYS_NAME} mathematica ${SRC_FILES} ${SRC_FILES_HPP})
    RW_ADD_INCLUDES(mathematica "rwlibs/mathematica" ${SRC_FILES_HPP}) 
    TARGET_LINK_LIBRARIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
ENDIF()