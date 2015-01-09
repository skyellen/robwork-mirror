SET(SUBSYS_NAME universalrobots )
SET(SUBSYS_DESC "Driver for universal robot " )
SET(SUBSYS_DEPS  )

# make sure that SDH can be found
SET(build TRUE)

set(DEFAULT TRUE)
set(REASON) 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND( build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} )
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
   
    SET(SRC_CPP UniversalRobotsData.cpp UniversalRobotsRTLogging.cpp URCommon.cpp URPrimaryInterface.cpp URCallBackInterface.cpp)
    SET(SRC_HPP UniversalRobotsData.hpp UniversalRobotsRTLogging.hpp URCommon.hpp URPrimaryInterface.hpp URCallBackInterface.hpp)
    

      file(READ urscript.ur URSCRIPT)


      #std::string(\"\")+\n\"") 
      #set(header "#include<sstream>\n std::stringstream UR_SCRIPT; UR_SCRIPT<< \"")
      set(header "const std::string NEW_LINE= \"\\n\"; const std::string QUOTATION = \"\\\"\"; std::string UR_SCRIPT = \"")
      string(REGEX REPLACE "\"" "\"+QUOTATION+\"" FILEVAR21 ${URSCRIPT})
      string(REGEX REPLACE "\\\n" "\" + NEW_LINE  + \n\"" FILEVAR21 ${FILEVAR21})  
      # set(end "\");")
      set(end "\";")

      file(WRITE urscript.hpp "${header}${FILEVAR21}${end}")
      #  MESSAGE("${header}${FILEVAR21}${end}")

    RW_ADD_LIBRARY(rwhw_universalrobots universalrobots ${SRC_CPP} ${SRC_HPP})
    RW_ADD_INCLUDES(universalrobots "rwhw/universalrobots" ${SRC_HPP} urscript.hpp)

    SET( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_universalrobots;")
ELSE()
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")
ENDIF()
