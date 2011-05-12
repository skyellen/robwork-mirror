
#FIND_FILE(SDH "sdh/sdh.h" ${SDH_INCLUDE_DIR})
FIND_PATH(SDH_INCLUDE_DIR sdh/sdh.h
   # TODO paths?
   ${ROBWORKHARDWARE_ROOT}/ext/sdh2/
   /usr/include
   /include
)

FIND_LIBRARY(SDH_LIBRARY NAMES SDH PATHS "${ROBWORKHARDWARE_ROOT}/ext/sdh2/libs/")

IF (SDH_INCLUDE_DIR AND SDH_LIBRARY)
    #MESSAGE(STATUS "SDH: Found!")
    SET (SDH_FOUND 1)
ELSE()
    #MESSAGE(STATUS "SDH: Not found!")
ENDIF()
