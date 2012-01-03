
#FIND_FILE(SDH "sdh/sdh.h" ${SDH_INCLUDE_DIR})
FIND_PATH(SDH_INCLUDE_DIR sdh/sdh.h
   # TODO paths?
   ${RWHW_ROOT}/ext/sdh2/
   /usr/include
   /include
)

FIND_LIBRARY(SDH_LIBRARY 
             NAMES "sdh" 
             PATHS "${RWHW_ROOT}/ext/sdh2/libs/")

#MESSAGE("SDH LIB ${SDH_LIBRARY}")
#MESSAGE("SDH INCLUDE ${SDH_INCLUDE_DIR}")

IF (SDH_INCLUDE_DIR AND SDH_LIBRARY)
    #MESSAGE(STATUS "SDH: Found!")
    SET (SDH_FOUND TRUE)
ELSE()
    #MESSAGE(STATUS "SDH: Not found!")
ENDIF()
