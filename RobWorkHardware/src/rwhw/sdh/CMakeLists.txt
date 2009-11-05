
# make sure that SDH can be found
FIND_FILE(SDH "sdh/sdh.h" ${SDH_INCLUDE_DIR})
IF(NOT SDH)
    MESSAGE(STATUS "SDHDriver disabled! sdh.h - Not Found!")
ELSE ()
    MESSAGE(STATUS "SDHDriver enabled! sdh.h - Found!")    
    ADD_LIBRARY(rwhw_sdh SDHDriver.cpp)
ENDIF()

