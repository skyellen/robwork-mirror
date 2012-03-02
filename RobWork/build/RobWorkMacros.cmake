#
# This is a collection of macros used throughout the robwork project
# 

######################################################################
# Converts a standard cmake list to a python string list
#
MACRO(RW_TO_PYTHON_STR_LIST ITEMS OUTPUT)

SET(RESULT_STR "'")
FOREACH (item ${ITEMS})
    SET(RESULT_STR "${RESULT_STR}${item}','")
ENDFOREACH()
SET(${OUTPUT} "${RESULT_STR}'")

ENDMACRO()

######################################################################
# Converts a standard VERSION 0.1.2 to three version numbers
#
MACRO(RW_SPLIT_VERSION VERSION MAJOR MINOR PATCH)
STRING( REGEX MATCHALL "[0-9]+" VERSIONS ${VERSION} )
LIST( GET VERSIONS 0 ${MAJOR})
LIST( GET VERSIONS 1 ${MINOR})
LIST( GET VERSIONS 2 ${PATCH})
ENDMACRO()