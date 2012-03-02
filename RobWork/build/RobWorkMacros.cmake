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

