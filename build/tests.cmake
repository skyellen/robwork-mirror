
include(CheckIncludeFileCXX)

link_directories(${RW_ROOT}/ext)

# Why is it that we need this? Why can't the CHECK_INCLUDE_FILE_CXX() command
# just use the directories used when compiling standard files of this project?
# Why isn't there a simple way of just saying: Compile this file exactly the
# same way as if it had been listed in an add_library() command? And why is it
# that setting up this variable here is not needed for Visual Studio, but *is*
# needed for MinGW? What if the has installed PQP somewhere? Then how does the
# user tell us where to search for PQP when include_directories() doesn't work?

list(APPEND CMAKE_REQUIRED_INCLUDES ${RW_INCLUDE_DIRECTORIES})


CHECK_INCLUDE_FILE_CXX("PQP/PQP.h" RW_HAVE_PQP)
CHECK_INCLUDE_FILE_CXX("yaobi/yaobi.h"RW_HAVE_YAOBI)


#IF( USE_XERCES )
#    CHECK_INCLUDE_FILE_CXX("xercesc/dom/DOM.hpp" RW_HAVE_XERCES)
#ENDIF ()
#message("XercesHeader: "${RW_HAVE_XERCES})

