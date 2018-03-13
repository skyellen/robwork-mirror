cmake_minimum_required(VERSION 2.8)
if(POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif()

project(yaobi)

SET(YAOBI_HPP_FILES
    yaobi.h
    yaobi_config.h
    yaobi_fpu.h
    yaobi_matvec.h
    yaobi_mesh_interface.h
    yaobi_moments.h
    yaobi_obb_node.h
    yaobi_settings.h
    yaobi_tree_builder.h
    yaobi_tri_overlap.h
    yaobi_vector.h
)

# CMake does not allow a library and executable to have the same name, so we use
# the name lualib for the library.
add_library(yaobi 
	yaobi_mesh_interface.cpp
	yaobi_moments.cpp
	yaobi_obb_node.cpp
	yaobi_tree_builder.cpp
	yaobi_tri_overlap.cpp
	yaobi.cpp
)

IF(CMAKE_VERSION VERSION_GREATER 3.3)
	SET_TARGET_PROPERTIES(yaobi PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
ENDIF()

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/yaobi_config.h.cmake 
			   ${CMAKE_CURRENT_SOURCE_DIR}/yaobi_config.h)
			   
INSTALL(TARGETS yaobi DESTINATION ${LIB_INSTALL_DIR})
INSTALL(FILES ${YAOBI_HPP_FILES} DESTINATION "${INCLUDE_INSTALL_DIR}/ext/yaobi/")
