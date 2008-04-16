
# CMake does not allow a library and executable to have the same name, so we use
# the name lualib for the library.
add_library(yaobilib 
	yaobi_mesh_interface.cpp
	yaobi_moments.cpp
	yaobi_obb_node.cpp
	yaobi_tree_builder.cpp
	yaobi_tri_overlap.cpp
	yaobi.cpp
)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/yaobi_config.h.cmake 
			   ${CMAKE_CURRENT_SOURCE_DIR}/yaobi_config.h)