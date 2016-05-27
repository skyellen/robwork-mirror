
# Order object files by frequency of execution.  Small files at end.
set(
    csgjs_src
        csgjs/csgjs.cpp
)

file(GLOB csgjs_hdr csgjs/*.cpp)

add_library(rw_csgjs STATIC ${csgjs_src})
set_property(TARGET rw_csgjs PROPERTY OUTPUT_NAME "rw_csgjs")

# this does not compile on some versions of gcc
 
INSTALL(TARGETS
    rw_csgjs 
	RUNTIME DESTINATION ${BIN_INSTALL_DIR}
	LIBRARY DESTINATION ${LIB_INSTALL_DIR}
	ARCHIVE DESTINATION ${LIB_INSTALL_DIR})
INSTALL(FILES ${csgjs_hdr} DESTINATION ${INCLUDE_INSTALL_DIR}/ext/csgjs)
