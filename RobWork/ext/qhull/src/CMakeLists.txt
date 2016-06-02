
# Order object files by frequency of execution.  Small files at end.
set(
    qhull_src
        libqhull/rboxlib.c
        libqhull/user.c
        libqhull/global.c
        libqhull/stat.c
        libqhull/io.c
        libqhull/geom2.c
        libqhull/poly2.c
        libqhull/merge.c
        libqhull/libqhull.c
        libqhull/geom.c
        libqhull/poly.c
        libqhull/qset.c
        libqhull/mem.c
        libqhull/usermem.c
        libqhull/userprintf.c
        libqhull/random.c
)

file(GLOB qhull_hdr *.h)

add_library(rw_qhull STATIC ${qhull_src})
set_property(TARGET rw_qhull PROPERTY OUTPUT_NAME "rw_qhull")

# this does not compile on some versions of gcc
#set_property(TARGET rw_qhull PROPERTY COMPILE_FLAGS "-Wunused-but-set-variable")
 
INSTALL(TARGETS #qhull 
    rw_qhull 
    #qhullcmd rbox qconvex qdelaunay qvoronoi qhalf
	RUNTIME DESTINATION ${BIN_INSTALL_DIR}
	LIBRARY DESTINATION ${LIB_INSTALL_DIR}
	ARCHIVE DESTINATION ${LIB_INSTALL_DIR})
INSTALL(FILES ${qhull_hdr} DESTINATION ${INCLUDE_INSTALL_DIR}/ext/qhull)
