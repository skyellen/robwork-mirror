
# Order object files by frequency of execution.  Small files at end.
set(
    qhull_src
        qhull/rboxlib.c
        qhull/user.c
        qhull/global.c
        qhull/stat.c
        qhull/io.c
        qhull/geom2.c
        qhull/poly2.c
        qhull/merge.c
        qhull/libqhull.c
        qhull/geom.c
        qhull/poly.c
        qhull/qset.c
        qhull/mem.c
        qhull/usermem.c
        qhull/userprintf.c
        qhull/random.c
)

file(GLOB qhull_hdr *.h)

add_library(qhullstatic STATIC ${qhull_src})
set_property(TARGET qhullstatic PROPERTY OUTPUT_NAME "rw_qhull")

# this does not compile on some versions of gcc
#set_property(TARGET qhullstatic PROPERTY COMPILE_FLAGS "-Wunused-but-set-variable")
 
INSTALL(TARGETS #qhull 
    qhullstatic 
    #qhullcmd rbox qconvex qdelaunay qvoronoi qhalf
	RUNTIME DESTINATION ${BIN_INSTALL_DIR}
	LIBRARY DESTINATION ${LIB_INSTALL_DIR}
	ARCHIVE DESTINATION ${LIB_INSTALL_DIR})
INSTALL(FILES ${qhull_hdr} DESTINATION ${INCLUDE_INSTALL_DIR}/ext/qhull)
