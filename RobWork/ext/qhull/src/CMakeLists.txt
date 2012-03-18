
# Order object files by frequency of execution.  Small files at end.
set(
    qhull_src
        rboxlib.c
        user.c
        global.c
        stat.c
        io.c
        geom2.c
        poly2.c
        merge.c
        libqhull.c
        geom.c
        poly.c
        qset.c
        mem.c
        usermem.c
        userprintf.c
        random.c
)

file(GLOB qhull_hdr *.h)

add_library(qhullstatic STATIC ${qhull_src})
set_property(TARGET qhullstatic PROPERTY OUTPUT_NAME "rw_qhull")
set_property(TARGET qhullstatic PROPERTY COMPILE_FLAGS "-Wunused-but-set-variable")
 
INSTALL(TARGETS #qhull 
    qhullstatic 
    #qhullcmd rbox qconvex qdelaunay qvoronoi qhalf
	RUNTIME DESTINATION ${BIN_INSTALL_DIR}
	LIBRARY DESTINATION ${LIB_INSTALL_DIR}
	ARCHIVE DESTINATION ${LIB_INSTALL_DIR})
INSTALL(FILES ${qhull_hdr} DESTINATION ${INCLUDE_INSTALL_DIR}/ext/qhull)
