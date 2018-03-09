PROJECT(minizip C)

set(VERSION "1.2.8")

#============================================================================
# minizip
#============================================================================

include_directories(${ZLIB_INCLUDE_DIRS})

set(SOURCE
	ioapi.c
#	miniunz.c
#	minizip.c
	mztools.c
	unzip.c
	zip.c
)

set(HEADERS
	crypt.h
	ioapi.h
	mztools.h
	unzip.h
	zip.h
)

if(WIN32)
    set(SOURCE ${SOURCE} iowin32.c)
    set(HEADERS ${HEADERS} iowin32.h)
endif(WIN32)

add_library(rw_unzip ${SOURCE} ${HEADERS})

target_link_libraries(rw_unzip ${ZLIB_LIBRARIES} )
if(BUILD_SHARED_LIBS)
	target_compile_definitions(rw_unzip PRIVATE ZLIB_DLL ZLIB_INTERNAL)
endif()

INSTALL(TARGETS rw_unzip DESTINATION "${LIB_INSTALL_DIR}" )
INSTALL(FILES ${HEADERS} DESTINATION "${INCLUDE_INSTALL_DIR}/ext/unzip/")
