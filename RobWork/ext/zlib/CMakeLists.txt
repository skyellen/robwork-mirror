PROJECT(zlib C)

set(VERSION "1.2.8")

#============================================================================
# zlib
#============================================================================

set(ZLIB_PUBLIC_HDRS
    zconf.h
    zlib.h
)
set(ZLIB_PRIVATE_HDRS
    crc32.h
    deflate.h
    gzguts.h
    inffast.h
    inffixed.h
    inflate.h
    inftrees.h
    trees.h
    zutil.h
)
set(ZLIB_SRCS
    adler32.c
    compress.c
    crc32.c
    deflate.c
    gzclose.c
    gzlib.c
    gzread.c
    gzwrite.c
    inflate.c
    infback.c
    inftrees.c
    inffast.c
    trees.c
    uncompr.c
    zutil.c
)

# parse the full version number from zlib.h and include in ZLIB_FULL_VERSION
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/zlib.h _zlib_h_contents)
string(REGEX REPLACE ".*#define[ \t]+ZLIB_VERSION[ \t]+\"([-0-9A-Za-z.]+)\".*"
    "\\1" ZLIB_FULL_VERSION ${_zlib_h_contents})

add_library(rw_zlib STATIC ${ZLIB_SRCS} ${ZLIB_PUBLIC_HDRS} ${ZLIB_PRIVATE_HDRS})

if(MSVC)
	SET_TARGET_PROPERTIES(rw_zlib PROPERTIES COMPILE_FLAGS -D_CRT_NONSTDC_NO_DEPRECATE)
endif()

INSTALL(TARGETS rw_zlib ARCHIVE DESTINATION "${LIB_INSTALL_DIR}" )
INSTALL(FILES ${ZLIB_PUBLIC_HDRS} DESTINATION "${INCLUDE_INSTALL_DIR}/ext/zlib/")