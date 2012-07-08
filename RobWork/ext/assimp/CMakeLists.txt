cmake_minimum_required( VERSION 2.6 )
PROJECT( Assimp )
SET ( PROJECT_VERSION "2.0" )

INCLUDE_DIRECTORIES( include )

# If this is an in-source build (CMAKE_SOURCE_DIR == CMAKE_BINARY_DIR),
# write the library/executable files to the respective directories in the
# source tree. During an out-of-source build, however, do not litter this
# directory, since that is probably what the user wanted to avoid.
IF ( CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR )
	SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/lib )
	SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin )
ENDIF ( CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR )

# Cache these to allow the user to override them manually.
#SET( LIB_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib" CACHE PATH
#	"Path the built library files are installed to." )
#SET( INCLUDE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/include" CACHE PATH
#	"Path the header files are installed to." )
#SET( BIN_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/bin" CACHE PATH
#	"Path the tool executables are installed to." )

SET( Boost_DETAILED_FAILURE_MSG ON )
FIND_PACKAGE( Boost 1.35 )

IF ( NOT Boost_FOUND )
	MESSAGE( FATAL_ERROR
		"Boost libraries (http://www.boost.org/) not found. "
		"You can build a non-boost version of Assimp with slightly reduced "
		"functionality by specifying -DENABLE_BOOST_WORKAROUND=ON."
	)
ENDIF ( NOT Boost_FOUND )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIRS} )

ADD_SUBDIRECTORY( code/ )
