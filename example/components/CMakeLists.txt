
# We currently can't compile these directories with neither Cygwin nor
# Microsoft compilers:
if (NOT DEFINED CYGWIN AND NOT DEFINED MSVC)
  if( OROCOS_LIB )
 	set(orodependantfiles ./qpcontroller/QPControllerComponent.cpp  
 		  			      ./velocityprofile/AsyncQVelRamp.cpp
    	  			      ./pa10/PA10Component.cpp)
  else () 
    MESSAGE("Warning: Several components will not be compiled since orocos library is missing!")
  endif ()
else ()
    MESSAGE("Warning: Currently components can't be compiled in CYGWIN or MSVC!")
endif()

add_library(
  rw_components
  ${orodependantfiles}
)
