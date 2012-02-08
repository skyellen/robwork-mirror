SET(component_name pa10)

INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(COMPONENT_${component_name}_ENABLE "Enable ${component_name} component." 
                       ON "COMPONENT_${component_name}_ENABLE" ON)

IF( COMPONENT_${component_name}_ENABLE )  
    IF (NOT DEFINED WIN32 OR NOT DEFINED MINGW )
    	SET(LINUX_HEADER_LIBRARY /usr/src/linux/include /usr/src)
    	
    	INCLUDE_DIRECTORIES(${LINUX_HEADER_LIBRARY})
    	#message("Search for PA10 dependencies")
    	find_file(delay_h_file "linux/delay.h" ${LINUX_HEADER_LIBRARY})
    	find_file(system_h_file "asm/system.h" ${LINUX_HEADER_LIBRARY})
    	    	    
    	if (delay_h_file AND system_h_file)
            MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")
    		set(pa10files 
    	    	./PA10.cpp
    	    	./PA10Driver.cpp
    	    	./PA10Virtual.cpp
    	    	./smsclib.c)
    	    
    	  	add_library(
    		  	rwhw_pa10
    		  	${pa10files}
    	  	)
    	  	set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_pa10;")
    	else () 
    	    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED - could not find delay.h and/or system.h ")
      	endif()
    else ()
      MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED - linux only")
    endif()
ELSE()
    MESSAGE(STATUS "RobWorkHardware: ${component_name} component DISABLED")
ENDIF()

