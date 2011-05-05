
if (NOT DEFINED WIN32 OR NOT DEFINED MINGW )
	set(LINUX_HEADER_LIBRARY /usr/src/linux/include /usr/src)
	
	include_directories(${LINUX_HEADER_LIBRARY})
	message("Search for PA10 dependencies")
	find_file(delay_h_file "linux/delay.h" ${LINUX_HEADER_LIBRARY})
	find_file(system_h_file "asm/system.h" ${LINUX_HEADER_LIBRARY})
	
	#message("ASDF "${delay_h_file})
	#message("ASDF "${system_h_file})
	
	if (NOT delay_h_file)
	    message("Info - Could not find linux/delay.h required to compile PA10")
	else()
	    message("Info - Found linux/delay.h required to compile PA10")
	endif()
	
	if (NOT system_h_file)
	    message("Info - Could not find asm/system.h required to compile PA10")
	else()
	    message("Info - Found asm/system.h required to compile PA10")
	endif()

	if (delay_h_file AND system_h_file)
        MESSAGE(STATUS "pa10 enabled!")
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
	    MESSAGE(STATUS "pa10 disabled!")
  	endif()
else ()
  message(STATUS "PA10 disabled! Currently only supported in linux!")
endif()



