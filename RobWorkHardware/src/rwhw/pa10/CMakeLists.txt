SET(SUBSYS_NAME pa10 )
SET(SUBSYS_DESC "Driver for pa10 robot" )
SET(SUBSYS_DEPS rw )

SET(build TRUE)
set(DEFAULT FALSE)
set(REASON "") 

IF(${BUILD_pa10} )
    IF (DEFINED WIN32 OR DEFINED MINGW )
        set(BUILD_pa10 FALSE)    
        set(REASON "Linux only component!")
    ELSE()
    	SET(LINUX_HEADER_LIBRARY /usr/src/linux/include /usr/src)
    	
    	INCLUDE_DIRECTORIES(${LINUX_HEADER_LIBRARY})
    	#message("Search for PA10 dependencies")
    	find_file(delay_h_file "linux/delay.h" ${LINUX_HEADER_LIBRARY})
    	find_file(system_h_file "asm/system.h" ${LINUX_HEADER_LIBRARY})
    	    	    
    	if (NOT (delay_h_file AND system_h_file) )
            set(BUILD_pa10 FALSE)    
            set(REASON "Missing linux headers: linux/delay.h and asm/system.h!")	
    	endif()
    
    ENDIF()
ENDIF()

RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND( build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} )
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
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
ENDIF()

