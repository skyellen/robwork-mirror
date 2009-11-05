
# The dockwelder device currently can't be built with cygwin because of some
# probably trivial issues related to boost::asio.
if (DEFINED CYGWIN)
    message("Warning: Dockwelder device will not be build.")
  set(DocwelderFile )
else ()
  	set(DocwelderFile ./DockWelder.cpp)
 	add_library(
		  rwhw_docwelder
  		  ${DocwelderFile}
	)
endif ()


