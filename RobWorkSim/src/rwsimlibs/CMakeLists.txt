# IMPORTANT the order in which the directories are included depends on
# the individual dependencies of each lib


# the gui is a collection of generally usefull gui tools
add_subdirectory(gui)

add_subdirectory(ode)
add_subdirectory(rwpe)
add_subdirectory(bullet) 
add_subdirectory(swig)

# the plugins may use all of the above 
add_subdirectory(plugins)
