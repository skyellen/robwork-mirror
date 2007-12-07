
# CMake does not allow a library and executable to have the same name, so we use
# the name tolualib for the library.
add_library(
  tolualib
  src/lib/tolua_event.c
  src/lib/tolua_is.c
  src/lib/tolua_map.c
  src/lib/tolua_push.c
  src/lib/tolua_to.c
  src/bin/tolua.c
  src/bin/toluabind.c)
  # src/bin/toluabind_default.c

add_executable(
  tolua
  src/bin/tolua.cpp)

target_link_libraries(tolua tolualib lualib)
