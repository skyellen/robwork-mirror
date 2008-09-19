# -*- cmake -*-

# Extra compiler flags and settings:

if (CMAKE_COMPILER_IS_GNUCXX)
  if (DEFINED MINGW)
    set(RW_CXX_FLAGS "-Wall")
  else ()
    message("RW_CXX_FLAGS: Building with -fPic by default.")
    set(RW_CXX_FLAGS "-Wall -fPIC")
  endif ()

elseif (DEFINED MSVC)
  # Remove the min()/max() macros or else RobWork won't compile.
  add_definitions(-DNOMINMAX)

  # Without this define for boost-bindings we can't link with lapack.
  add_definitions(-DBIND_FORTRAN_LOWERCASE_UNDERSCORE)
endif ()
