
# Find and setup OpenGL.
include(FindOpenGL)

set(LibraryList
  rw_algorithms
  rw
)

# The JTag demo executable to build:
set(TargetName JTagMarkerDemo)
add_executable(${TargetName} JTagMarkerDemo.cpp)

# Libraries to link with the target:
target_link_libraries(${TargetName} ${LibraryList})
