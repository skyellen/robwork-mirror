

# The RobWork test executable to build:
set(TargetName TestSuite)
add_executable(${TargetName} 
    TestSuite.cpp
)

# Libraries to link with the target:
target_link_libraries(${TargetName} ${ROBWORK_LIBRARIES})
