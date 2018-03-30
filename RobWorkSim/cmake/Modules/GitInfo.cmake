function(git_describe _var)

    if(NOT GIT_FOUND)
        find_package(Git)
    endif()

    if(NOT GIT_FOUND)
        set(${_var} "NOGIT" PARENT_SCOPE)
        return()
    endif()

    execute_process(COMMAND
        "${GIT_EXECUTABLE}"
        describe
        ${ARGN}
        WORKING_DIRECTORY
        "${CMAKE_SOURCE_DIR}"
        RESULT_VARIABLE
        res
        OUTPUT_VARIABLE
        out
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE)

    if(NOT res EQUAL 0)
        set(out "GITERROR")
    endif()

    set(${_var} "${out}" PARENT_SCOPE)

endfunction()


function(git_revparse _var)

    if(NOT GIT_FOUND)
        find_package(Git)
    endif()

    if(NOT GIT_FOUND)
        set(${_var} "NOGIT" PARENT_SCOPE)
        return()
    endif()

    execute_process(COMMAND
        "${GIT_EXECUTABLE}"
        rev-parse
        ${ARGN}
        WORKING_DIRECTORY
        "${CMAKE_SOURCE_DIR}"
        RESULT_VARIABLE
        res
        OUTPUT_VARIABLE
        out
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE)

    if(NOT res EQUAL 0)
        set(out "GITERROR")
    endif()

    set(${_var} "${out}" PARENT_SCOPE)

endfunction()
