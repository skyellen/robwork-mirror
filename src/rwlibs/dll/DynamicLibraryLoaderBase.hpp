/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef RWLIBS_DLL_DYNAMICLIBRARYLOADERBASE_HPP
#define RWLIBS_DLL_DYNAMICLIBRARYLOADERBASE_HPP


#include <string>
#include <iostream>

#include <rw/common/os.hpp>

#ifdef RW_WIN32
    #include <windows.h>
    #include <winbase.h>
#endif

#include <rw/common/macros.hpp>

namespace rwlibs {
namespace dll {


    /** @addtogroup dll */
    /*@{*/


/**
 * @brief Base for DynamicLibraryLoader
 *
 * The DynamicLibraryLoaderBase implements the platform dependent code
 * associated with dynamic loading of libraries. It has been tested on
 * Linux, MingW (on Windows XP), Visual Studio 2005, and MacOS 10.5.
 */
class DynamicLibraryLoaderBase {
public:
    /**
     * @brief Open the library
     *
     * If the library cannot be opened an exception is thrown
     * @param filename [in] Library filename
     */
	DynamicLibraryLoaderBase(const std::string& filename);

    /**
     * @brief Closes the handle to the library
     */
	virtual ~DynamicLibraryLoaderBase();

    /**
     * @brief Returns pointer to object loaded using the factory function \b funcname
     * @param funcname [in] Name of factory function in library (usually factory0)
     */
	void* getObject(const std::string& funcname);


    /**
     * @brief Returns file extension of dynamic libraries on the current platform
     *
     * The return value will be
     * Windows: ".dll"
     * Unix/Linux: ".so"
     * MacOSX: ".dylib"
     */
    virtual std::string getFileExtension() const;

private:
    char* _err;

#ifdef RW_WIN32
    HMODULE h;
#else
    void *_handle;
#endif

    bool getSymbol(void** v, const char *sym_name);
};




} //end namespace dll
} //end namespace rwlibs

#endif /*RWLIBS_DLL_DYNAMICLIBRARYLOADERBASE_HPP*/
