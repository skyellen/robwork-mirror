/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#ifndef RW_PLUGIN_DYNAMICLIBRARYLOADERBASE_HPP
#define RW_PLUGIN_DYNAMICLIBRARYLOADERBASE_HPP


#include <string>

#include <rw/common/os.hpp>

#ifdef RW_WIN32
    #include <windows.h>
    #include <winbase.h>
#endif

namespace rw {
namespace plugin {


/** @addtogroup plugin */
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


/** @} */

} //end namespace plugin
} //end namespace rw

#endif /*RW_PLUGIN_DYNAMICLIBRARYLOADERBASE_HPP*/
