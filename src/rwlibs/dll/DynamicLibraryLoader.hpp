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


#ifndef RWLIBS_DLL_DYNAMICLIBRARYLOADER_HPP
#define RWLIBS_DLL_DYNAMICLIBRARYLOADER_HPP

#include "DynamicLibraryLoaderBase.hpp"

namespace rwlibs {
namespace dll {


    /** @addtogroup dll */
    /*@{*/

/**
 * @brief Loader for dynamic/shared libraries
 *
 * The DynamicLibraryLoader provides a template based interface to the
 * DynamicLibraryLoaderBase.
 *
 * Notice that when using the dynamicLibraryLoader it is necessary to link with
 * additional libraries. On Linux/Mac link with "dl". On Windows link with "kernel32".
 *
 */
template <class T>
class DynamicLibraryLoader: public DynamicLibraryLoaderBase {
public:
    /**
     * @brief Constructs loader and opens the library
     *
     * When constructed the DynamicLibraryLoader open the library and throws
     * an exception on failure. The search for the \b funcname entry is not
     * done until trying to obtain the actual object.
     *
     * @param filename [in] Library file name (with extension)
     * @param funcname [in] Name of factory function in the library (default: factory0)
     */
    DynamicLibraryLoader(const std::string& filename, const std::string& funcname = "factory0"):
        DynamicLibraryLoaderBase(filename),
        _funcname(funcname)
    {

    }

    /**
     * @copydoc ~DynamicLibraryLoaderBase
     */
    virtual ~DynamicLibraryLoader() {};

    /**
     * @brief Returns object loaded from library
     *
     * Calls the factory method in library and returns the object. This method
     * throws an exception if the factory method cannot be found.
     *
     * @return The loaded object. This may be null
     */
    T* get() {
        return (T*) getObject(_funcname);
    }

private:
    const std::string& _funcname;
};


/** @} */

} //end namespace dll
} //end namespace rwlibs

#endif /*RWLIBS_DLL_DYNAMICLIBRARYLOADER_HPP*/
