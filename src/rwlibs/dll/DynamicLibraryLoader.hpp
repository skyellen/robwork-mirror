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
