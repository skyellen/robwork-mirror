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

#ifndef RWLIBS_DLL_FACTORYMACRO_HPP
#define RWLIBS_DLL_FACTORYMACRO_HPP

#include <rw/common/os.hpp>

#ifdef RW_WIN32
#define DLL_EXPORT extern "C" __declspec(dllexport)
#else
#define DLL_EXPORT extern "C"
#endif

/**
 * @brief Macro to be used when loading dynamically
 *
 * Given a class MyPlugin call this macro in the MyPlugin.cpp
 * file as
 * \code
 * DLL_FACTORY_METHOD(MyPlugin);
 * \endcode
 *
 * Notice that to be able to load a plugin dynamically it is required
 * to have a default constructor.
 *
 * @param name [in] Name of class to provide factory method for.
 */
#define DLL_FACTORY_METHOD(name) \
    DLL_EXPORT void* factory0(void) { \
        return new name(); \
    }


#endif /*RWLIBS_DLL_FACTORYMACRO_HPP_*/
