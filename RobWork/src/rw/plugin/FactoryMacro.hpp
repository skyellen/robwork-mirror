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


#ifndef RW_PLUGIN_FACTORYMACRO_HPP
#define RW_PLUGIN_FACTORYMACRO_HPP

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


#endif /*RW_PLUGIN_FACTORYMACRO_HPP*/
