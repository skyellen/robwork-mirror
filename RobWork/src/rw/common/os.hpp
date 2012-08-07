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


#ifndef RW_COMMOM_OS_HPP
#define RW_COMMOM_OS_HPP

#include <string>

#if defined(__CYGWIN__)
	#define RW_CYGWIN
#elif defined(_WIN32) || defined(__WIN32__) || defined(WIN32)
    #define RW_WIN32
#elif defined(macintosh) || defined(__APPLE__) || defined(__APPLE_CC__)
    #define RW_MACOS
#elif defined(linux) || defined(__linux) || defined(__linux__)
    #define RW_LINUX
#endif

#ifdef RW_WIN32
#define DLL_EXPORT extern "C" __declspec(dllexport)
#else
#define DLL_EXPORT extern "C"
#endif


/**
 * @brief encapsulates os dependent functionality
 */
class OS {
public:

    /**
     * @brief
     * @return
     */
    static std::string getDLLExtension() {
        #if defined(RW_WIN32)
            return "dll";
        #elif defined(RW_MACOS)
		    return "dylib";
        #else
            return "so";
        #endif
    }
};

#endif /*RW_COMMOM_OS_HPP*/
