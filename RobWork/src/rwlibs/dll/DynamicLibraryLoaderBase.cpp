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


#include "DynamicLibraryLoaderBase.hpp"
#include <rw/common/macros.hpp>

using namespace rwlibs::dll;

#ifdef RW_WIN32



DynamicLibraryLoaderBase::DynamicLibraryLoaderBase(const std::string& fname)
{
    // Try to open the library now and get any error message.
	h = LoadLibraryA((fname + getFileExtension()).c_str());
    
    //h = LoadLibraryA((fname).c_str());
    if (h == NULL)
    {
        LPTSTR buffer = NULL;
        std::cout<<"Ready to generate error message "<<GetLastError()<<std::endl;
        if(FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER |
                      FORMAT_MESSAGE_FROM_SYSTEM,
                      NULL,             // Instance
                      GetLastError(),   // Message Number
                      0,                // Language
                      buffer,              // Buffer
                      0,                // Min/Max Buffer size
                      NULL))            // Arguments
        {
            RW_THROW(buffer);
        } else {
            RW_THROW("Unknown Error: Could not open library");
        }
    }
    else
    {
        _err = NULL;
    }
}


DynamicLibraryLoaderBase::~DynamicLibraryLoaderBase() {
    if (h != NULL)
        FreeLibrary(h);
}

bool DynamicLibraryLoaderBase::getSymbol(void** v,
                                         const char *sym_name) {
    // try extract a symbol from the library
    // get any error message is there is any

    if( h!=0 )
    {
        *v = (void*)GetProcAddress(h, sym_name);
        if (v != NULL)
          return true;
        else
        {
            LPTSTR buffer = NULL;
            FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER |
                      FORMAT_MESSAGE_FROM_SYSTEM |
                      FORMAT_MESSAGE_IGNORE_INSERTS,
                      NULL,             // Instance
                      GetLastError(),   // Message Number
                      0,                // Language
                      buffer,           // Buffer
                      0,                // Min/Max Buffer size
                      NULL);            // Arguments
            RW_WARN(buffer);
            return false;
        }
    }
    else
    {
        return false;
    }
}


#else //#ifdef RW_WIN32

#include <dlfcn.h>

DynamicLibraryLoaderBase::DynamicLibraryLoaderBase(const std::string& filename) {
    _handle = dlopen((filename+getFileExtension()).c_str(), RTLD_NOW /* RTLD_GLOBAL*/);
    _err = dlerror();
    if (_handle == NULL || _err != NULL)
        RW_THROW("Unknown Error: Could not open library: "<<_err);
}

DynamicLibraryLoaderBase::~DynamicLibraryLoaderBase() {
    if(_handle != 0)
        dlclose(_handle);
}

bool DynamicLibraryLoaderBase::getSymbol(void** v,
                                         const char *sym_name)
{
    // try extract a symbol from the library
    // get any error message is there is any

    if( _handle != 0 )
    {
        *v = dlsym( _handle, sym_name );
        _err = dlerror();
        if( _err == 0 )
          return true;
        else
          return false;
    }
    else
    {
        return false;
    }

}


#endif //#else



std::string DynamicLibraryLoaderBase::getFileExtension() const {
#ifdef RW_WIN32
    return ".dll";
#endif
#ifdef RW_MACOS
    return ".dylib";
#endif
    return ".so";
}


void* DynamicLibraryLoaderBase::getObject(const std::string& funcname) {

    if (_err != 0) {
        return NULL;
    }

    const char* func;
    if (funcname.size() == 0)
        func = "factory0";
    else
        func = funcname.c_str();




    void* (*factory_func)(void) = NULL;
    if (getSymbol((void**)&factory_func, func )) {
        return factory_func();
    }
    else
		RW_THROW("Unable to identify factory function in dynamic library");
    return 0;
}

