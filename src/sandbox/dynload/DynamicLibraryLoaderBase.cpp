#include "DynamicLibraryLoaderBase.hpp"

#include <rw/common/macros.hpp>

using namespace rw::common;

#ifdef WIN_32 

void* DynamicLibraryLoaderBase::Get(const std::string& filename, const std::string& method) {
    return 0;
}


#else //#ifndef WIN_32

#include <dlfcn.h>

DynamicLibraryLoaderBase::DynamicLibraryLoaderBase(const std::string& filename) {
    _handle = dlopen(filename.c_str(), RTLD_NOW);
    _err = dlerror();
}

bool DynamicLibraryLoaderBase::getSymbol(void** v,
                                   const char *sym_name)
{
    // try extract a symbol from the library
    // get any error message is there is any

    if( _handle!=0 )
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

void* DynamicLibraryLoaderBase::getObject(const std::string& funcname) {
    
    if (_err != 0) {
        return NULL;
    }
    
    const char* func;
    if (funcname.size() == 0)
        func = "factory0";
    else
        func = funcname.c_str();
            
    
    
    
    void* (*factory_func)(void);
    if (getSymbol((void**)&factory_func, func )) {
        return factory_func();  
    }
    else
        RW_WARN("Unable to identify factory function in dynamic library");
    return 0;
}

#endif //#else
