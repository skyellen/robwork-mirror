#include "DynamicLibraryLoaderBase.hpp"

#include <rw/common/macros.hpp>

using namespace rw::common;

#ifdef WIN_32 

#include <windows.h>
#include <winbase.h>


DynamicLibraryLoaderBase::DynamicLibraryLoaderBase(const std::string& fname )
{
    // Try to open the library now and get any error message.
    
    h = LoadLibrary((fname + getFileExtension()).c_str());
    
    if (h == NULL)
    {
        DWORD m;
        m = FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | 
                      FORMAT_MESSAGE_FROM_SYSTEM | 
                      FORMAT_MESSAGE_IGNORE_INSERTS,
                      NULL,             /* Instance */
                      GetLastError(),   /* Message Number */
                      0,                /* Language */
                      err,              /* Buffer */
                      0,                /* Min/Max Buffer size */
                      NULL);            /* Arguments */
        cout << err << endl;
    }
    else
    {
        err = NULL;
    }
}


void* DynamicLibraryLoaderBase::Get(const std::string& filename, const std::string& method) {
    // try extract a symbol from the library
    // get any error message is there is any
    
    if( h!=0 )
    {
        *v = GetProcAddress(h, sym_name);
        if (v != NULL)
          return true;
        else
        {
            FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | 
                      FORMAT_MESSAGE_FROM_SYSTEM | 
                      FORMAT_MESSAGE_IGNORE_INSERTS,
                      NULL,             /* Instance */
                      GetLastError(),   /* Message Number */
                      0,                /* Language */
                      err,              /* Buffer */
                      0,                /* Min/Max Buffer size */
                      NULL);            /* Arguments */
            return false;
        }
    }
    else
    {   
        return false;
    }
}


#else //#ifndef WIN_32

#include <dlfcn.h>

DynamicLibraryLoaderBase::DynamicLibraryLoaderBase(const std::string& filename) {
    _handle = dlopen((filename+getFileExtension()).c_str(), RTLD_NOW);
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


#endif //#else



std::string DynamicLibraryLoaderBase::getFileExtension() const {
#ifdef WIN_32
    return ".dll";
#endif
#ifdef MACOSX
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
        RW_WARN("Unable to identify factory function in dynamic library");
    return 0;
}

