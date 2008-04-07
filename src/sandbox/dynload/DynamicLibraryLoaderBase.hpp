#ifndef DYNAMICLIBRARYLOADERBASE_HPP
#define DYNAMICLIBRARYLOADERBASE_HPP


#include <string>
#include <iostream>
#ifdef WIN32
    #include <windows.h>
    #include <winbase.h>
#endif

#include <rw/common/macros.hpp>

namespace rw {
namespace common {

/**
 * @brief Base for DynamicLibraryLoader
 * 
 * The DynamicLibraryLoaderBase implements the platform dependent code
 * associated with dynamic loading of libraries. It has been tested on
 * Linux, MingW (no XP), Visual Studio 2005, and MacOS 10.4. 
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
    
#ifdef WIN32
    HMODULE h;    
#else
    void *_handle;
#endif
    
    bool getSymbol(void** v, const char *sym_name); 
};




} //end namespace common
} //end namespace rw

#endif /*DYNAMICLIBRARYLOADERBASE_HPP*/
