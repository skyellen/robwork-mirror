#ifndef DYNAMICLIBRARYLOADERBASE_HPP
#define DYNAMICLIBRARYLOADERBASE_HPP

#include <string>

namespace rw {
namespace common {


class DynamicLibraryLoaderBase {
public:
    DynamicLibraryLoaderBase(const std::string& filename);

    int lastError();
    void* getObject(const std::string& funcname);
    
    
    virtual std::string getFileExtension() const;
    
private:
    char* _err;
    
#ifdef WIN32
    HMODULE h;
#else
    void *_handle;
#endif
    
    bool getSymbol(void** v,
                   const char *sym_name);

};




} //end namespace common
} //end namespace rw

#endif /*DYNAMICLIBRARYLOADERBASE_HPP*/
