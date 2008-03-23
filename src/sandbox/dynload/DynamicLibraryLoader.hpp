#ifndef DYNAMICLIBRARYLOADER_HPP_
#define DYNAMICLIBRARYLOADER_HPP_

#include "DynamicLibraryLoaderBase.hpp"

namespace rw {
namespace common {    


template <class T>
class DynamicLibraryLoader: public DynamicLibraryLoaderBase {

public:
    DynamicLibraryLoader(const std::string& filename, const std::string& funcname):
        DynamicLibraryLoaderBase(filename),
        _funcname(funcname)
    {

    }
    virtual ~DynamicLibraryLoader() {};
    
    T* get() {
        return (T*) getObject(_funcname);
    }
    
private:
    const std::string& _funcname;
};

} //end namespace common
} //end namespace rw

#endif /*DYNAMICLIBRARYLOADER_HPP_*/
