#ifndef DYNAMICLIBRARYLOADER_HPP_
#define DYNAMICLIBRARYLOADER_HPP_

#include "DynamicLibraryLoaderBase.hpp"

namespace rw {
namespace common {    


/**
 * @brief Loader for dynamic/shared libraries
 * 
 * The DynamicLibraryLoader provides a template based interface to the
 * DynamicLibraryLoaderBase.
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
     * @copydoc DynamicLibraryLoaderBase
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

} //end namespace common
} //end namespace rw

#endif /*DYNAMICLIBRARYLOADER_HPP_*/
