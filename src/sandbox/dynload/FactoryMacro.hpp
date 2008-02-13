#ifndef FACTORYMACRO_HPP_
#define FACTORYMACRO_HPP_

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


#endif /*FACTORYMACRO_HPP_*/
