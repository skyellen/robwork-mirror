#ifndef rwlibs_simulation_GLScanner1D_HPP
#define rwlibs_simulation_GLScanner1D_HPP

#include <rw/sensor/Scanner1D.hpp>

namespace rwlibs { namespace simulation {

    /** @addtogroup simulation */
    /* @{ */
    
    /**
     * @brief 
     */
    class GLScanner1D: public rw::sensor::Scanner1D {
    public:
        /**
         * @brief constructor
         */
        GLScanner1D();
        
        virtual ~GLScanner1D();
        
    };

    /* @} */
    
}} // end namespaces

#endif // end include guard
