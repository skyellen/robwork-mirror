#ifndef rwlibs_simulation_GLScanner3D_HPP
#define rwlibs_simulation_GLScanner3D_HPP

#include <rw/sensor/Scanner3D.hpp>

namespace rwlibs { namespace simulation {

    /** @addtogroup simulation */
    /* @{ */
    
    /**
     * @brief 
     */
    class GLScanner3D: public rw::sensor::Scanner3D {
    public:
        /**
         * @brief constructor
         */
        GLScanner3D();
        
        virtual ~GLScanner3D();
        
    };

    /* @} */
    
}} // end namespaces

#endif // end include guard
