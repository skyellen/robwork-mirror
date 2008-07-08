#ifndef rwlibs_simulation_GLScanner2D_HPP
#define rwlibs_simulation_GLScanner2D_HPP

#include <rw/sensor/Scanner2D.hpp>

namespace rwlibs { namespace simulation {

    /** @addtogroup simulation */
    /* @{ */
    
    /**
     * @brief 
     */
    class GLScanner2D: public rw::sensor::Scanner2D {
    public:
        /**
         * @brief constructor
         */
        GLScanner2D();
        
        virtual ~GLScanner2D();
        
    };

    /* @} */
    
}} // end namespaces

#endif // end include guard
