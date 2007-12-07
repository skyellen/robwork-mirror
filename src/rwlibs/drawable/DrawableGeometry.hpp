#ifndef RWLIBS_DRAWABLE_DRAWABLEGEOMETRY_HPP
#define RWLIBS_DRAWABLE_DRAWABLEGEOMETRY_HPP


#include "Drawable.hpp"

#include <rw/geometry/Geometry.hpp>

namespace rwlibs {
namespace drawable {

    /**
     * @brief DrawableGeometry provide a class for visualizing Geometry objects
     * 
     */ 
    class DrawableGeometry: public Drawable {
    public:
        /**
         * @brief Constructs DrawableGeometry object
         *
         * Constructs a DrawableGeometry object to visualize the geometry. 
         * The DrawableGeometry takes ownership of the Geometry object and
         * deletes when done with it.
         *
         * @param geo [in] the geometry to draw
         * @param r [in] red color component
         * @param g [in] green color component
         * @param b [in] blue color component
         *
         */
        DrawableGeometry(rw::geometry::Geometry* geo,
                        float r = 0.8,
                        float g = 0.8,
                        float b = 0.8);
        
        /**
         * @brief Destructor
         */
        virtual ~DrawableGeometry();
        
        /**
         * @brief Sets color of the object
         * @param r [in] red color component
         * @param g [in] green color component
         * @param b [in] blue color component
         */
        void setColor(float r, float g, float b);
        
    protected:
        /**
         * @copydoc Drawable::update
         */
        void update(UpdateType type);
        
        
    private:
        rw::geometry::Geometry* _geometry;
        float _r, _g, _b;
    };
    
} //end namespace drawable
} //end namespace rwlibs

#endif //#ifndef RWLIBS_DRAWABLE_DRAWABLEGEOMETRY_HPP
