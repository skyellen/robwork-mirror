/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef rwlibs_drawable_Renderer_HPP
#define rwlibs_drawable_Renderer_HPP

/**
 * @file Renderer.hpp
 */

#include <rwlibs/os/rwgl.hpp>
#include <iostream>

namespace rwlibs { namespace drawable {

    /** @addtogroup drawable */
    /*@{*/

    /**
     * @brief Abstract base class for all renderable classes
     *
     * Classes that are able to render them self, may inherit from this class.
     */
    class Render {
    public:
        /**
         * @brief when calling render on the draw mode or type
         * can be specified.
         */
        enum DrawType {
            SOLID, //! Render in solid
            WIRE, //! Render in wireframe
            OUTLINE //! Render both solid and wireframe
        };
        
        /**
         * @brief destructor
         */
        virtual ~Render(){};
        
        /**
         * @brief draws the object.
         */
        virtual void draw(DrawType type, double alpha) const = 0;
    
    protected:
    	Render(){};
    	
    private:
        Render(const Render&);
        Render& operator=(const Render&);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
