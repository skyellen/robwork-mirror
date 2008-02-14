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

#ifndef rwlibs_drawable_DrawableMatrix_HPP
#define rwlibs_drawable_DrawableMatrix_HPP

/**
 * @file DrawableMatrix.hpp
 */

#include <rwlibs/os/rwgl.hpp>

#include "Drawable.hpp"

namespace rwlibs { namespace drawable {

    /** @addtogroup drawable */
    /*@{*/

    /**
     * @brief DrawableMatrix, visualization of a matrix of values
     */
    class DrawableMatrix : public Drawable
    {
    private:
        float _width,_height,_maxZ,_zscale;
        void update(UpdateType type);
        boost::numeric::ublas::matrix<float> _vals;

    public:
        /* Functions inherited from Drawable */
        /**
         * @copydoc Drawable::draw
         */
        void draw() const;

        /**
         * @copydoc Drawable::setHighlighted
         */
        void setHighlighted(bool b);

        /**
         * @brief Constructs a DrawableMatrix
         * @param size [in] size of the frame coordinate system
         */
        DrawableMatrix(size_t cols, size_t rows, float width, float height);

        void setValue(size_t col, size_t row, float val){
        	_vals(col,row) = val;
        }
        
        /**
         * @brief sets the max depth of the 3d boxed graph
         */
        void setMaxDepth(float maxlen){
        	_maxZ = maxlen;
        	if(_maxZ<0.001)
        		_maxZ = 0.001;
        		
        	_zscale = 1/_maxZ;
        }
        
        /**
         * Destroys DrawableMatrix
         */
        virtual ~DrawableMatrix(){};
        
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
