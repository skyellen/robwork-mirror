/*********************************************************************
 * RobWork Version 0.3
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

#ifndef RWLIBS_DRAWABLE_RENDERMATRIX_HPP
#define RWLIBS_DRAWABLE_RENDERMATRIX_HPP

/**
 * @file RenderMatrix.hpp
 */

#include <rwlibs/os/rwgl.hpp>

#include <boost/numeric/ublas/matrix.hpp>

#include "Render.hpp"

namespace rwlibs { namespace drawable {

    /** @addtogroup drawable */
    /*@{*/

    /**
     * @brief RenderMatrix, visualization of a matrix of values
     */
    class RenderMatrix : public Render
    {
    private:
        float _width,_height,_maxZ,_zscale;
        boost::numeric::ublas::matrix<float> _vals;

    public:

        /**
         * @brief Constructs a RenderMatrix
         * @param size [in] size of the frame coordinate system
         */
        RenderMatrix(const std::string& id, size_t cols, size_t rows, float width, float height);

        /**
         * Destroys RenderMatrix
         */
        virtual ~RenderMatrix(){};

        /**
         * @brief sets the value of the texel at (col,row)
         */
        void setValue(size_t col, size_t row, float val){
        	_vals(col,row) = val;
        }

        /**
         * @brief sets the max depth of the 3d boxed graph
         */
        void setMaxDepth(float maxlen){
        	_maxZ = maxlen;
        	if(_maxZ < 0.001f)
        		_maxZ = 0.001f;

        	_zscale = 1 / _maxZ;
        }

        /* Functions inherited from Render */
        /**
         * @copydoc Render::draw
         */
        void draw(DrawType type, double alpha) const;

    };

    /*@}*/
}} // end namespaces

#endif // end include guard
