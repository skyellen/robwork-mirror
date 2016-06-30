/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#ifndef RWLIBS_OPENGL_RENDERMATRIX_HPP
#define RWLIBS_OPENGL_RENDERMATRIX_HPP

//! @file RenderMatrix.hpp

#include <boost/numeric/ublas/matrix.hpp>

#include <rw/graphics/Render.hpp>

namespace rwlibs { namespace opengl {

    /** @addtogroup opengl */
    /*@{*/

    /**
     * @brief RenderMatrix, visualization of a matrix of values
     */
    class RenderMatrix : public rw::graphics::Render
    {
    private:
        float _width,_height,_maxZ,_zscale;
        boost::numeric::ublas::matrix<float> _vals;

    public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<RenderMatrix> Ptr;

        /**
         * @brief Constructs a RenderMatrix
         * @param id [in] String identifier
         * @param cols [in] Number of columns
         * @param rows [in] Number of rows
         * @param width [in] Width of matrix in meters
         * @param height [in] Height of matrix in meters
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

        //! @copydoc rw::graphics::Render::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const
        void draw(const rw::graphics::DrawableNode::RenderInfo& info,
                  rw::graphics::DrawableNode::DrawType type,
                  double alpha) const;

    };

    /*@}*/
}} // end namespaces

#endif // end include guard
