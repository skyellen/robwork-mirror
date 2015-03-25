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

#ifndef RWLIBS_OPENGL_RENDERSCAN_HPP_
#define RWLIBS_OPENGL_RENDERSCAN_HPP_

//! @file RenderScan.hpp

#include <rwlibs/os/rwgl.hpp>
#include <rw/geometry/PointCloud.hpp>
#include <rw/sensor/Scanner25DModel.hpp>

#include "RWGLTexture.hpp"
#include <rw/graphics/Render.hpp>

namespace rwlibs {
namespace opengl {

	//! @addtogroup opengl
	// @{


    /**
     * @brief renders Image25D, Scan2D or a simple distance.
     */
	class RenderScan: public rw::graphics::Render{
	public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<RenderScan> Ptr;

        //! @brief constructor
		RenderScan();

		//! @brief constructor
		RenderScan(const rw::geometry::PointCloud& img);

        /**
         * @brief the renderer will pull the scanner for
         * @param scanner
         * @return
         */
        RenderScan(const rw::sensor::Scanner25DModel::Ptr scanner);

        //! @brief destructor
		virtual ~RenderScan();

		/**
		 * \brief set a 2.5 dimensional scan
		 * @param img
		 */
		void setScan(const rw::geometry::PointCloud& img);

		/**
		 * \brief set a one dimensional scan
		 * @param dist
		 */
		void setScan(float dist);

        /**
         * @brief the minimum depth of the points, where depth is in
         * the negative z-axis direction
         * @param depth [in] depth in the negative z-axis direction
         */
        void setMinDepth(float depth){ _minDepth=depth;};

        /**
         * @brief the maximum depth of the points, where depth is in
         * the negative z-axis direction
         * @param depth [in] depth in the negative z-axis direction
         */
        void setMaxDepth(float depth){ _maxDepth=depth;};

        //! @copydoc rw::graphics::Render::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const
        void draw(const rw::graphics::DrawableNode::RenderInfo& info,
                  rw::graphics::DrawableNode::DrawType type,
                  double alpha) const;

	private:
        rw::sensor::Scanner25DModel::Ptr _scanner;
        rw::geometry::PointCloud _img;
        float _minDepth,_maxDepth;
	};
	//! smart pointer type
	typedef rw::common::Ptr<RenderScan> RenderScanPtr;

	//! @}
}
}
#endif /* RENDERSCAN_HPP_ */
