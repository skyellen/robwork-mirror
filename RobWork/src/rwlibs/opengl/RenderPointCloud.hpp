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

#ifndef RWLIBS_OPENGL_DRAWABLEPOINTCLOUD_HPP
#define RWLIBS_OPENGL_DRAWABLEPOINTCLOUD_HPP

//! @file RenderPointCloud.hpp

#include <rw/math/Vector3D.hpp>
#include <rw/geometry/Line.hpp>
#include <list>

#include <rw/graphics/Render.hpp>

namespace rwlibs { namespace opengl {
/** @addtogroup opengl */
/*@{*/
    /**
     * @brief Render a point cloud
     */	
    class RenderPointCloud: public rw::graphics::Render
    {
    public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<RenderPointCloud> Ptr;

        /**
         * @brief Constructs RenderPointCloud with no points
         */
        RenderPointCloud();

        /**
         * @brief Construct RenderPointCloud adding the points specified
         *
         * @param points [in] Points to draw
         */
        RenderPointCloud(const std::vector<rw::math::Vector3D<float> >& points);

        /**
         * @brief Descructor
         */
        virtual ~RenderPointCloud();

        /**
         * @brief Adds a single line to the drawable
         *
         * @note Each call to addLine generates an update of the display list.
         * Use addLines to add multiple lines with only one update.
         *
         * @param p [in] point to add
         */
        void addPoint(const rw::math::Vector3D<float>& p);

        /**
         * @brief Adds a single line to the drawable
         */
		void addPoint(const rw::math::Vector3D<double>& p);

        /**
         * @brief Adds a collection of points
         *
         * After all points are added, the display list will be updated
         *
         * @param points [in] List of points
         */
        void addPoints(const std::vector<rw::math::Vector3D<float> >& points);

        /**
         * @brief Adds a collection of points
         * @param points
         */
		void addPoints(const std::vector<rw::math::Vector3D<double> >& points);

        /**
         * @brief Sets the color of the points.
         *
         * The influence of the alpha value depends on how opengl is configured.
         * Calling setColor triggers an update of the display list
         *
         * @param r [in] red [0;1]
         * @param g [in] green [0;1]
         * @param b [in] blue [0;1]
         * @param alpha [in] alpha [0;1]
         */
        void setColor(float r, float g, float b, float alpha);

        /**
         * @brief Sets point size.
         *
         * The thickness is forwarded to glPointSize. Default 2.0.
         * Calling setPointSize triggers an update of the display list
         * @param size [in] Point size
         */
        void setPointSize(float size);

        /**
         * @brief Clears all points
         *
         * When clearing the points a new display list without points will be generated.
         */
        void clear();

        //! @copydoc rw::graphics::Render::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const
        void draw(const rw::graphics::DrawableNode::RenderInfo& info,
                  rw::graphics::DrawableNode::DrawType type,
                  double alpha) const;

        /**
		* @brief Rerender the scene into the display list.
		* This method need to be called after all points has been added.
		*/
        void rerender();

    private:


        std::string _id;
		std::vector<rw::math::Vector3D<float> > _points;
        GLuint _displayListId;

        float _r;
        float _g;
        float _b;
        float _alpha;
        float _pointSize;
    };

    typedef rw::common::Ptr<RenderPointCloud> RenderPointCloudPtr;
//! @}
}} // end namespaces

#endif // end include guard
