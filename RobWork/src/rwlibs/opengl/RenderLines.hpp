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

#ifndef RWLIBS_OPENGL_DRAWABLELINES_HPP
#define RWLIBS_OPENGL_DRAWABLELINES_HPP

//! @file RenderLines.hpp

#include <rw/math/Vector3D.hpp>

#include <rw/graphics/Render.hpp>

namespace rw { namespace geometry { class Line; } }

namespace rwlibs { namespace opengl {
/** @addtogroup opengl */
/*@{*/
    /**
     * @brief Render drawing a collection of lines
     */
    class RenderLines: public rw::graphics::Render
    {
    public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<RenderLines> Ptr;

        /**
         * @brief Constructs RenderLine with no lines
         */
        RenderLines();

        /**
         * @brief Construct RenderLine adding the lines specified
         *
         * @param lines [in] Lines to draw
         */
        RenderLines(const std::vector<rw::geometry::Line >& lines);

        /**
         * @brief Descructor
         */
        virtual ~RenderLines();

        /**
         * @brief Adds a single line to the drawable
         *
         * @param v1 [in] Start point for line
         * @param v2 [in] End point for line
         */
        void addLine(const rw::math::Vector3D<>& v1, const rw::math::Vector3D<>& v2);

        /**
         * @brief Adds a collection of lines
         *
         * @param lines [in] List of lines
         */
        void addLines(const std::vector<rw::geometry::Line>& lines);

        /**
         * @brief Sets the color of the lines.
         *
         * The influence of the alpha value depends on how opengl is configured.
         *
         * @param r [in] red [0;1]
         * @param g [in] green [0;1]
         * @param b [in] blue [0;1]
         * @param alpha [in] alpha [0;1]
         */
        void setColor(float r, float g, float b, float alpha);

        /**
         * @brief Sets thickness of the line.
         *
         * The thickness is forwarded to glLineWidth. Default 1.0.
         * @param thickness [in] Thickness of the lines
         */
        void setThickness(float thickness);

        /**
         * @brief Clears all lines
         *
         * When clearing the lines a new display list without lines will be generated.
         */
        void clear();

        //! @copydoc rw::graphics::Render::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const
        void draw(const rw::graphics::DrawableNode::RenderInfo& info,
                  rw::graphics::DrawableNode::DrawType type,
                  double alpha) const;

    private:
        //Initilized the color and thickness parameters
        void rerender();

        std::string _id;
        std::vector<rw::geometry::Line> _lines;

        float _r;
        float _g;
        float _b;
        float _alpha;
        float _thickness;
    };

    //! smart pointer to renderlines
    typedef rw::common::Ptr<RenderLines> RenderLinesPtr;
//! @}
}} // end namespaces

#endif // end include guard
