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


#ifndef RWLIBS_OPENGL_DRAWABLEGEOMETRY_HPP
#define RWLIBS_OPENGL_DRAWABLEGEOMETRY_HPP

/**
 * @file DrawableGeometry.hpp
 */

#include <rw/graphics/Render.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/common/Ptr.hpp>
#include <rwlibs/os/rwgl.hpp>
#include <rw/graphics/DrawableGeometryNode.hpp>
#include "Drawable.hpp"
#include "RenderLines.hpp"
#include "RenderFrame.hpp"
#include "RenderGeometry.hpp"

namespace rwlibs { namespace opengl {

    //! @addtogroup drawable
	// @{

	/**
	 * @brief Abstract base class for all drawable classes
	 *
	 * Classes that are able to draw them self, may inherit from this class.
	 *
	 * The drawable class use a draw mask to distinguish between different
	 * groups to draw. E.g. when taking snapshots with a simulated camera
	 * virtual objects such as the red laser vector or the lines showing
	 * the camera view angle is should not be renered. Hence objects that
	 * are virtual should be set to virtual.
	 *
	 * A call to draw enabling Physical and User1 defined objects look like:
	 * \code
	 * drawable->draw(Drawable::Physical | Drawable::User1);
	 * \endcode
	 */
    class DrawableGeometry: public rw::graphics::DrawableGeometryNode {
    public:

        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<DrawableGeometry> Ptr;

        /**
         * @brief Constructer for Drawable
         *
         * @param name [in] name/id of this drawable
         * @param dmask [in] the drawmask of this drawable
         */
        DrawableGeometry(
            const std::string& name,
            unsigned int dmask = Physical);

        /**
         * @brief Virtual destructor
         */
        virtual ~DrawableGeometry();

        /**
         * @brief Get this drawables Render object
         */
        std::vector<rw::common::Ptr<rw::graphics::Render> > getRenders() const{
            return _drawable->getRenders();
        }

        //--------------------- inherited from DrawableGeometryNode
        //! @copydoc DrawableGeometryNode::setColor(double r, double g, double b, double alpha)
        void setColor(double r, double g, double b, double alpha);
        //! @copydoc rw::graphics::DrawableGeometryNode::setColor(const rw::math::Vector3D<>& rgb)
        void setColor(const rw::math::Vector3D<>& rgb);
        //! @copydoc rw::graphics::DrawableGeometryNode::setAlpha(double )
        void setAlpha(double alpha);
        //! @copydoc rw::graphics::DrawableGeometryNode::getColor()
        rw::math::Vector3D<> getColor();
        //! @copydoc rw::graphics::DrawableGeometryNode::getAlpha()
        double getAlpha();
        //! @copydoc rw::graphics::DrawableGeometryNode::addLines()
        void addLines(const std::vector<rw::geometry::Line >& lines);
        //! @copydoc rw::graphics::DrawableGeometryNode::addLine()
        void addLine(const rw::math::Vector3D<>& v1, const rw::math::Vector3D<>& v2);
        //! @copydoc rw::graphics::DrawableGeometryNode::addGeometry()
        void addGeometry(rw::geometry::Geometry::Ptr geom);
        //! @copydoc rw::graphics::DrawableGeometryNode::addFrameAxis()
        void addFrameAxis(double size);


        //--------------------- inherited from DrawableNode
        //! @copydoc rw::graphics::DrawableNode::draw
        virtual void draw(const rw::graphics::DrawableNode::RenderInfo& info) const{ _drawable->draw(info); };
        //! @copydoc rw::graphics::DrawableNode::setHighlighted
        void setHighlighted(bool b){ _drawable->setHighlighted(b); }
        //! @copydoc rw::graphics::DrawableNode::isHighlighted
        bool isHighlighted() const{ return _drawable->isHighlighted(); }
        //! @copydoc rw::graphics::DrawableNode::setDrawType
        void setDrawType(rw::graphics::DrawableNode::DrawType drawType){ _drawable->setDrawType(drawType); }
        //! @copydoc rw::graphics::DrawableNode::setTransparency()
        void setTransparency(float alpha){ _drawable->setTransparency(alpha); }
        //! @copydoc rw::graphics::DrawableNode::getTransparency()
        float getTransparency(){ return _drawable->getTransparency(); }
        //! @copydoc rw::graphics::DrawableNode::setScale()
        void setScale(float scale){ _drawable->setScale(scale); }
        //! @copydoc rw::graphics::DrawableNode::getScale()
        float getScale() const{ return _drawable->getScale(); }
        //! @copydoc rw::graphics::DrawableNode::setVisible()
        void setVisible(bool enable) { _drawable->setVisible(enable); }
        //! @copydoc rw::graphics::DrawableNode::isVisible()
        bool isVisible() { return _drawable->isVisible(); }
        //! @copydoc rw::graphics::DrawableNode::getTransform()
        const rw::math::Transform3D<>& getTransform() const{ return _drawable->getTransform(); }
        //! @copydoc rw::graphics::DrawableNode::setTransform()
        void setTransform(const rw::math::Transform3D<>& t3d){ _drawable->setTransform(t3d); }
        //! @copydoc rw::graphics::DrawableNode::setMask()
        void setMask(unsigned int mask){ _drawable->setMask(mask); }
        //! @copydoc rw::graphics::DrawableNode::getMask()
        unsigned int getMask() const { return _drawable->getMask(); }


    protected:
        Drawable::Ptr _drawable;

        RenderLines::Ptr _rlines;
        std::vector<RenderFrame::Ptr> _rframes;
        std::vector<RenderGeometry::Ptr> _rgeoms;
    private:
        double _alpha;
        rw::math::Vector3D<> _rgb;

    private:
        DrawableGeometry(const DrawableGeometry&);
        DrawableGeometry& operator=(const DrawableGeometry&);

        void initLines();
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
