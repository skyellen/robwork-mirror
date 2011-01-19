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


#ifndef RWLIBS_DRAWABLE_DRAWABLEGEOMETRY_HPP
#define RWLIBS_DRAWABLE_DRAWABLEGEOMETRY_HPP

/**
 * @file Drawable.hpp
 */

#include "Render.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/common/Ptr.hpp>
#include <rwlibs/os/rwgl.hpp>
#include "DrawableGeometryNode.hpp"
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
    class DrawableGeometry: public DrawableGeometryNode {
    public:

        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<DrawableGeometry> Ptr;

        /**
         * @brief Constructer for Drawable
         *
         * @param render [in] Render to be used by the drawable
         * @param drawType [in] DrawType of the Drawable. Default value is Physical
         */
        DrawableGeometry(
            const std::string& name,
            unsigned int dmask = Physical);

        /**
         * @brief Virtual destructor
         */
        virtual ~DrawableGeometry();

        //--------------------- inherited from DrawableGeometryNode
        void setColor(double r, double g, double b, double alpha);
        void setColor(const rw::math::Vector3D<>& rgb);
        void setAlpha(double alpha);

        rw::math::Vector3D<> getColor();
        double getAlpha();

        void addLines(const std::vector<rw::geometry::Line >& lines);
        void addLine(const rw::math::Vector3D<>& v1, const rw::math::Vector3D<>& v2);

        void addGeometry(rw::geometry::Geometry::Ptr geom);

        void addFrameAxis(double size);


        //--------------------- inherited from DrawableNode
        virtual void draw(const RenderInfo& info) const{ _drawable->draw(info); };

        void setHighlighted(bool b){ _drawable->setHighlighted(b); }

        bool isHighlighted() const{ return _drawable->isHighlighted(); }

        void setDrawType(DrawType drawType){ _drawable->setDrawType(drawType); }

        void setTransparency(float alpha){ _drawable->setTransparency(alpha); }

        float getTransparency(){ return _drawable->getTransparency(); }

        void setScale(float scale){ _drawable->setScale(scale); }

        float getScale() const{ return _drawable->getScale(); }

        void setVisible(bool enable) { _drawable->setVisible(enable); }

        bool isVisible() { return _drawable->isVisible(); }

        const rw::math::Transform3D<>& getTransform() const{ return _drawable->getTransform(); }

        void setTransform(const rw::math::Transform3D<>& t3d){ _drawable->setTransform(t3d); }

        void setMask(unsigned int mask){ _drawable->setMask(mask); }
        unsigned int getMask() const { return _drawable->getMask(); }


        /**
         * @brief Get this drawables Render object
         */
        std::vector<rw::common::Ptr<Render> > getRenders() const{
        	return _drawable->getRenders();
        }

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
