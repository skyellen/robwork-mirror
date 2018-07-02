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


#ifndef RWLIBS_OPENGL_RENDERMODEL3D_HPP
#define RWLIBS_OPENGL_RENDERMODEL3D_HPP

//! @file RenderModel3D.hpp

#include <rw/graphics/Model3D.hpp>
#include <rw/graphics/Render.hpp>

namespace rwlibs { namespace opengl {
	class RWGLTexture;

//! @addtogroup opengl
// @{


    /**
     * @brief render for the Model3D class.
     */
    class RenderModel3D : public rw::graphics::Render {
    private:
        rw::graphics::Model3D::Ptr _model;

    public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<RenderModel3D> Ptr;

        /**
         * @brief constructor.
         * @param model [in] the model that is to be rendered
         */
        RenderModel3D(rw::graphics::Model3D::Ptr model);

        /**
         * @brief Destructor
         */
    	virtual ~RenderModel3D();

    	/**
    	 * @brief get the model that is being rendered
    	 */
    	rw::graphics::Model3D::Ptr getModel(){return _model;};



    	// Functions inherited from Render
        //! @copydoc rw::graphics::Render::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const
    	void draw(const rw::graphics::DrawableNode::RenderInfo& info,
                  rw::graphics::DrawableNode::DrawType type,
                  double alpha) const;

        /**
         * @brief Fast drawing of model using drawelements array. This draw method require that
         * textured objects use texture coordinates that are mapped to vertices and not
         * faces. Also this method only works on triangle meshes
         * @param info [in] state and rendering specific info
         * @param type [in] the drawtype which is being used
         * @param alpha [in] the alpha value to render with
         */
        void drawUsingArrays(const rw::graphics::DrawableNode::RenderInfo& info, DrawType type, double alpha) const;

        /**
         * @brief Slower drawing of model using simple opengl draw calls. This draw
         * method is slower than the array method but it is more general and does not
         * requre texture coordinates to be ordered specifically
         * @param info [in] state and rendering specific info
         * @param type [in] the drawtype which is being used
         * @param alpha [in] the alpha value to render with
         */
        void drawUsingSimple(const rw::graphics::DrawableNode::RenderInfo& info, DrawType type, double alpha) const;

        //void drawUsingList(DrawType type, double alpha) const;

    private:
        template <class T>
        void drawUsingSimpleFct(const rw::graphics::DrawableNode::RenderInfo& info,
                             const rw::graphics::Model3D::Object3D<T> &obj,
                             rw::graphics::DrawableNode::DrawType type,
                             double alpha) const;

        template <class T>
        void drawUsingArraysFct(const rw::graphics::DrawableNode::RenderInfo& info,
                             const rw::graphics::Model3D::Object3D<T> &obj,
                             rw::graphics::DrawableNode::DrawType type,
                             double alpha) const;

        void useMaterial(const rw::graphics::Model3D::Material& mat, rw::graphics::DrawableNode::DrawType type, double alpha) const;

    private:
        std::vector<rw::common::Ptr<rwlibs::opengl::RWGLTexture> > _textures;
        //bool _shownormals;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
