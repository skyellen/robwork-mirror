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


#ifndef RWLIBS_DRAWABLE_RENDERMODEL3D_HPP
#define RWLIBS_DRAWABLE_RENDERMODEL3D_HPP

//! @file RenderModel3D.hpp

#include "Model3D.hpp"
#include "Render.hpp"

#include <rwlibs/os/rwgl.hpp>
#include <cstring>
#include <iostream>

namespace rwlibs { namespace opengl {

//! @addtogroup drawable
// @{


    /**
     * @brief render for the Model3D class.
     */
    class RenderModel3D : public Render {
    private:
        Model3DPtr _model;

    public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<RenderModel3D> Ptr;

        /**
         * @brief constructor.
         * @param model [in] the model that is to be rendered
         */
        RenderModel3D(Model3DPtr model);

        /**
         * @brief Destructor
         */
    	virtual ~RenderModel3D();

    	/**
    	 * @brief get the model that is being rendered
    	 */
    	Model3DPtr getModel(){return _model;};



    	// Functions inherited from Render
        /**
         * @copydoc Render::draw
         */
        void draw(DrawType type, double alpha) const;

        /**
         * @brief draws the model using drawelements array.
         * @param type [in]
         * @param alpha [in]
         */
        void drawUsingArrays(DrawType type, double alpha) const;

    private:
        void drawUsingArrays(const Model3D::Object3D& obj, DrawType type, double alpha) const;
        void useMaterial(const Model3D::Material& mat, DrawType type, double alpha) const;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
