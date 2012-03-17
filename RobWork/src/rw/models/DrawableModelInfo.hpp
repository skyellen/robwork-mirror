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


#ifndef RW_MODELS_DRAWABLEMODELINFO_HPP
#define RW_MODELS_DRAWABLEMODELINFO_HPP

#include <iostream>
#include <rw/math/Transform3D.hpp>
#include <rw/common/PropertyMap.hpp>
#include <rw/kinematics/Frame.hpp>

namespace rw { namespace models {

    /**
       @brief this is a light weight container class for varius information
       regarding drawable models. The drawable information in this class is
       regarded const.
    */
#ifdef DEPRECATED
    class DrawableModelInfo {
    private:
        /**
         * @brief constructor
         */
        DrawableModelInfo() {}

    public:
        /**
         * @brief constructor
         * @param id [in] string idetifier of this drawable
         * @param id [in]
         */
        DrawableModelInfo(const std::string& id, const std::string& name);

       /**
         * @brief constructor
         * @param id [in] string idetifier of this drawable
         * @param t3d [in] Transform for the Drawable
         */
        DrawableModelInfo(
            const std::string& id,
            const std::string& name,
            rw::math::Transform3D<> t3d);

       /**
         * @brief constructor
         * @param id [in] string idetifier of this drawable
         * @param t3d [in] Transofmr for the Drawable
         * @param scale [in] Geometry scale
         * @param wire [in] True to draw in wire mode
         * @param high [in] True to highlight
         */
        DrawableModelInfo(const std::string& id,
                          const std::string& name,
                          rw::math::Transform3D<> t3d,
                          double scale,
                          bool wire,
                          bool high);

        /**
         * @brief gets if this drawable is initially highlighted
         */
        bool isHighlighted() const { return _highlighted; }

        /**
         * @brief gets if this drawable initially is to be drawn in Wire mode
         */
        bool isWireMode() const { return _wireMode; }

        /**
         * @brief gets the string identifier of this drawable info
         */
        const std::string& getId() const { return _drawableId; }

        /**
         * @brief gets the geometric scale of this drawable
         */
        double getGeoScale() const { return _geoScale; }

        /**
         * @brief gets the transform of this drawable
         */
        const rw::math::Transform3D<>& getTransform() const
        { return _transform; }

        /**
         * @brief set the name identifier of this drawable
         * @param name
         */
        void setName(const std::string& name){ _name = name; }

        /**
         * @brief get the name identifier of this drawable
         * @return
         */
        const std::string& getName() const { return _name; }

        static std::vector<DrawableModelInfo> get(const rw::common::PropertyMap& pmap);

        static std::vector<DrawableModelInfo> get(rw::kinematics::Frame* frame);

        static void set(const std::vector<DrawableModelInfo>& data, rw::kinematics::Frame* frame);

        static void set(const std::vector<DrawableModelInfo>& data, rw::common::PropertyMap& pmap);


    private:
        std::string _drawableId, _name;
        rw::math::Transform3D<> _transform;
        double _geoScale;
        bool _wireMode;
        bool _highlighted;
    };

#endif
}} // end namespaces

#endif // end include guard
