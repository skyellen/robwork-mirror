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

#ifndef RWLIBS_DRAWABLE_LOADERIVG_HPP
#define RWLIBS_DRAWABLE_LOADERIVG_HPP

#include <list>
#include <string>
#include <cmath>
#include <cstring>

#include <rwlibs/drawable/Model3DLoader.hpp>

namespace rwlibs { namespace opengl {
    /**
     * @brief Class for loading in IVG files
     */
    class LoaderIVG: public Model3DLoader
    {
    public:
        LoaderIVG();
        virtual ~LoaderIVG();

        Model3DPtr load(const std::string& name); // Loads a model

    };

}} // end namespaces

#endif
