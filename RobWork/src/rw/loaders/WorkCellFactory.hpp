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


#ifndef RW_LOADERS_WORKCELLFACTORY_HPP
#define RW_LOADERS_WORKCELLFACTORY_HPP

/**
 * @file WorkCellLoader.hpp
 */

#include <string>
#include <memory>
#include <rw/graphics/WorkCellScene.hpp>
#include <rw/models/WorkCell.hpp>

// Forward declarations
//namespace rw { namespace models { class WorkCell; }}

namespace rw { namespace loaders {

    /** @addtogroup loaders */
    /* @{*/

    /**
     * @brief Loader for workcell files.
     */
    class WorkCellFactory
    {
    public:
        /**
         * @brief Loads/imports a workcell from a file.
         * An exception is thrown if the file can't be loaded.
         * XML as well as TUL workcell formats are supported.
         * @param filename [in] name of workcell file.
         */
        static models::WorkCell::Ptr load(const std::string& filename);

        /**
         * @brief Loads/imports a workcell from a file.
         * An exception is thrown if the file can't be loaded.
         * XML as well as TUL workcell formats are supported.
         * @param filename [in] name of workcell file.
         */
        static models::WorkCell::Ptr load(const std::string& filename, rw::graphics::WorkCellScene::Ptr wcscene);


    private:
        WorkCellFactory() {}
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
