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


#ifndef RW_LOADER_TUL_TULLOADER_HPP
#define RW_LOADER_TUL_TULLOADER_HPP

/**
 * @file TULLoader.hpp
 */

#include <string>
#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>


namespace rw { namespace loaders {

    /** @addtogroup loaders */
    /* @{*/

    /**
     * @brief Loader for the AMROSE TUL format
     */
    class TULLoader: public WorkCellLoader
    {
    public:
    	//! @copydoc WorkCellLoader::loadWorkCell
		rw::models::WorkCell::Ptr loadWorkCell(const std::string& filename);

        /**
         * @brief Loads/imports TUL file
         *
         * An exception is thrown if the file can't be loaded.
         *
         * @param filename [in] filename of TUL file
         */
		static models::WorkCell::Ptr load(const std::string& filename);
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
