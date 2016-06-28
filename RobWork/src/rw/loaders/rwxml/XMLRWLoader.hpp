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


#ifndef RW_LOADERS_XMLRWLOADER_HPP
#define RW_LOADERS_XMLRWLOADER_HPP

#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

namespace rw { namespace loaders {
	/** @addtogroup loaders */
	/*@{*/

	/**
	 * @brief this class loads a workcell in xml format from a filename.
	 *
	 */
    class XMLRWLoader: public WorkCellLoader
    {
    public:
        //! @brief default constructor
        XMLRWLoader(){};

        //! @brief constructor
        //XMLRWLoader(rw::graphics::WorkCellScene::Ptr scene):WorkCellLoader(scene){};

        //! @brief destructor
        virtual ~XMLRWLoader(){}

        //! @copydoc WorkCellLoader::loadWorkCell(const std::string&)
        models::WorkCell::Ptr loadWorkCell(const std::string& filename);

        /**
         * @brief Loads/imports robwork workcell in XML file format
         *
         * An exception is thrown if the file can't be loaded.
         *
         * @param filename [in] filename of XML file
         */
		static rw::models::WorkCell::Ptr load(const std::string& filename);
    };

	/*@}*/
}} // end namespaces

#endif // end include guard
