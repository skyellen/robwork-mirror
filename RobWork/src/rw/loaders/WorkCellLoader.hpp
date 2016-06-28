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


#ifndef RW_LOADERS_WORKCELLLOADER_HPP
#define RW_LOADERS_WORKCELLLOADER_HPP

/**
 * @file WorkCellLoader.hpp
 */

#include <string>
#include <rw/graphics/WorkCellScene.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/common/ExtensionPoint.hpp>

// Forward declarations
//namespace rw { namespace models { class WorkCell; }}

namespace rw { namespace loaders {

    /** @addtogroup loaders */
    /* @{*/

    /**
     * @brief Defines an interface
     */
    class WorkCellLoader
    {
    public:

        //! @brief smart pointer of WorkCellLoader
        typedef rw::common::Ptr<WorkCellLoader> Ptr;

        //! @brief destructor
        virtual ~WorkCellLoader(){}

        /**
         * @brief load a workcell from file
         * @param filename [in] path to workcell file
         */
        virtual models::WorkCell::Ptr loadWorkCell(const std::string& filename) = 0;

        /**
         * @brief set the scene that is used to create drawable models
         * @param scene [in] scene factory
         */
        virtual void setScene( rw::graphics::WorkCellScene::Ptr scene ){_wcscene = scene;};

        /**
         * @brief get the scene used to create a drawable scene
         * @return
         */
        virtual rw::graphics::WorkCellScene::Ptr getScene( ){return _wcscene;};

    	/**
    	 * @addtogroup extensionpoints
    	 * @extensionpoint{rw::loaders::WorkCellLoader::Factory, rw::loaders::WorkCellLoader, rw.loaders.WorkCellLoader}
 	 	 */


		/**
		 * @brief a factory for WorkCellLoader. This factory also defines an
		 * extension point for workcell loaders.
		 */
	    class Factory: public rw::common::ExtensionPoint<WorkCellLoader> {
	    public:
	    	//! constructor
	        Factory():rw::common::ExtensionPoint<WorkCellLoader>("rw.loaders.WorkCellLoader", "Example extension point"){};

	        static rw::common::Ptr<WorkCellLoader> getWorkCellLoader(const std::string& format);

			/**
			 * @brief Loads/imports a workcell from a file.
			 * An exception is thrown if the file can't be loaded.
			 * XML as well as TUL workcell formats are supported.
			 * @param filename [in] name of workcell file.
			 */
			static models::WorkCell::Ptr load(const std::string& filename);

	    };

    protected:
		WorkCellLoader() {}

		WorkCellLoader(rw::graphics::WorkCellScene::Ptr scene):_wcscene(scene) {}

    private:
		rw::graphics::WorkCellScene::Ptr _wcscene;

    };

    typedef WorkCellLoader::Factory WorkCellFactory;
    /**@}*/
}} // end namespaces

#endif // end include guard
