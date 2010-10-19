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

#include <rw/sensor.hpp>

#ifndef RWS_LUA_RWSTUDIO_HPP
#define RWS_LUA_RWSTUDIO_HPP

#include <rws/RobWorkStudio.hpp>
#include <rw/trajectory.hpp>
#include <rw/trajectory/Path.hpp>
#include <rw/kinematics.hpp>
#include <rwlibs/lua/LuaRobWork.hpp>

namespace rws {
namespace lua {
namespace rwstudio {

    //! @addtogroup lua @{

    /**
     * @brief Lua Wrapper for RobWorkStudio.
     */
    class RobWorkStudio {
    public:
        /**
         * @brief Constructor
         */
        RobWorkStudio(rws::RobWorkStudio* rws);

        /**
         * @see rws::RobWorkStudio::openFile
         */
        void openFile(const std::string& filename);

        /**
         * @see rws::RobWorkStudio::getPropertyMap
         */
        rw::common::PropertyMap& getPropertyMap();

        /**
         * @see rws::RobWorkStudio::setWorkCell
         */
        rwlibs::lua::WorkCell getWorkCell();

        /**
         * @see rws::RobWorkStudio::setWorkCell
         */
        void setWorkcell(rw::models::WorkCellPtr workcell);

        /**
         * @see rws::RobWorkStudio::getCollisionDetector
         */
        rw::proximity::CollisionDetector* getCollisionDetector();

        /**
         * @see rws::RobWorkStudio::getWorkCellGLDrawer
         */
        rwlibs::drawable::WorkCellGLDrawer* getWorkCellGLDrawer();

        /**
         * @see rws::RobWorkStudio::getTimedStatePath
         */
        const rwlibs::lua::TimedStatePath getTimedStatePath();

        /**
         * @see rws::RobWorkStudio::setTimedStatePath
         */
        void setTimedStatePath(const rwlibs::lua::TimedStatePath& path);

        /**
         * @see rws::RobWorkStudio::setState
         */
        void setState(const rwlibs::lua::State& state);

        /**
         * @see rws::RobWorkStudio::getState
         */
        rwlibs::lua::State getState();

        /**
         * @see rws::RobWorkStudio::log
         */
        rw::common::Log& log();

        /**
         * @see rws::RobWorkStudio::saveViewGL
         */
        void saveViewGL(const std::string& filename);

        /**
         * @see rws::RobWorkStudio::updateAndRepaint
         */
        void updateAndRepaint();


        /**
         * @see rws::RobWorkStudio::getView
         */
        rws::ViewGL* getView();

        /**
         * @brief The RobWorkStudio instance
         */
        rws::RobWorkStudio *_rws;
    };

    /**
     * @brief get current robworkstudio instance
     */
    RobWorkStudio* getRobWorkStudio();

    /**
     * @brief set current robworkstudio instance
     */
	void setRobWorkStudio(rws::RobWorkStudio* rwstudio);

	// utils for accessing the current RobWorkStudio state

//	void list(){}


	//! @}
}}}


#endif
