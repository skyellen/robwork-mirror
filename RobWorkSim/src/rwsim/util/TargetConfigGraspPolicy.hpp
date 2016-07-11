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

#ifndef RWSIM_UTIL_TARGETCONFIGGRASPPOLICY_HPP_
#define RWSIM_UTIL_TARGETCONFIGGRASPPOLICY_HPP_

#include <rw/common/PropertyMap.hpp>
#include <rw/common/Ptr.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>

#include "GraspPolicy.hpp"

namespace rwsim { namespace dynamics { class DynamicDevice; } }

namespace rwsim {
namespace util {

    /**
     * @brief This grasp policy will close the fingers of a device to a
     * randomly choosen target position which is generated either from a
     * predefined set of target configurations or from one of the
     * selected hueristics.
     */
    class TargetConfigGraspPolicy: public GraspPolicy {
    public:

        typedef rw::common::Ptr<TargetConfigGraspPolicy> Ptr;

        /**
         * @brief constructor
         * @param dev
         * @return
         */
        TargetConfigGraspPolicy(rwsim::dynamics::DynamicDevice* dev);

        virtual ~TargetConfigGraspPolicy();

        void setDefaultSettings();

        static std::string getID(){ return "TargetConfigGraspPolicy"; };

        // inherited from GraspPolicy

        virtual void reset(const rw::kinematics::State& state);

        virtual rwlibs::simulation::SimulatedController::Ptr getController();

        virtual std::string getIdentifier(){ return TargetConfigGraspPolicy::getID();}

        virtual rw::common::PropertyMap getSettings(){ return _settings;};

        virtual void applySettings();

    private:

        rw::common::PropertyMap _settings;
        rwsim::dynamics::DynamicDevice* _dev;
        rwlibs::simulation::SimulatedController::Ptr _controller;
    };

}
}


#endif /* GRASPPOLICY_HPP_ */
