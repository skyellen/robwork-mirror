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

#ifndef RW_KINEMATICS_OBJECTSTATEDATA_HPP_
#define RW_KINEMATICS_OBJECTSTATEDATA_HPP_

#include <rw/kinematics/StateData.hpp>
#include <rw/kinematics/State.hpp>

namespace rw {
namespace kinematics {

    /**
     * @brief the object state saves a single integer in the state array which
     * is used as index to a specific implementation of the object state in StatelessObject
     */
    class ObjectStateData: public rw::kinematics::StateData {
    public:
        typedef rw::common::Ptr<ObjectStateData> Ptr;

        ObjectStateData():StateData(1, "")
        {}

        void setObjectID(int id, rw::kinematics::State& state) {
            double* q = getData(state);
            q[0] = (double)id;
        }

        int getObjectID(const rw::kinematics::State& state) const{
            const double* q = getData(state);
            return (int)q[0];
        }

        virtual ObjectStateData::Ptr clone() = 0;

    };

}
}

#endif /* OBJECTSTATEDATA_HPP_ */
