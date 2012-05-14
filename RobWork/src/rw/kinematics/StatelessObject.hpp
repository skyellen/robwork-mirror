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

#ifndef RW_KINEMATICS_STATELESSOBJECT_HPP_
#define RW_KINEMATICS_STATELESSOBJECT_HPP_

#include <rw/kinematics/StateStructure.hpp>
#include "ObjectStateData.hpp"

namespace rw {
namespace kinematics {

    class StatelessObject {
    public:

        virtual void addStateData(rw::kinematics::StateStructure::Ptr statestructure){
            for(size_t i=0;i<_stateDatas.size();i++){
                statestructure->addData(_stateDatas[i]);
            }
        }

    protected:
        void addStateData(StateData* objStateData){
            _stateDatas.push_back(objStateData);
        }

        std::vector<StateData*> _stateDatas;
    };

}
}
#endif /* STATELESSOBJECT_HPP_ */
