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

#include <rw/common/StringUtil.hpp>

#include "StateData.hpp"
#include "StateStructure.hpp"

namespace rw {
namespace kinematics {

    /**
     * @brief class for enabling statelessness in classes
     */
    class StatelessObject {
    public:
        //! destructor
        virtual ~StatelessObject(){}

        /**
         * @brief register all states of this StatelessObject in the statestructure
         * @param statestructure [in] state to register statedata
         */
        virtual void registerStateData(rw::kinematics::StateStructure::Ptr statestructure);

        /**
         * @brief remove all statedata from
         */
        virtual void resetStateData();

    protected:
        /**
         * @brief when inheriting from StatelessObject one can use this Data class for
         * constructing stateless member variables.
         *
         * @example
         * When declaring a double as a stateless variable one would write:
         *
         * StatelessObject::Data<double> _myDouble;
         *
         */
        template<class dataType>
        class Data {
        public:
            Data(rw::kinematics::StatelessObject* obj, int dN=1):
                _N(dN),
                _obj(obj),
                _sdata(NULL)

            {
                StateData *sdata = new StateData((sizeof(dataType)*dN)/sizeof(double)+1, rw::common::StringUtil::ranName("sdata"));
                _sdata = sdata;
                obj->addStateData(sdata);
            }

            dataType* get(const rw::kinematics::State& state){
                return (dataType*)_sdata->getData(state);
            }

            const dataType* get(const rw::kinematics::State& state) const {
                return (dataType*)_sdata->getData(state);
            }

            int getN() const {return _N;}
        private:
            int _N;
            rw::kinematics::StatelessObject* _obj;
            StateData *_sdata;
        };


    protected:
        void addStateData(StateData* objStateData){
            _stateDatas.push_back(objStateData);
        }

        std::vector<StateData*> _stateDatas;
        rw::kinematics::StateStructure::Ptr _statestructure;
    };

}
}
#endif /* STATELESSOBJECT_HPP_ */
