/*
 * ControllerModel.hpp
 *
 *  Created on: 24/01/2011
 *      Author: jimali
 */

#ifndef CONTROLLERMODEL_HPP_
#define CONTROLLERMODEL_HPP_

#include <rwlibs/simulation/SimulatedController.hpp>

namespace rwsim {
namespace dynamics {

    class ControllerModel {
    public:
        typedef rw::common::Ptr<ControllerModel> Ptr;

        bool isInitialized(rw::kinematics::State& state){
            if(_controllerStateMap.find( state.getUniqueId() )==_controllerStateMap.end() ){
                return false;
            }
            return true;
        }

        rwlibs::simulation::SimulatedController::Ptr getController(rw::kinematics::State& state)
        {
            if(_controllerStateMap.find( state.getUniqueId() )==_controllerStateMap.end() ){
                return NULL;
            }
            return _controllerStateMap[state.getUniqueId()];
        }

        void setController(rwlibs::simulation::SimulatedController::Ptr sensor, rw::kinematics::State& state){
            _controllerStateMap[state.getUniqueId()] = sensor;
        }

    private:

        std::map<int, rwlibs::simulation::SimulatedController::Ptr> _controllerStateMap;
    };

}
}

#endif /* CONTROLLERMODEL_HPP_ */
