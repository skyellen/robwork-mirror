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

	//! @brief A model of a controller.
    class ControllerModel {
    public:
    	//! @brief Smart pointer type for ControllerModel.
        typedef rw::common::Ptr<ControllerModel> Ptr;

        /**
         * @brief Check if controller is initialized.
         * @param state [in] the state.
         * @return true if initialized, false otherwise.
         */
        bool isInitialized(rw::kinematics::State& state){
            if(_controllerStateMap.find( state.getUniqueId() )==_controllerStateMap.end() ){
                return false;
            }
            return true;
        }

        /**
         * @brief Get controller.
         * @param state [in] the state.
         * @return the controller, or NULL if not found.
         */
        rwlibs::simulation::SimulatedController::Ptr getController(rw::kinematics::State& state)
        {
            if(_controllerStateMap.find( state.getUniqueId() )==_controllerStateMap.end() ){
                return NULL;
            }
            return _controllerStateMap[state.getUniqueId()];
        }

        /**
         * @brief Set a simulated controller in the given state.
         * @param controller [in] the controller.
         * @param state [in/out] update state with new controller.
         */
        void setController(rwlibs::simulation::SimulatedController::Ptr controller, rw::kinematics::State& state){
            _controllerStateMap[state.getUniqueId()] = controller;
        }

    private:
        std::map<int, rwlibs::simulation::SimulatedController::Ptr> _controllerStateMap;
    };

}
}

#endif /* CONTROLLERMODEL_HPP_ */
