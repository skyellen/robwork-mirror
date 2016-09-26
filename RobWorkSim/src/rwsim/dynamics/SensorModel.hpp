/*
 * SensorModel.hpp
 *
 *  Created on: 24/01/2011
 *      Author: jimali
 */

#ifndef SENSORMODEL_HPP_
#define SENSORMODEL_HPP_


#include <rwlibs/simulation/SimulatedSensor.hpp>


namespace rwsim {
namespace dynamics {

	//! @brief A model of a sensor.
    class SensorModel {
    public:
    	//! @brief Smart pointer type for SensorModel.
        typedef rw::common::Ptr<SensorModel> Ptr;

        /**
         * @brief Check if sensor is initialized.
         * @param state [in] the state.
         * @return true if initialized, false otherwise.
         */
        bool isInitialized(rw::kinematics::State& state){
            if(_sensorStateMap.find( state.getUniqueId() )==_sensorStateMap.end() ){
                return false;
            }
            return true;
        }

        /**
         * @brief Get sensor.
         * @param state [in] the state.
         * @return the sensor, or NULL if not found.
         */
        rwlibs::simulation::SimulatedSensor::Ptr getSensor(rw::kinematics::State& state){
            if(_sensorStateMap.find( state.getUniqueId() )==_sensorStateMap.end() ){
                return NULL;
            }
            return _sensorStateMap[state.getUniqueId()];
        }

        /**
         * @brief Set a simulated sensor in the given state.
         * @param sensor [in] the sensor.
         * @param state [in/out] update state with new sensor.
         */
        void setSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor, rw::kinematics::State& state){
            _sensorStateMap[state.getUniqueId()] = sensor;
        }

        /**
         * @brief Get property map for model.
         * @return the properties.
         */
        rw::common::PropertyMap::Ptr getPropertyMap(){ return _propertyMap; }

    protected:
        std::map<int, rwlibs::simulation::SimulatedSensor::Ptr> _sensorStateMap;
        rw::common::PropertyMap::Ptr _propertyMap;
    };

}
}
#endif /* SENSORMODEL_HPP_ */
