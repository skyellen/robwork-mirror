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

    class SensorModel {
    public:

        typedef rw::common::Ptr<SensorModel> Ptr;

        bool isInitialized(rw::kinematics::State& state){
            if(_sensorStateMap.find( state.getUniqueId() )==_sensorStateMap.end() ){
                return false;
            }
            return true;
        }

        rwlibs::simulation::SimulatedSensor::Ptr getSensor(rw::kinematics::State& state){
            if(_sensorStateMap.find( state.getUniqueId() )==_sensorStateMap.end() ){
                return NULL;
            }
            return _sensorStateMap[state.getUniqueId()];
        }

        void setSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor, rw::kinematics::State& state){
            _sensorStateMap[state.getUniqueId()] = sensor;
        }

        rw::common::PropertyMap::Ptr getPropertyMap(){ return _propertyMap; }

    protected:
        std::map<int, rwlibs::simulation::SimulatedSensor::Ptr> _sensorStateMap;
        rw::common::PropertyMap::Ptr _propertyMap;
    };

}
}
#endif /* SENSORMODEL_HPP_ */
