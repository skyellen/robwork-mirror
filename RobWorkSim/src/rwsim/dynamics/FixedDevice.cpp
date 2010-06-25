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

#include "FixedDevice.hpp"

//#include "DynamicUtil.hpp"

using namespace rw::math;
using namespace rw::models;

using namespace rwsim::dynamics;


FixedDevice::FixedDevice( rw::models::Device &device , 
						  std::vector<FixedLink*> links , 
						  rw::kinematics::State& state):
    _device(&device),
    _links( links ),
    _qGoal(device.getQ(state)),
    _lastTime(0),_qKp(2.5),_qKd(0.01),_vKp(10),_vKd(0.0)
{
}

#include <rw/math/Math.hpp>
						  
void FixedDevice::addForces(rw::kinematics::State &state, double time){
	std::cout << "FIXEDDEVICE: adding forces! "<< std::endl;
    // calculate the difference between current q and qGoal
  /*  Q qCurr = _device->getQ(state);
    Q qErr = _qGoal - qCurr;
    double dt = time - _lastTime;
    _lastTime = time;
    double MAX_VEL = 1.0;
    for(size_t i=0; i<_links.size(); i++){
    	double vel = _qKp*qErr(i) + _qKd*qErr(i)/dt;
    	vel = Math::Clamp(vel, -MAX_VEL, MAX_VEL);
    	_links[i]->setTargetVel( vel );
    }*/
    //_links[i]->setAcc(MAX_ACC);
    
    /*if( dt < 0.00000001 )
    	return;
    
    double MAX_ACC = 2.0;
    double MAX_VEL = 1.0;
    for(size_t i=0; i<_links.size(); i++){
    	double vel = _qKp*qErr(i) + _qKd*qErr(i)/dt;
    	vel = Math::Clamp(vel, -MAX_VEL, MAX_VEL);
    	double vErr = vel - _links[i]->getQd();
    	std::cout << "Link: " << i << " VEL: " << _links[i]->getQd() << std::endl;
    	double acc = _vKp*vErr + _vKd*vErr/dt;
    	acc = Math::Clamp(acc,-MAX_ACC, MAX_ACC);
    	_links[i]->setAcc(acc);
    }
    */
}
