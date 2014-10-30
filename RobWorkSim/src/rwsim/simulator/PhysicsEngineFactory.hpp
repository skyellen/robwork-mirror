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

#ifndef RWSIM_SIMULATOR_PHYSICSENGINEFACTORY_HPP_
#define RWSIM_SIMULATOR_PHYSICSENGINEFACTORY_HPP_

#include "PhysicsEngine.hpp"
//#include <RobWorkSimConfig.hpp>

//#include "DynamicSimulator.hpp"

//#include <rwsim/dynamics/DynamicWorkCell.hpp>
//#include <rwsim/rwphysics/RWSimulator.hpp>

#include <boost/function.hpp>

/*
#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp> // !
#include <boost/lambda/construct.hpp>
#include <boost/function.hpp>
*/
#include <vector>

namespace rwsim {
namespace simulator {

	/**
	 * @brief Factory for creating physics engines
	 */
	class PhysicsEngineFactory {
	public:
		static std::vector<std::string> getEngineIDs();

		static bool hasEngineID(const std::string& engineID);

		static PhysicsEngine::Ptr makePhysicsEngine(const std::string& engineID, rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc);

        static PhysicsEngine::Ptr makePhysicsEngine(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc);

        typedef boost::function<PhysicsEngine*(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell>)> makePhysicsEngineFunctor;
        static void addPhysicsEngine(const std::string& engineID, makePhysicsEngineFunctor constructor);

        /*
        template<class T>
        class Register{
        public:
            static bool _Register(const std::string& name){

                PhysicsEngineFactory::makePhysicsEngineFunctor odephysics =
                        boost::lambda::bind( boost::lambda::new_ptr<T>(), boost::lambda::_1);

                PhysicsEngineFactory::addPhysicsEngine(name, odephysics);
                return true;
            };
        private:
            Register(){};
            ~Register(){};
            //static std::string type;
        };
        */

	};
}
}


#endif /* PHYSICSENGINEFACTORY_HPP_ */
