/*
 * PhysicsEngineFactory.hpp
 *
 *  Created on: 20-10-2008
 *      Author: jimali
 */

#ifndef PHYSICSENGINEFACTORY_HPP_
#define PHYSICSENGINEFACTORY_HPP_

#include <RWSimConfig.hpp>

#include <vector>

#include <dynamics/DynamicWorkcell.hpp>
#include "Simulator.hpp"

#include <RWSimConfig.hpp>

#ifdef RWSIM_HAVE_RWPHYS
#include <simulator/rwphysics/RWSimulator.hpp>
#endif

#ifdef RWSIM_HAVE_ODE
#include <simulator/ode/ODESimulator.hpp>
#endif

#ifdef RWSIM_HAVE_BULLET
#include <simulator/bullet/BtSimulator.hpp>
#endif

class PhysicsEngineFactory {
public:
    static std::vector<std::string> getEngineIDs();

    static Simulator* newPhysicsEngine(const std::string& engineID,
                                       dynamics::DynamicWorkcell* dwc);

};


#endif /* PHYSICSENGINEFACTORY_HPP_ */
