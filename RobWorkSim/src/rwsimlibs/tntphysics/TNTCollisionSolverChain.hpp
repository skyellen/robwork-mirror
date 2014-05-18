/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTCOLLISIONSOLVERCHAIN_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTCOLLISIONSOLVERCHAIN_HPP_

/**
 * @file TNTCollisionSolverChain.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTCollisionSolverChain
 */

#include "TNTCollisionSolver.hpp"

namespace rwsimlibs {
namespace tntphysics {

// Forward declarations
class TNTCollisionSolverSingle;

//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief A collision solver that can handle impulse chains.
 *
 * The solver uses the TNTCollisionSolverSingle to solve for individual pairs of objects.
 *
 * An exception will be thrown if the bodies are connected in a circular or tree type structure.
 *
 * The impulse must only origin from once object pair in the chain, otherwise an exception will be thrown.
 */
class TNTCollisionSolverChain: public rwsimlibs::tntphysics::TNTCollisionSolver {
public:
    //! @brief Empty constructor.
	TNTCollisionSolverChain();

    //! @brief Destructor.
	virtual ~TNTCollisionSolverChain();

	//! @copydoc TNTCollisionSolver::applyImpulses
	virtual void applyImpulses(
			const std::vector<const TNTContact*>& contacts,
			const TNTBodyConstraintManager& bc,
			const TNTMaterialMap* map,
			TNTIslandState& tntstate,
			const rw::kinematics::State& rwstate) const;

private:
	TNTCollisionSolverSingle* _solver;
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTCOLLISIONSOLVERCHAIN_HPP_ */
