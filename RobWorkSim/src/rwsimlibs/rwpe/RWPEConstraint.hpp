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

#ifndef RWSIMLIBS_RWPE_RWPECONSTRAINT_HPP_
#define RWSIMLIBS_RWPE_RWPECONSTRAINT_HPP_

/**
 * @file RWPEConstraint.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPEConstraint
 */

#include <rw/math/VectorND.hpp>
#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/Wrench6D.hpp>

#include <list>
#include <vector>

// Forward declarations
namespace rw { namespace kinematics { class State; } }

namespace rwsimlibs {
namespace rwpe {

// Forward declarations
class RWPEBody;
class RWPEIslandState;

//! @addtogroup rwsimlibs_rwpe
//! @{
/**
 * @brief A generic interface that makes implementation of constraints simple.
 *
 * A constraint works between two objects, and it is possible to choose a constraint mode independently for
 * different directions. If Velocity mode is chosen, this indicates that there should be zero relative velocity
 * between the two objects. A Wrench constraint indicates that there is a relationship between forces that should
 * be maintained in force-space. If the Wrench mode is used, a custom model for the force-space constraint should
 * be specified - otherwise the wrench will be fixed to zero by default. Using Wrench mode with a zero wrench
 * constraint should be avoided - instead use the Free mode. There is an important distinction between these two
 * cases. When using the Wrench mode, equations and unknowns are added and solved for. For degenerate systems
 * the Wrench mode might not give the exact forces that where desired. The Free mode applies allows applying a
 * fixed force that is included in the existing equations, but it does not increase the size of the equation
 * system. Even though there are degeneracies in a system, the force in the Free directions will always be as
 * specified manually.
 *
 * To be able to supply velocity-constraints to the solver, the RWPEConstraint uses the RWPEIntegrator associated
 * to the constrained objects. These integrators
 * are responsible for providing a relationship between the applied constraint forces and torques and the objects
 * contribution to the relative motion in the constraint. The RWPEConstraint can combine these linear
 * relationships and can construct matrix blocks that are used directly by a RWPESolver to set up the complete
 * equation system. This is how the RWPESolverSVD works for instance.
 *
 * A constraint is specified by a point on each object where the constraint will be acting. Furthermore it
 * is defined by a orthonormal basis for linear constraints, and a orthonormal basis for angular constraints.
 * Often these two rotations will be equal, but in some cases they might not be equal.
 * The contraint mode can be set for these orthonormal directions individually as already described.
 */
class RWPEConstraint {
public:
	//! @brief The choice of constraint mode - the RWPESolver uses these modes to impose the correct kind of constraints.
	typedef enum Mode {
		Free,   //!< No constraint
		Wrench, //!< Force-space constraint
		Velocity//!< Zero velocity constraint
	} Mode;

	//! @brief Destructor.
	virtual ~RWPEConstraint();

	/**
	 * @brief Get a pointer to the parent RWPEBody object.
	 * @return pointer to RWPEBody object.
	 */
	const RWPEBody* getParent() const;

	/**
	 * @brief Get a pointer to the parent RWPEBody object.
	 * @return pointer to RWPEBody object.
	 */
	const RWPEBody* getChild() const;

	/**
	 * @name Local positions.
	 * Functions used to get the positions of the constraints in local coordinates.
	 */
	///@{
	/**
	 * @brief Get the local position on the parent object from the center of mass.
	 * @return position in local coordinates.
	 */
	virtual const rw::math::Vector3D<>& getPositionParent() const;

	/**
	 * @brief Get the local position on the child object from the center of mass.
	 * @return position in local coordinates.
	 */
	virtual const rw::math::Vector3D<>& getPositionChild() const;

	/**
	 * @brief Get the linear orthonormal basis on the parent object.
	 * @return rotation from local coordinate system.
	 */
	virtual const rw::math::Rotation3D<>& getLinearRotationParent() const;

	/**
	 * @brief Get the linear orthonormal basis on the child object.
	 * @return rotation from local coordinate system.
	 */
	virtual const rw::math::Rotation3D<>& getLinearRotationChild() const;

	/**
	 * @brief Get the angular orthonormal basis on the parent object.
	 * @return rotation from local coordinate system.
	 */
	virtual const rw::math::Rotation3D<>& getAngularRotationParent() const;

	/**
	 * @brief Get the angular orthonormal basis on the child object.
	 * @return rotation from local coordinate system.
	 */
	virtual const rw::math::Rotation3D<>& getAngularRotationChild() const;
	///@}

	/**
	 * @name Global positions.
	 * Functions used to get the positions of the constraints in global coordinates.
	 */
	///@{
	/**
	 * @brief Get the global position on the parent object in world coordinates.
	 * @param state [in] the current state of the system.
	 * @return position in world coordinates.
	 */
	virtual rw::math::Vector3D<> getPositionParentW(const RWPEIslandState &state) const;

	/**
	 * @brief Get the global position on the child object in world coordinates.
	 * @param state [in] the current state of the system.
	 * @return position in world coordinates.
	 */
	virtual rw::math::Vector3D<> getPositionChildW(const RWPEIslandState &state) const;

	/**
	 * @brief Get the linear orthonormal basis on the parent object in world coordinates.
	 * @param state [in] the current state of the system.
	 * @return rotation from world coordinate system.
	 */
	virtual rw::math::Rotation3D<> getLinearRotationParentW(const RWPEIslandState &state) const;

	/**
	 * @brief Get the linear orthonormal force basis on the parent object in world coordinates.
	 * @param state [in] the current state of the system.
	 * @return rotation from world coordinate system.
	 */
	virtual rw::math::Rotation3D<> getLinearRotationParentForceW(const RWPEIslandState &state) const;

	/**
	 * @brief Get the linear orthonormal basis on the child object in world coordinates.
	 * @param state [in] the current state of the system.
	 * @return rotation from world coordinate system.
	 */
	virtual rw::math::Rotation3D<> getLinearRotationChildW(const RWPEIslandState &state) const;

	/**
	 * @brief Get the linear orthonormal force basis on the child object in world coordinates.
	 * @param state [in] the current state of the system.
	 * @return rotation from world coordinate system.
	 */
	virtual rw::math::Rotation3D<> getLinearRotationChildForceW(const RWPEIslandState &state) const;

	/**
	 * @brief Get the angular orthonormal basis on the parent object in world coordinates.
	 * @param state [in] the current state of the system.
	 * @return rotation from world coordinate system.
	 */
	virtual rw::math::Rotation3D<> getAngularRotationParentW(const RWPEIslandState &state) const;

	/**
	 * @brief Get the angular orthonormal basis on the child object in world coordinates.
	 * @param state [in] the current state of the system.
	 * @return rotation from world coordinate system.
	 */
	virtual rw::math::Rotation3D<> getAngularRotationChildW(const RWPEIslandState &state) const;
	///@}

	/**
	 * @name Global velocities.
	 * Functions used to get the velocities of the constraints in global coordinates.
	 */
	///@{
	/**
	 * @brief Get the velocity of the constraint on the parent object.
	 * @param islandState [in] the state.
	 * @param rwstate [in] the state.
	 * @return the velocity in world coordintes.
	 */
	virtual rw::math::VelocityScrew6D<> getVelocityParentW(const RWPEIslandState &islandState, const rw::kinematics::State &rwstate) const;

	/**
	 * @brief Get the velocity of the constraint on the child object.
	 * @param islandState [in] the state.
	 * @param rwstate [in] the state.
	 * @return the velocity in world coordintes.
	 */
	virtual rw::math::VelocityScrew6D<> getVelocityChildW(const RWPEIslandState &islandState, const rw::kinematics::State &rwstate) const;
	///@}

	/**
	 * @brief Update the constraint for instance by applying wrenches.
	 * @param islandState [in/out] the state to update.
	 * @param rwstate [in] the input to reset to.
	 */
	virtual void update(RWPEIslandState &islandState, const rw::kinematics::State &rwstate) = 0;

	/**
	 * @brief Reset the initial positions and orientations and clear wrenches.
	 * @param islandState [in/out] the state to reset.
	 * @param rwstate [in] the input to reset to.
	 */
	virtual void reset(RWPEIslandState &islandState, const rw::kinematics::State &rwstate) = 0;

	/**
	 * @brief Step a stateful constraint forward in time.
	 * @param islandState [in/out] the state to update.
	 * @param rwstate [in] the input to reset to.
	 * @param h [in] the stepsize.
	 */
	virtual void step(RWPEIslandState &islandState, const rw::kinematics::State &rwstate, double h) = 0;

	/**
	 * @name Wrenches
	 * Functions used to set and get the constraint wrenches.
	 */
	///@{
	/**
	 * @brief Get the total constraint wrench.
	 * @param islandState [in] the current state where wrenches are stored.
	 * @return the wrench in world coordinates.
	 */
	virtual rw::math::Wrench6D<> getWrench(const RWPEIslandState &islandState) const;

	/**
	 * @brief Get the part of the constraint wrench that was applied directly in force mode.
	 * @param islandState [in] the current state where wrenches are stored.
	 * @return the wrench in world coordinates.
	 */
	virtual rw::math::Wrench6D<> getWrenchApplied(const RWPEIslandState &islandState) const;

	/**
	 * @brief Get the part of the constraint wrench that was applied due to enforced velocity constraints.
	 * @param islandState [in] the current state where wrenches are stored.
	 * @return the wrench in world coordinates.
	 */
	virtual rw::math::Wrench6D<> getWrenchConstraint(const RWPEIslandState &islandState) const;

	/**
	 * @brief Clear the wrench.
	 * @param islandState [in/out] the state to reset the wrench in.
	 */
	virtual void clearWrench(RWPEIslandState &islandState);

	/**
	 * @brief Clear the applied wrench.
	 * @param islandState [in/out] the state to reset the wrench in.
	 */
	virtual void clearWrenchApplied(RWPEIslandState &islandState);

	/**
	 * @brief Clear the constraint wrench.
	 * @param islandState [in/out] the state to reset the wrench in.
	 */
	virtual void clearWrenchConstraint(RWPEIslandState &islandState);

	/**
	 * @brief Apply a wrench in the constraint manually.
	 * @param islandState [in/out] the state to update.
	 * @param wrench [in] the wrench to apply.
	 * @note Applying a wrench in a constrained direction will cause the solver to make
	 * the constraint wrench smaller. The total wrench should be the same in these directions.
	 */
	virtual void applyWrench(RWPEIslandState &islandState, const rw::math::Wrench6D<>& wrench);
	///@}

	/**
	 * @name Equation system
	 * Functions used to set up the equation system.
	 */
	///@{
	/**
	 * @brief Get which directions to constrain in velocity mode, and which to constrain in wrench mode.
	 * @return list of length 6.
	 */
	virtual std::vector<Mode> getConstraintModes() const = 0;

	/**
	 * @brief Get the part of the equation that is independent of the forces and torques
	 * (often the desired relative velocity).
	 * @param stepsize [in] the stepsize to use.
	 * @param gravity [in] the gravity in world coordinates.
	 * @param discontinuity [in] first step after a discontinuity will be integrated and solved for differently.
	 * @param constraints0 [in] list of initial constraints.
	 * @param constraintsH [in] list of constraints with pre-applied forces.
	 * @param rwstate [in] the state.
	 * @param islandState0 [in] the initial state.
	 * @param islandStateH [in] the new state.
	 * @return a vector block  with dimensions given by getDimVelocity().
	 */
	virtual Eigen::VectorXd getRHS(double stepsize, const rw::math::Vector3D<> &gravity, bool discontinuity, const std::list<RWPEConstraint*>& constraints0, const std::list<RWPEConstraint*>& constraintsH, const rw::kinematics::State &rwstate, const RWPEIslandState &islandState0, const RWPEIslandState &islandStateH) const;

	/**
	 * @brief Get the matrix block that relates the force/torque contribution of some arbitrary constraint
	 * to the motion in the constraint which is solved for.
	 * @param constraint [in] the constraint to get contribution matrix for.
	 * @param stepsize [in] the stepsize to use.
	 * @param discontinuity [in] first step after a discontinuity will be integrated and solved for differently.
	 * @param rwstate [in] the state.
	 * @param islandState [in] the state.
	 * @return a matrix block with dimensions given by getDimVelocity().
	 */
	virtual Eigen::MatrixXd getLHS(const RWPEConstraint* constraint, double stepsize, bool discontinuity, const rw::kinematics::State &rwstate, const RWPEIslandState &islandState) const;

	/**
	 * @brief The number of velocity constraints.
	 * @return the number of directions controlled in velocity mode.
	 */
	virtual std::size_t getDimVelocity() const = 0;

	/**
	 * @brief The number of wrench constraints.
	 * @return the number of directions controlled in wrench mode.
	 */
	virtual std::size_t getDimWrench() const = 0;

	/**
	 * @brief The number of force constraints.
	 * @return the number of directions controlled in free mode.
	 */
	virtual std::size_t getDimFree() const = 0;
	///@}

protected:
	/**
	 * @brief Construct a unspecified base constraint between two bodies.
	 * @param parent [in] the first object.
	 * @param child [in] the second object.
	 */
	RWPEConstraint(const RWPEBody* parent, const RWPEBody* child);

	/**
	 * @brief Get the contribution of another constraint force to this wrench-constraint.
	 * @param constraint the other constraint.
	 * @return matrix of dimensions n x 6, where n must be the number of directions set to Wrench mode (see getDimWrench).
	 * @note If not overridden by more specific subclasses, the wrench will be nonconstrained (the solver can set it to anything).
	 */
	virtual Eigen::MatrixXd getWrenchModelLHS(const RWPEConstraint* constraint) const;

	/**
	 * @brief Get the wrench-independent term of the wrench model.
	 * @return vector of dimensions n x 1, where n must be the number of directions set to Wrench mode (see getDimWrench).
	 */
	virtual Eigen::VectorXd getWrenchModelRHS() const;

	//! @brief The local position on the parent object from the center of mass.
	rw::math::Vector3D<> _posParent;
	//! @brief The local position on the child object from the center of mass.
	rw::math::Vector3D<> _posChild;
	//! @brief The local linear rotation on the parent.
	rw::math::Rotation3D<> _rotLinParent;
	//! @brief The local linear rotation on the child.
	rw::math::Rotation3D<> _rotLinChild;
	//! @brief The local angular rotation on the parent.
	rw::math::Rotation3D<> _rotAngParent;
	//! @brief The local angular rotation on the child.
	rw::math::Rotation3D<> _rotAngChild;

private:
	const RWPEBody* const _parent;
	const RWPEBody* const _child;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPECONSTRAINT_HPP_ */
