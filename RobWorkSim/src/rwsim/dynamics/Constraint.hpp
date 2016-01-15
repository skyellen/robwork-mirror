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

#ifndef RWSIM_DYNAMICS_CONSTRAINT_HPP_
#define RWSIM_DYNAMICS_CONSTRAINT_HPP_

/**
 * @file Constraint.hpp
 *
 * \copydoc rwsim::dynamics::Constraint
 */

#include <rw/kinematics/StateData.hpp>
#include <rw/math/Transform3D.hpp>

#include <boost/thread/mutex.hpp>

namespace rwsim {
namespace dynamics {

// Forward declarations
class Body;

//! @addtogroup rwsim_dynamics
//! @{

/**
 * @brief A constraint is a mathematical model that constrain the movement
 * between two arbitrary bodies in a dynamic simulation.
 *
 * For a device the Link and Joint types should be used to specify the constraints.
 *
 * The Constraint type can however be used between any two bodies. For instance between two
 * free bodies, or between a free body and a link of a device.
 *
 * All functions in this class are thread-safe. Only the transform of the constraint and the spring
 * parameters can be changed after creation of the constraint.
 */
class Constraint: public rw::kinematics::StateData {
public:
    //! @brief smart pointer type to this class
	typedef rw::common::Ptr<Constraint> Ptr;

	//! @brief The different constraint types.
	typedef enum {
		Fixed,             //!< bodies will not be able to move relative to eachother (0 DOF).
		Prismatic,         //!< bodies will be able to move in the z-direction only (1 DOF).
		Revolute,          //!< bodies will be able to rotate around the z-direction only (1 DOF).
		Universal,         //!< bodies will be able to both rotate around the x- and y- directions (2 DOF).
		Spherical,         //!< bodies will be able to rotate relative to eachother, but not move linearly (3 DOF).
		Piston,            //!< bodies will be able to translate and rotate around the z-axis (2 DOF).
		PrismaticRotoid,   //!< bodies will be able to translate along the z-axis and rotate around the x-axis (2 DOF).
		PrismaticUniversal,//!< bodies will be able to translate along the z-axis and rotate around both the x- and y-axes (3 DOF).
		Free               //!< bodies are not constrained (6 DOF) - can be used for 6D springs.
	} ConstraintType;

	/**
	 * @brief Construct new constraint.
	 * @param name [in] a unique name identifying the constraint.
	 * @param type [in] the type of constraint.
	 * @param b1 [in] the parent body to attach the constraint to.
	 * @param b2 [in] the child body to attach the constraint to.
	 */
	Constraint(const std::string& name, const ConstraintType &type, Body* b1, Body* b2);

	//! @brief Destructor
	virtual ~Constraint();

	/**
	 * @brief Get the type of constraint.
	 * @return the ConstraintType
	 */
	ConstraintType getType() const;

	/**
	 * @brief Get a pointer to Body 1.
	 * @return pointer to body 1.
	 */
	Body* getBody1() const;

	/**
	 * @brief Get a pointer to Body 2.
	 * @return pointer to body 2.
	 */
	Body* getBody2() const;

	/**
	 * @brief Get the degrees of freedom of the constraint.
	 * @return the degrees of freedom.
	 */
	size_t getDOF() const;

	/**
	 * @brief Get the linear degrees of freedom of the constraint.
	 * @return the linear degrees of freedom.
	 */
	size_t getDOFLinear() const;

	/**
	 * @brief Get the angular degrees of freedom of the constraint.
	 * @return the angular degrees of freedom.
	 */
	size_t getDOFAngular() const;

	/**
	 * @brief Get the transform of the constraint, relative to the parents body frame.
	 * @return the transform relative to the parent, \f$\bf{T}_{parent}^{constraint}\f$.
	 */
	rw::math::Transform3D<> getTransform() const;

	/**
	 * @brief Set the transform of the constraint, relative to the parents body frame.
	 * @param parentTconstraint [in] the transform relative to the parent, \f$\bf{T}_{parent}^{constraint}\f$.
	 */
	void setTransform(const rw::math::Transform3D<> &parentTconstraint);

	/**
	 * @brief Parameters for a spring.
	 *
	 * @note The compliance should under normal circumstances be symmetric positive definite.
	 * The compliance can be semi-definite under special conditions.
	 * In this case the infinitely stiff directions are automatically controlled with a zero
	 * velocity instead of applying infinite forces that would cause instability.
	 * It is however only possible to set the linear and angular velocity independently.
	 * Therefore the null-space must be spanned by vectors that selects
	 * only linear or angular parts (and not a combination).
	 */
	struct SpringParams {
		//! @brief Constructor
		SpringParams(): enabled(false) {};
		//! @brief Enable or disable the spring.
		bool enabled;
		//! @brief The compliance of the spring as a n x n matrix, where n is the DOF of the constraint.
		Eigen::MatrixXd compliance;
		//! @brief The damping of the spring as a n x n matrix, where n is the DOF of the constraint.
		Eigen::MatrixXd damping;
	};

	/**
	 * @brief Get the spring parameters.
	 * @return SpringParams structure.
	 */
	SpringParams getSpringParams() const;

	/**
	 * @brief Set spring parameters.
	 * @param params [in] SpringParams structure.
	 */
	void setSpringParams(const SpringParams &params);

	//! @brief Definition of a limit for one single degree of freedom.
	struct Limit {
		//! @brief Constructor.
		Limit(): lowOn(false), highOn(false), low(0), high(0) {}
		//! @brief Enable low limit.
		bool lowOn;
		//! @brief Enable high limit.
		bool highOn;
		//! @brief Value for low limit.
		double low;
		//! @brief Value for high limit.
		double high;
	};

	/**
	 * @brief Get limit information.
	 * @param i [in] the degree of freedom to get limit information for.
	 * @return the limit.
	 */
	Limit getLimit(std::size_t i) const;

	/**
	 * @brief Set limit.
	 * @param i [in] the degree of freedom to set limit information for.
	 * @param limit [in] the limit information.
	 */
	void setLimit(std::size_t i, const Limit& limit);

	/**
	 * @brief Convert a string to a ConstraintType.
	 * @param string [in] the string to convert.
	 * @param type [in/out] the result of the convertion.
	 * @return true if string was converted, false otherwise.
	 */
	static bool toConstraintType(const std::string &string, ConstraintType &type);

private:
	static size_t getDOF(ConstraintType type);
	static size_t getDOFLinear(ConstraintType type);
	static size_t getDOFAngular(ConstraintType type);

	const ConstraintType _type;
	Body* const _body1;
	Body* const _body2;

	rw::math::Transform3D<> _parentTconstraint;
	mutable boost::mutex _parentTconstraint_mutex;

	SpringParams _springParams;
	mutable boost::mutex _springParams_mutex;

	std::vector<Limit> _limits;
	mutable boost::mutex _limits_mutex;
};
//! @}
} /* namespace dynamics */
} /* namespace rwsim */
#endif /*RWSIM_DYNAMICS_CONSTRAINT_HPP_*/
