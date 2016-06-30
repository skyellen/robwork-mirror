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


#ifndef RWLIBS_ALGORITHMS_NULLSPACEPROJECTION_HPP
#define RWLIBS_ALGORITHMS_NULLSPACEPROJECTION_HPP

#include <rw/math/Q.hpp>
#include <rw/kinematics/State.hpp>

#include <boost/numeric/ublas/matrix.hpp>

namespace rw { namespace kinematics { class Frame; } }
namespace rw { namespace models { class Device; } }

namespace rwlibs {
namespace algorithms {

/**
 * @brief Performs a projection in the null space of the device Jacobian to move joints away from
 * singularities.
 *
 * Given a device with redundant degrees of freedom, the null space of the Jacobian can be used to
 * move joints away from their limits. The problem of finding an optimal correction is formulated
 * as a quadratic optimization problem in which joint position, velocity and acceleration limits are
 * formulated as in the QP/XQP method.
 *
 * The basic NullSpaceProjection assumes all 6 degrees of freedom of the tool needs to be constrainted.
 */
class NullSpaceProjection
{
public:
    /**
     * @brief Construct NullSpaceProjection
     * @param device [in] Device to consider
     * @param controlFrame [in] Frame for which to calculate the Jacobian
     * @param state [in] State giving the assembly of the workcell
     * @param dt [in] Time step size
     */
    NullSpaceProjection(rw::models::Device* device,
                        rw::kinematics::Frame* controlFrame,
                        const rw::kinematics::State& state,
                        double dt);

    /**
     * @brief Destructor
     */
	virtual ~NullSpaceProjection();

	/**
	 * @brief Solves to give a joint motion moving away from joint limits while satisfying the main task.
	 *
	 * Usage:
	 * \code
	 * NullSpaceProjection nps(device, device->getEnd(), state);
	 * ...
	 * ...
	 * \\input Current configuration q, current velocity dq and desired new velocity dq1
	 * Q qns = nps.solve(q, dq, dq1);
	 * dq = dq1+qns;
	 * \endcode
	 *
	 * @param q [in] Configuration of the device
	 * @param dqcurrent [in] The current velocity
	 * @param dq1 [in] The new velocity calculated e.g. by the XQPController
	 */
	rw::math::Q solve(const rw::math::Q& q, const rw::math::Q& dqcurrent, const rw::math::Q& dq1);


	/**
     * @brief Enumeration used to specify frame associated with the projection
     */
    enum ProjectionFrame { BaseFrame = 0, /** Robot Base Frame */
	                       ControlFrame  /**The Frame specified as the controlFrame*/
	                      };

   /**
     * @brief Specifies an initial projection of the Jacobian before calculating the null-space
     *
     * Given a projection matric \f$P\f$ it is multiplied with the device Jacobian as $\f$P J\f$. This
     * can be used to ignore degrees of freedom such as tool rool.
     *
     * \see XQPController::setProjection
     *
     * Usage: Setup for system ignoring tool roll
     * \code
     * XQPController* xqp = new XQPController(device, device->getEnd(), state, dt)
     * boost::numeric::ublas::matrix<double> P = boost::numeric::ublas::zero_matrix<double>(5,6);
     * for (int i = 0; i<5; i++)
     *     P(i,i) = 1;
     * xqp->setProjection(P, XQPController::ControlFrame);
     * \endcode
     *
     * @param P [in] The projection matrix
     * @param space [in] The space in which to apply the projection
     */
    void setProjection(const boost::numeric::ublas::matrix<double>& P, ProjectionFrame space);


    /**
     * @brief Sets the threshold for the joint limits
     *
     * Given an upper and a lower bound \f$upper\f$ and \f$lower\f$ the proximity of the joint
     * limits is defined as \f$upper-\tau (upper-lower)\f$ where \f$\tau\f$ is the threshold
     * specified here.
     *
     * @param threshold [in] Relative threshold for the joint limits
     */
	void setThreshold(double threshold);


	/**
	 * @brief Sets the weight of the joint limits
	 *
	 * @param w [in] Weight of the joint limit
	 */
	void setJointLimitsWeight(double w);


private:
    /**
     * Calculate gradient for joint limit cost function
     */
    rw::math::Q getGradient(const rw::math::Q& q);

    /**
     * Calculate velocity limits associated with position, velocity and acceleration limits as
     * in the QPController
     */
    void calculateVelocityLimits(rw::math::Q& lower,
                                 rw::math::Q& upper,
                                 const rw::math::Q& q,
                                 const rw::math::Q& dq);


    rw::models::Device* _device;
    rw::kinematics::Frame* _controlFrame;
    rw::kinematics::State _state;
    int _dof;
    double _dt;

    rw::math::Q _qlower;
    rw::math::Q _qupper;
    rw::math::Q _dqlimit;
    rw::math::Q _ddqlimit;
    rw::math::Q _thresholdLower;
    rw::math::Q _thresholdUpper;
    boost::numeric::ublas::matrix<double> _P;
    ProjectionFrame _space;

	double _weightJointLimits;
};

} //end namespace algorithms
} //end namespace rws

#endif /*RWLIBS_ALGORITHMS_NULLSPACEPROJECTION_HPP*/
