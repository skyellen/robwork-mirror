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


#ifndef RWLIBS_ALGORITHMS_BASICGPM_HPP
#define RWLIBS_ALGORITHMS_BASICGPM_HPP

#include <rw/math/Q.hpp>
#include <rw/math/VelocityScrew6D.hpp>

#include <rw/kinematics/State.hpp>

namespace rw { namespace kinematics { class FKRange; } }
namespace rw { namespace models { class Device; } }
namespace rw { namespace models { class JacobianCalculator; } }
namespace rw { namespace models { class JointDevice; } }
namespace rw { namespace models { class TreeDevice; } }

namespace rwlibs {
namespace algorithms {

/**
 * @brief Implements a Gradient Projection Method (GPM)
 *
 * GPM implements the basic Gradient Projection Method with joint limit and
 * singularity avoidance presented in [1]: "Using the task function approach to
 * avoid robot joint limits and kinematic singularities in visual servoing"
 * by Marchand, Chaumette and Rizzo, IEEE/RSJ Int. Conf. on Intelligent Robots
 * and Systems, 1996, vol. 3, pp. 1083-1090.
 *
 *
 */
class BasicGPMM
{
public:

    /**
     * @brief Constructs BasicGPMM for TreeDevice. Uses the default
     * end effectors of the treedevice
     * @param device [in] Device to work with
     * @param state [in] Default state
     * @param qhome [in] Configuration somewhere between the lower and upper limit and towards which the joints should move
     * @param dt [in] Step size
     */
	BasicGPMM(const rw::models::TreeDevice* device,
              const rw::kinematics::State& state,
              const rw::math::Q& qhome,
 	          double dt);

    /**
     * @brief Constructs BasicGPMM for a
     * JointDevice(SerialDevice and TreeDevice). It does not use
     * the default end effectors. A list of interest frames are
     * given instead.
     * @param device [in] Device to work with
     * @param foi [in] The frames on the device to control. Usually this will be the tool, but not necessarily.
     * @param state [in] Default state
     * @param qhome [in] Configuration somewhere between the lower and upper limit and towards which the joints should move
     * @param dt [in] Step size
     */
	BasicGPMM(const rw::models::JointDevice* device,
			  const std::vector<rw::kinematics::Frame*>& foi,
			  const rw::kinematics::State& state,
			  const rw::math::Q& qhome,
			  double dt);

	/**
	 * @brief Destructor
	 */
	virtual ~BasicGPMM();

	/**
	 * @brief Solves for joint velocities given a desired tool velocity
	 *
	 * @param q [in] The current joint configuration
	 * @param dq [in] The current joint velocity
	 * @param tcpvel [in] The desired tool velocity seen in the base frame
	 */
	rw::math::Q solve(const rw::math::Q& q, const rw::math::Q& dq, const std::vector<rw::math::VelocityScrew6D<> >& tcpvel);

	/**
	 * @brief Specifies whether to use joint limit avoidance
	 *
	 * The method implements the method with Activation threshold from the [1]
	 *
	 * @param use [in] True to use joint limit avoidance
	 */
	void setUseJointLimitsCost(bool use);

	/**
	 * @brief Specifies whether to use singularity avoidance
	 *
	 * The singularity avoidance as described in [1]
	 *
	 * @param use [in] True to use singularity avoidance
	 */
	void setUseSingularityCost(bool use);

	/**
	 * @brief Sets the weight of the joint limits
	 *
	 * @param w [in] Weight of the joint limit
	 */
	void setJointLimitsWeight(double w);

	/**
	 * @brief Sets the threshold for the joint limits
	 *
	 * Given an upper and a lower bound \f$upper\f$ and \f$lower\f$ the proximity of the joint
	 * limits is defined as \f$upper-\tau_{max} (upper-lower)\f$ and \f$lower+\tau_{min} (upper-lower)\f$
	 * where \f$\tau_{min}\f$ and \f$\tau_{max}\f$ are the thresholds specified here.
	 *
	 * @param thresholdLowerRatio [in] Relative threshold for the lower joint limits
	 * @param thresholdUpperRatio [in] Relative threshold for the upper joint limits
	 */
	void setJointLimitThreshold(double thresholdLowerRatio, double thresholdUpperRatio);

	/**
	 * @brief Sets the weight of the singularity avoidance task
	 *
	 * @param w [in] The weight
	 */
	void setSingularityWeight(double w);

   /**
     * @brief Enumeration used to specify frame associated with the projection
     */
    enum ProjectionFrame { BaseFrame = 0, /** Robot Base Frame */
                           ControlFrame  /**The Frame specified as the controlFrame*/
                          };

    /**
     * @brief Setup the projection
     *
     * The traditional relationship between device Jacobian, joint velocities and tool velocity
     * is given by \f$J\dot{q}=\dot{q}\f$. To ignore certain degrees of freedom or put more
     * emphasis (with respect to the least square solution) on some we can multiply with
     * \f$P\f$ to get \f$P J\dot{q}=P \dot{x}\f$.
     *
     * \f$P\f$ needs to have exactly 6 columns, however the number of row may be less than 6.
     * Use the \b space flag to specify in which space the projection should occur.
     *
     * Usage: Setup to ignore tool roll
     * \code
     * BasicGPM* gpm = new BasicGPM(device, device->getEnd(), state, qhome, dt)
     * boost::numeric::ublas::matrix<double> P = boost::numeric::ublas::zero_matrix<double>(5,6);
     * for (int i = 0; i<5; i++)
     *     P(i,i) = 1;
     * gpm->setProjection(P, BasicGPM::ControlFrame);
     * \endcode
     *
     * @param P [in] The projection matrix
     * @param space [in] The space in which to apply the projection
     */
    void setProjection(const Eigen::MatrixXd& P, ProjectionFrame space);

private:
    const rw::models::Device* _device;

    rw::common::Ptr<rw::models::JacobianCalculator> _jacCalc;
    std::vector<rw::kinematics::Frame*> _foi; // frames of interest, end frames
    std::vector<boost::shared_ptr<rw::kinematics::FKRange> > _fkranges;

    rw::kinematics::State _state;
    rw::math::Q _qhome;
    int _dof;
    double _dt;
    rw::math::Q _qlower;
    rw::math::Q _qupper;
    rw::math::Q _dqlimit;
    rw::math::Q _ddqlimit;


    Eigen::MatrixXd _P;
    ProjectionFrame _space;

    //TODO Rewrite into using the PropertyMap
    bool _useJointLimitsCost;
    bool _useSingularityCost;

    double _weightSingularity;
    double _weightJointLimits;

    rw::math::Q _thresholdLower;
    rw::math::Q _thresholdUpper;

    Eigen::VectorXd getCostGradient(const rw::math::Q& q, const Eigen::MatrixXd& jac);

    rw::math::Q applyJointVelocityConstraint(const rw::math::Q& q, const rw::math::Q& dq, const rw::math::Q& dqnew);

    void calculatePosAndVelLimits(Eigen::VectorXd& lower,
                                  Eigen::VectorXd& upper,
                                  const rw::math::Q& q,
                                  const rw::math::Q& dq);


};

} //end namespace algorithms
} //end namespace rw

#endif /*RWLIBS_ALGORITHMS_BASICGPM_HPP*/
