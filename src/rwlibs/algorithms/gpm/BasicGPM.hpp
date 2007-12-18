#ifndef RWLIBS_ALGORITHMS_BASICGPM_HPP_
#define RWLIBS_ALGORITHMS_BASICGPM_HPP_

#include <rw/math/Q.hpp>
#include <rw/math/VelocityScrew6D.hpp>

#include <rw/models/DeviceModel.hpp>
#include <rw/kinematics/State.hpp>

#include <boost/numeric/ublas/vector.hpp>

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
class BasicGPM
{
public:
    /**
     * @brief Constructs BasicGPM object
     * 
     * @param device [in] Device to work with
     * @param controlFrame [in] The frame on the device to control. Usually this will be the tool, but not necessarily.
     * @param state [in] Default state
     * @param qhome [in] Configuration somewhere between the lower and upper limit and towards which the joints should move
     * @param dt [in] Step size
     */
	BasicGPM(rw::models::DeviceModel* device, 
	         rw::kinematics::Frame* controlFrame,
	         const rw::kinematics::State& state, 
	         const rw::math::Q& qhome, 
	         double dt);

	/**
	 * @brief Destructor
	 */
	virtual ~BasicGPM();
	
	/**
	 * @brief Solves for joint velocities given a desired tool velocity
	 * 
	 * @param q [in] The current joint configuration
	 * @param dq [in] The current joint velocity
	 * @param tcpvel [in] The desired tool velocity seen in the base frame 
	 */
	rw::math::Q solve(const rw::math::Q& q, const rw::math::Q& dq, const rw::math::VelocityScrew6D<>& tcpvel);   
	
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
	 * limits is defined as \f$upper-\tau (upper-lower)\f$ where \f$\tau\f$ is the threshold
	 * specified here. 
	 * 
	 * @param threshold [in] Relative threshold for the joint limits 
	 */
	void setJointLimitThreshold(double threshold);
	
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
    void setProjection(const boost::numeric::ublas::matrix<double>& P, ProjectionFrame space);

private:
    rw::models::DeviceModel* _device;
    rw::kinematics::Frame* _controlFrame;
    rw::kinematics::State _state;
    rw::math::Q _qhome;
    int _dof;
    double _dt;
    rw::math::Q _qlower;
    rw::math::Q _qupper;
    rw::math::Q _dqlimit;
    rw::math::Q _ddqlimit;  
    
    
    boost::numeric::ublas::matrix<double> _P;
    ProjectionFrame _space;
    
    //TODO Rewrite into using the PropertyMap
    bool _useJointLimitsCost;
    bool _useSingularityCost;
    
    double _weightSingularity;
    double _weightJointLimits;
    
    rw::math::Q _thresholdLower;
    rw::math::Q _thresholdUpper;
    
    boost::numeric::ublas::vector<double> getCostGradient(const rw::math::Q& q, 
                                                          const boost::numeric::ublas::matrix<double>& jac);
        
    rw::math::Q applyJointVelocityConstraint(const rw::math::Q& q, const rw::math::Q& dq, const rw::math::Q& dqnew); 

    void calculatePosAndVelLimits(boost::numeric::ublas::vector<double>& lower,
                                  boost::numeric::ublas::vector<double>& upper,
                                  const rw::math::Q& q,
                                  const rw::math::Q& dq);

    
};

} //end namespace algorithms
} //end namespace rw

#endif /*RWLIBS_ALGORITHMS_BASICGPM_HPP_*/
