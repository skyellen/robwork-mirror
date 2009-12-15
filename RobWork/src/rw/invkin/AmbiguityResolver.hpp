#ifndef RW_INVKIN_AMGIGUITYRESOLVER_HPP
#define RW_INVKIN_AMGIGUITYRESOLVER_HPP

#include "InvKinSolver.hpp"

#include <rw/models/JointDevice.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/math/Transform3D.hpp>

namespace rw {
namespace invkin {

/** @addtogroup invkin */
/*@{*/

/**
 * @brief Wraps a InvKinSolver and searches for ambiguities due to joint able to rotate \f$2\pi\f$ or more.
 *
 * For each solution \f$\mathbf{q}\f$ the method tries to see if a \f$j\f$ exists s.t. 
 * \f$\mathbf{q}(i)=\mathbf{q}(i)+j*2\pi\f$ is a valid solution.
 *
 * The AmbiguityResolver always tests for joint limits.
 */
class AmbiguityResolver: public InvKinSolver
{
public:
    /**
     * @brief Constructs an AmbiguityResolver
     * @param invkin [in] The inverse kinematics solver to obtain solutions from
     */
    AmbiguityResolver(const InvKinSolverPtr& invkin, rw::models::JointDevicePtr device);

    /**
     * @brief Destructor
     */
    ~AmbiguityResolver(void);

    /**
     * @brief Calls the InvKinSolver provided and resolves ambiguities.
     * @copydoc InvKinSolver::solve
     */
    virtual std::vector<math::Q> solve(const rw::math::Transform3D<>& baseTend, const rw::kinematics::State& state) const;

    /** 
     * @brief No effect. The AmbiguityResolver always tests for joint limits.
     */
    virtual void setCheckJointLimits(bool check);


private:
    InvKinSolverPtr _invkin;
    rw::models::DevicePtr _device;
    std::vector<size_t> _indices;
    rw::math::Q _lower;
    rw::math::Q _upper;
    
};

/** @} */

} //end namespace invkin
} //end namespace rw

#endif //#ifndef RW_INVKIN_AMGIGUITYRESOLVER_HPP
