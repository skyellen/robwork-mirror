#include "IKSoftCSolver.hpp"

#include <rw/rw.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/TreeDevice.hpp>

#include <map>
#include <vector>


/**
 * @brief an iterative and heuristics based inverse kinematics solver specifically for the
 * Schunk Dexterous Hand.
 *
 * It is assumed that the pose of the hand-base is part of the configuration to solve. So 7
 * dof of the fingers plus the 6 dof of cartesean space is used to find a solution for the targets.
 *
 *
 */
class SDHInvKinSolver{
public:

    SDHInvKinSolver();

    virtual ~SDHInvKinSolver(){};

    /**
     * @brief solve inverse kinematics for hand but make the hand approach from a specific
     * direction. The approach
     * @param baseTend [in] 3 unordered targets. All combinations are investigated.
     * @param approach [in] the direction of the z-axis of the base of the hand.
     * @param state [in] current state.
     * @return
     */
    std::vector<boost::tuple<rw::math::Transform3D<>, rw::math::Q, bool> > solve(const std::vector<rw::math::Transform3D<> >& baseTend,
                                   const rw::math::Vector3D<>& approach) const;

private:
    rw::models::WorkCell::Ptr _wc;
    rw::models::TreeDevice::Ptr _sdh;
    mutable rw::kinematics::State _defState;
    IKSoftCSolver *_iksolver;
    rw::invkin::JacobianIKSolverM *_jiksolver;
    rw::invkin::JacobianIKSolverM *_jiksolver2f;
    rw::kinematics::MovableFrame* _sdhbase;
    rw::models::Device::Ptr _se3dev;
    mutable rw::trajectory::TimedStatePath _path;
    mutable int _nrReducedTargets;
    rw::math::Transform3D<> _baseTtcp;
};
