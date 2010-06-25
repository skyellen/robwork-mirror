#ifndef CG3IKSOLVER2D_HPP_
#define CG3IKSOLVER2D_HPP_

#include "Grasp2D.hpp"

#include <rw/models/Joint.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/TreeDevice.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Q.hpp>

namespace rw {
namespace graspplanning {
/**
 * @brief solves the inverse kinematics problem for a SDH
 */
class CG3IKSolver2D {

public:

    typedef std::vector<std::pair<rw::math::Transform3D<>,rw::math::Q> > IKResult;

    /**
     * @brief constructor
     */
    CG3IKSolver2D(const rw::models::TreeDevice& device, const rw::kinematics::State& state);

    /**
     * @brief destructor
     */
    virtual ~CG3IKSolver2D(){};

    /**
     * @brief
     * @param bTgrasp
     * @param grasp
     * @param state
     * @return
     */
    IKResult solve(const rw::math::Transform3D<>& bTgrasp,
                   const Grasp2D& grasp, const rw::kinematics::State& state);

    /**
     * @brief sets the minimum required distance from hand base to grasp plane.
     * @param height [in] height from hand base to grasp plane in meters
     *
     */
    void setMinWristHeight(double height){
        _minWristHeight = height;
    }

    /**
     *
     * @return the minimum wrist height
     */
    double getMinWristHeight(){
        return _minWristHeight;
    }

private:
    void init(const rw::kinematics::State& state);

    const rw::models::TreeDevice& _device;
    double _maxWristToFingerTip;
    double _minWristHeight;

    rw::models::Joint *_thumbBase,*_thumbMid;
    rw::kinematics::Frame *_thumbTcp;
    rw::models::Joint *_f1Base,*_f1Mid;
    rw::kinematics::Frame *_f1Tcp;
    rw::models::Joint *_f2Base,*_f2Mid;
    rw::kinematics::Frame *_f2Tcp;

    double _aThum, _bThum;
    double _af1,_bf1;
    double _af2, _bf2;
};

}
}

#endif /*CG3IKSOLVER2D_HPP_*/
