#ifndef SBGraspPlanner3D_HPP_
#define SBGraspPlanner3D_HPP_

#include <rw/models/TreeDevice.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/math/Q.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <vector>
#include <rw/kinematics/MovableFrame.hpp>

#include "Grasp3D.hpp"

namespace rw {
namespace graspplanning {


/**
 * @brief a grasp planner for 3D objects.
 *
 */
class SBGraspPlanner3D {
public:
    /**
     * @brief configuration of pregrasp and actual grasp
     */
    struct GraspResult {
        GraspResult(const Grasp3D& g3d, const rw::math::Q& hQ, const rw::math::Transform3D<>& t3d):
            grasp(g3d),handQ(hQ),objThb(t3d){};
        // the grasp, contact, approach and so on
        Grasp3D grasp;
        // A configuration of the hand that will enable the grasp
        rw::math::Q handQ;
        // The transform from object to hand base
        rw::math::Transform3D<> objThb;

    };

    /**
     * @brief constructor
     * @param hand [in] the model of the SDH.
     * @param robot [in] the robot that the SDH is mounted on. It is assumed that
     * the SDH is mounted on the primary endeffector of the robot.
     * @param colDect [in] a collision detector for verifying collision free
     * configurations.
     */
    SBGraspPlanner3D(rw::models::TreeDevice* hand,
                     std::vector<std::vector<rw::models::Joint*> >& fingers,
                     rw::pathplanning::QSamplerPtr preshapeSampler,
                     const rw::kinematics::State& state);

    /**
     * @brief destructor
     */
    virtual ~SBGraspPlanner3D();


    /**
     * @brief resets the planner to plan grasps on \b object.
     * @param object
     * @param state
     */
    void reset(rw::kinematics::MovableFrame* object,const rw::kinematics::State& state);

    /**
     * @brief queries the planner to find a number of good grasp
     * configurations
     * @param minQuality [in] a quality threshold any grasp
     * @return a list of pairs of configurations. the first configuration is a pre-grasp
     * or approach configuration, the second is the actual grasp configuration.
     */
    std::vector<GraspResult> query(const rw::kinematics::State& state, int maxNrOfQs=100);

    /**
     * @param set the objects starting pose relative to the hand base.
     * The object will for each sample be placed in the pose \b bTo. Though
     * random rotations and translations are added to this init pose.
     * @param bTo
     */
    void setBaseToObjectPose(const rw::math::Transform3D<>& bTo){
        _bTo = bTo;
    }

    /**
     * @brief set the interval for which to generate random rotation samples.
     * Notice that because RPY is used the smapling will not be uniform.
     * @param roll
     * @param pitch
     * @param yaw
     */
    void setRandomRPY(double roll, double pitch, double yaw){
        _roll = roll;
        _pitch = pitch;
        _yaw = yaw;
    }

    int getProgress(){ return _progress; };

    std::pair<int,int> getProgressRange(){ return _progressRange; };

private:
    rw::models::TreeDevice *_hand;

    // object specific stuff
    rw::kinematics::MovableFrame *_obj;
    rw::math::Vector3D<> _objCM;
    double _objRadii;

    // hand specific stuff
    std::vector<std::vector<rw::models::Joint*> > _fingers;
    std::vector<std::vector<std::pair<rw::math::Q,rw::math::Q> > > _fingerBounds;

    std::vector<rw::kinematics::Frame*> _endEffFrames;
    std::vector<rw::kinematics::Frame*> _fingerTipFrames;

    rwlibs::proximitystrategies::ProximityStrategyPQP *_strategy;

    rw::pathplanning::QSamplerPtr _preshapeSampler;

    std::pair<int,int> _progressRange;
    int _progress;

    rw::math::Transform3D<> _bTo;

    double _roll,_pitch,_yaw;
};

}
}



#endif /*SDHGRASPPLANNER_HPP_*/
