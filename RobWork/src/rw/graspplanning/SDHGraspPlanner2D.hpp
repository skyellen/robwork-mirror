#ifndef SDHGRASPPLANNER2D_HPP_
#define SDHGRASPPLANNER2D_HPP_

#include <rw/geometry/Contour2D.hpp>

#include "CG3Grasp2DGen.hpp"

#include <rw/models/TreeDevice.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/math/Q.hpp>
#include <vector>

namespace rw {
namespace graspplanning {

/**
 * @brief a grasp planner for the 3 finger SDH (Schunk Dextreous Hand).
 * The grasp planner use 2d contour information to plan good grasp
 * configurations for both hand and robot.
 *
 */
class SDHGraspPlanner2D {


public:
    /**
     * @brief configuration of pregrasp and actual grasp
     */
    typedef std::pair<rw::math::Q,rw::math::Q> GraspResult;

    /**
     * @brief constructor
     * @param hand [in] the model of the SDH.
     * @param robot [in] the robot that the SDH is mounted on. It is assumed that
     * the SDH is mounted on the primary endeffector of the robot.
     * @param colDect [in] a collision detector for verifying collision free
     * configurations.
     */
    SDHGraspPlanner2D(const rw::models::TreeDevice& hand,
                      rw::models::JointDevice& robot,
                      rw::proximity::CollisionDetector& colDect,
                      const rw::kinematics::State& state);

    /**
     * @brief constructor
     * @param hand [in] the model of the SDH.
     * @param robot [in] the robot that the SDH is mounted on. It is assumed that
     * the SDH is mounted on the primary endeffector of the robot.
     * configurations.
     */
    SDHGraspPlanner2D(const rw::models::TreeDevice& hand,
                      rw::models::JointDevice& robot,
                      const rw::kinematics::State& state);

    /**
     * @brief destructor
     */
    virtual ~SDHGraspPlanner2D(){};

    /**
     * @brief sets the contour that describe the object that is to be grasped.
     * A transform must be supplied that describe the transform from robot base to
     * contour reference frame.
     * @param contour [in] the 2d contour that is to be processed.
     * @param baseTcontour [in] transform from robot base to contour reference frame.
     */
    void setContour(const rw::geometry::Contour2D& contour, const rw::math::Transform3D<>& baseTcontour);

    /**
     * @brief queries the planner to find a number of good grasp
     * configurations
     * @param minQuality [in] a quality threshold any grasp
     * @return a list of pairs of configurations. the first configuration is a pre-grasp
     * or approach configuration, the second is the actual grasp configuration.
     */
    std::vector<GraspResult> query(const rw::kinematics::State& state, int maxNrOfQs=1);

    /**
     * @brief sets the allowed minimum quality of any grasp solution.
     * @param minQuality [in] between [0;1] where 1 is highest quality.
     *
     * @note this parameter is usefull when trying to grasp objects that has
     * no really good grasps. Lowering the minimum quality requirement will
     * produce more solutions.
     */
    void setMinGraspQuality(double minQuality){_minQuality=minQuality;};

    /**
     * @brief sets the approach clearance in meters. The approach clearance is
     * used to calculate the pre-grasp or approach configuration which is a configuration
     * close to the actual grasp-contact configuration. The clearance specify how far from
     * the object-contour the approach grasp configuration should be.
     * @param clerace [in] clerance of approach grasp contacts from object
     * contour in meters
     */
    void setApproachClearance(double clerance){
        _clerance = clerance;
    }

    /**
     * @brief a max limit on the height of the objects is needed
     * since only 2d contours of the object to be grasped is used in
     * the planning.
     * @param objHeight [in] max height of object in meters
     */
    void setMaxObjectHeight(double objHeight){
        _maxObjHeight = objHeight;
    }

    /***
     * @brief set the height to grasp the object in.
     */
    void setGraspHeight(double graspHeight){
        _graspHeight = graspHeight;
    }

    /**
     * @brief sets the threshold values for the contact candidate filter.
     * @param maxCurvature [in] sets the max Curvature allowed for contacts [0;Pi/2]
     * where 0 is flat surface
     * @param maxAppAngle [in] sets the max allowed angle between surface normal
     * and grasp approach vectors [0;Pi]
     *
     */
    void setFilter(double maxCurvature, double maxAppAngle){
        _graspGen.setMaxCurvature(maxCurvature);
        _graspGen.setPerpFilter(maxAppAngle);
    }

    /**
     * @brief gets the last list of generated candidate grasps
     * @return
     */
    const std::vector<Grasp2D>& getGrasps(){
        return _grasps;
    }


    CG3Grasp2DGen& getGraspGenerator(){
        return _graspGen;
    }

private:
    const rw::models::TreeDevice &_hand;
    rw::models::JointDevice &_robot;
    CG3Grasp2DGen _graspGen;
    rw::proximity::CollisionDetector *_detector;
    rw::math::Transform3D<> _hbTwrist,_baseTcontour;
    double _clerance;
    double _maxObjHeight;
    double _minQuality;
    double _graspHeight;
    double _maxCurvature, _maxAppAngle;

    std::vector<rw::geometry::Contour2D> _contours;
    std::vector<Grasp2D> _grasps;

};

}
}



#endif /*SDHGRASPPLANNER_HPP_*/
