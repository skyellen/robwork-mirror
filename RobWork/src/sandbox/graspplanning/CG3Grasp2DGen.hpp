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

#ifndef RW_GRASPPLANNING_CG3GRASP2DGEN_HPP_
#define RW_GRASPPLANNING_CG3GRASP2DGEN_HPP_

#include <rw/math/Vector2D.hpp>
#include <rw/models/TreeDevice.hpp>
#include <rw/kinematics/State.hpp>

#include "Grasp2D.hpp"

#include "QualityMeasure2D.hpp"
#include "Contour2DInfoMap.hpp"

namespace rw {
namespace graspplanning {

/**
 * @brief generates good grasp contacts for a 3 finger schunk hand
 */
class CG3Grasp2DGen {

public:
    /**
     * @brief constructor
     * @param hand [in] model of the hand used for grasping
     * @param thetaRes [in] resolution of the discretization
     * @param
     */
    CG3Grasp2DGen(
        const rw::models::TreeDevice& hand,
        const rw::kinematics::State& state,
        int thetaRes=100, bool counterclock=true);

    //! @brief destructor
    virtual ~CG3Grasp2DGen() {};

    /**
     * @brief initialize this contact generator using a 2d contour
     */
    void init(
        const rw::geometry::Contour2D& contour,
        int psiRes, int phiRes);

    /**
     * @brief computes a list of the best grasps in the grasp candidate list.
     */
    std::vector<Grasp2D>
        computeGrasps(const QualityMeasure2D& measure, double minQuality, const unsigned int nrOfGrasps);

    /**
     * @brief gets grasp candidate at idx
     */
    Grasp2D getGrasp(int idx);

    /**
     * @brief sets the maximum allowed distance that the projection of thumb
     * contact onto the vector between the other two fingers can deviate from
     * the center of the vector. [0;0.5]
     *
     */
    void setUniformFilter(double acceptUniform){
        _acceptUniform = acceptUniform;
    }

    /**
     * @brief return the uniform filter threshold
     */
    double getUniformFilter(){
        return _acceptUniform;
    }

    /**
     * @brief set max curvature threshold filter
     * @param curvThres
     */
    void setMaxCurvature(double curvThres){
        _sqrCurvThres = curvThres*curvThres;
    }

    /**
     * @brief get the max curvature threshold filter
     * @return
     */
    double getMaxCurvature() const {
        return sqrt(_sqrCurvThres);
    }

    /**
     * @brief sets the maximum allowed angle
     */
    void setPerpFilter(double acceptPerp){
        _acceptPerp = acceptPerp;
    }

    /**
     * @brief
     * @return
     */
    double getPerpFilter() const{
        return _acceptPerp;
    }

    /**
     * @brief set the maximum allowed angle between thumb approach and the vector between
     * the two other fingers. [0;Pi/2]
     */
    void setDirFilter(double direction){
        _acceptDirs = cos(direction);
    }

    /**
     * @brief get the direction filter threshold
     * @return
     */
    double getDirFilter() const{
        return _acceptDirs;
    }

    /**
     * @brief sets the maximum allowed distance between any fingers
     * in meters.
     */
    void setMaxGraspWidth(double maxWidth){
        _maxGraspWidth = maxWidth;
    }

    /**
     * @brief get the maximum grasp width
     * @return max grasp width
     */
    double getMaxGraspWidth() const{
        return _maxGraspWidth;
    }

    /**
     * @brief get the contour on which the grasp is planned
     * @return 2d contour
     */
    const rw::geometry::Contour2D& getContour(){
        return _infoMap.getContour();
    }

private:
    const rw::models::TreeDevice& _hand;
    rw::kinematics::State _state;
    bool _counterClock;
    int _thetaRes,_psiRes,_phiRes;
    double _thetaStep,_psiStep,_phiStep;
    double  _sqrCurvThres;
    double _acceptUniform;
    double _acceptPerp;
    double _acceptDirs;
    double _maxGraspWidth;

    double _L, _sqrL;
    double _h, _w;

    // params for filter rules


    rw::math::Vector2D<> _center;
    rw::math::Vector2D<> _f1Dir,_f2Dir,_thumDir;

    Contour2DInfoMap _infoMap;

    std::vector<Grasp2D> _graspCandidates;
};

}
}

#endif /*CG3Grasp2DGen_HPP_*/
