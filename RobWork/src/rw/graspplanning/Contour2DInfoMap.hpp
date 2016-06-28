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

#ifndef RW_GRASPPLANNING_CONTOUR2DINFOMAP_HPP_
#define RW_GRASPPLANNING_CONTOUR2DINFOMAP_HPP_

#include <rw/sensor/Contact2D.hpp>
#include <rw/geometry/Contour2D.hpp>

namespace rw {
namespace graspplanning {

/**
 * @brief class for analysing 2d contours.
 */
class Contour2DInfoMap {
public:
    //! 2d contact pointer list
    typedef std::vector<rw::sensor::Contact2D*> ContactPtrList;

    /**
     * @brief constructor
     * @param resolution [in] the resolution of the dicretisation of the
     * polar coordinates.
     */
    Contour2DInfoMap(int resolution=100);

    /**
     * @brief destructor
     */
    virtual ~Contour2DInfoMap(){};

    /*
     * @brief gets the contact at angle angle from the direction vector (1,0)
     */
    //const Contact& getContact(double angle);

    /**
     * @brief get all contacts that has a normal with an angle nAngle +-thres
     * relative to the direction vector (1,0).
     */
    ContactPtrList getCNormals(double nAngle, double margin, double thres);

    /**
     * @brief initialize the contour2d map with a contour
     */
    void reset(const rw::geometry::Contour2D& contor);

    /**
     * @brief writes this contour information to file
     * @param file [in] name of file
     */
    void printToFile(const std::string& file);

    /**
     * @brief get min curvature of contour
     * @return min curvature
     */
    double getMinCurvature(){
        return _minCurvature;
    }

    /**
     * @brief get max curvature of contour
     * @return max curvature
     */
    double getMaxCurvature(){
        return _maxCurvature;
    }

    /**
     * @brief get average curvature of contour
     * @return average curvature
     */
    double getAvgCurvature(){
        return _avgCurvature;
    }

    /**
     * @brief get the 2d contour
     * @return 2d contour
     */
    const rw::geometry::Contour2D& getContour(){
        return *_contour;
    }

private:

    //int getContactIdx(double angle);

    const rw::geometry::Contour2D* _contour;
    const int _res;
    const double _resStep;
    const double _resStepInv;
    double _minCurvature,_maxCurvature,_avgCurvature;
    //double _resStep;

    // contact info per contour point
    std::vector<rw::sensor::Contact2D> _contacts;

    // mapping between normal direction and contacts
    std::vector<ContactPtrList > _normalToContactsMap;

    //rw::math::Vector2D<> _center;
    //std::vector<double> _distMap;
    //std::vector<rw::math::Vector2D<> > _contactPoints;
    //std::vector<double> _curvatureMap;

};


}
}

#endif /*CONTOURINFOMAP_HPP_*/
