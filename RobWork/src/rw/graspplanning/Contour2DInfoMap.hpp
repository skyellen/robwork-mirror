#ifndef CONTOUR2D_INFO_MAP_HPP_
#define CONTOUR2D_INFO_MAP_HPP_

#include <rw/sensor/Contact2D.hpp>
#include <rw/geometry/Contour2D.hpp>

#include <rw/math/Constants.hpp>
#include <rw/math/Vector2D.hpp>

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
     * @param res [in] the resolution of the dicretisation of the
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
    bool _counterClock;

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
