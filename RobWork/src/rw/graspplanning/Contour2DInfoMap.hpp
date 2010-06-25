#ifndef CONTOUR2D_INFO_MAP_HPP_
#define CONTOUR2D_INFO_MAP_HPP_

#include <rw/sensor/Contact2D.hpp>
#include <rw/geometry/Contour2D.hpp>

#include <rw/math/Constants.hpp>
#include <rw/math/Vector2D.hpp>

namespace rw {
namespace graspplanning {


class Contour2DInfoMap {
public:
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

    /**
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

    void printToFile(const std::string& file){
        FILE *cfile = fopen(file.c_str(), "w");

        if (cfile== NULL) {
            perror( "Can't create img_file_name");
            return;
        }

        for(size_t i=0;i<_contacts.size();i++){
            rw::sensor::Contact2D &c = _contacts[i];
            double theta = _resStep*i;
            ContactPtrList &list = _normalToContactsMap[0];
            if( list.size()==0 )
                fprintf(cfile,"%f %f %f %f %f %f -0.1\n", theta*rw::math::Rad2Deg, c.p(0), c.p(1), c.p(2), c.curvature, c.avgCurvature );
            else
                fprintf(cfile,"%f %f %f %f %f %f %f\n", theta*rw::math::Rad2Deg, c.p(0), c.p(1), c.p(2), c.curvature, c.avgCurvature, list[0]->avgCurvature );
        }

        fclose(cfile);
        printf("wrote: img_file_name\n");
    }

    double getMinCurvature(){
        return _minCurvature;
    }

    double getMaxCurvature(){
        return _maxCurvature;
    }

    double getAvgCurvature(){
        return _avgCurvature;
    }

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
