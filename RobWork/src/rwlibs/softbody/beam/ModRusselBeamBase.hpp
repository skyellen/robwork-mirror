/*
    Copyright 2013 <copyright holder> <email>

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/


#ifndef MODRUSSELBEAMBASE_HPP
#define MODRUSSELBEAMBASE_HPP

#include <boost/shared_ptr.hpp>

#include <rw/math/Transform3D.hpp>

#include "BeamGeometry.hpp"
#include "BeamObstaclePlane.hpp"


namespace rwlibs { namespace softbody {
    /** @addtogroup softbody */
/*@{*/
    /**
     * @brief Base class for implementing Modified Russel beam problems
     **/
class ModRusselBeamBase {
public:
    /**
     * @brief Constructor
     *
     * @param geomPtr pointer to the beam geometry
     * @param obstaclePtr pointer to the obstacle 
     * @param M number of slices in the beam
     **/
    ModRusselBeamBase (
        boost::shared_ptr< rwlibs::softbody::BeamGeometry > geomPtr,
        boost::shared_ptr< rwlibs::softbody::BeamObstaclePlane > obstaclePtr,
        int M
    );

    virtual ~ModRusselBeamBase();
    
public:
    virtual void solve ( boost::numeric::ublas::vector< double >& xinituser, boost::numeric::ublas::vector<double> &U, boost::numeric::ublas::vector<double> &V ) = 0;
    
public:
    /**
     * @brief given a vector of angles, calculates the x-part of the corresponding curve
     *
     * @param U reference to the vector to store the x-component in 
     * @param avec reference to the vector of angles
     **/
    void integrateAngleU ( boost::numeric::ublas::vector<double> &U, const boost::numeric::ublas::vector<double> &avec );
    
    /**
     * @brief given a vector of angles, calculates the y-part of the corresponding curve
     *
     * @param V reference to the vector to store the x-component in 
     * @param avec reference to the vector of angles
     **/
    void integrateAngleV ( boost::numeric::ublas::vector<double> &V, const boost::numeric::ublas::vector<double> &avec );
    
    /**
     * @brief precomputes the indices on the beam at which to place integral constraints
     * 
     * This function will precompute indices on the beam at which integral constraints are present to check for penetration with the plane. 
     * For nIntegralConstraints equal to 1, only the tip at x=L will be tested. 
     * For 2 integral constraints, indices equivalent to x=L and x=L/2 will be tested and so forth.
     * 
     * The number of integral constraints is set by the method set_nIntegralConstraints 
     *
     * @bug Does not support more than M/2 integral constraints
     * 
     **/   
    void computeIntegralIndicies(void);
    
    /**
     * @brief returns the indices on the beam at which to place integral constraints
     *
     * @return indices on the beam
     **/   
    std::vector<int> getIntegralIndices(void) const { return _integralConstraintIdxList; };
    
    boost::shared_ptr< BeamGeometry > getGeometry ( void ) const;
    boost::shared_ptr< BeamObstaclePlane > getObstacle ( void ) const;

    int getM ( void )  const;
    
    double getAccuracy ( void ) const;
    void setAccuracy ( double acc );
    
    friend std::ostream& operator<< ( std::ostream& out, const ModRusselBeamBase& obj ) {
        std::stringstream str;

        const rw::math::Transform3D<> planeTbeam = obj.get_planeTbeam();

        double yTCP = obj.getObstacle()->get_yTCP ( planeTbeam );
        double thetaTCP = obj.getObstacle()->get_thetaTCP ( planeTbeam );
        double g1 = obj.getGeometry()->g1();
        double g2 = obj.getGeometry()->g2();
        const double uxTCPy =  obj.get_uxTCPy();
        const double uyTCPy = obj.get_uyTCPy();

        str << "ModRusselBeam {M:" << obj.getM() << ", g1: " << g1 << ", g2:" << g2 << ", uxTCPy: " << uxTCPy << ", uyTCPy: " << uyTCPy << ", yTCP: " << yTCP << ", thetaTCP: " << thetaTCP << ", accuracy: " << obj.getAccuracy() << "}";

        return out << str.str();
    };
    
    
    void setUseNoUpwardConstraint ( bool val );
    bool getUseNoUpwardConstraint(void) const { return _useNoUpwardConstraint; };
    
    void setUseHingeConstraint ( bool val );
    bool getUseHingeConstraint(void) const { return _useHingeConstraint; };


    void setMuStart ( double muStart );
    double getMuStart(void) const { return _muStart; };
    
    void setMuDecrementFactor ( double decFactor );
    double getMuDecrementFactor(void) const { return _muDec; };

    void set_nIntegralConstraints ( int nIntegralConstraints );
    int get_nIntegralConstraints ( void ) const;

public:
    rw::math::Transform3D< double > get_planeTbeam ( void ) const ;
    double get_thetaTCP ( void ) const ;

    double get_yTCP ( void ) const ;
    
    double get_uxTCPy ( void ) const ;
    double get_uyTCPy ( void ) const ;
    
    static double get_uxTCPy ( const rw::math::Transform3D<> planeTbeam )  ;
    static double get_uyTCPy ( const rw::math::Transform3D<> planeTbeam )  ;
    
    double get_h ( void )  const ;

private:
    boost::shared_ptr< BeamGeometry > _geomPtr;
    boost::shared_ptr< BeamObstaclePlane > _obstaclePtr;
    int _M;

    double _accuracy;
    
private: // TODO: get methods for these
    bool _useNoUpwardConstraint;

    int _nIntegralConstraints;

    bool _useHingeConstraint;
    std::vector<int> _integralConstraintIdxList;

    double _muStart;
    double _muDec;
};
/*@}*/
}
};

#endif // MODRUSSELBEAMBASE_HPP
