/*
Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,

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


#ifndef RWLIBS_SOFTBODY_MODRUSSELBEAMBASE_HPP
#define RWLIBS_SOFTBODY_MODRUSSELBEAMBASE_HPP

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
     static std::vector<int> computeIntegralIndicies( const int nIntegralConstraints, const int N );
    
    /**
     * @brief returns the indices on the beam at which to place integral constraints
     *
     * @return indices on the beam
     **/   
    std::vector<int> getIntegralIndices(void) const;;
    
    void setIntegralIndices(const std::vector<int> &indices);
    
    /**
     * @brief returns the BeamGeometry used by the beam
     *
     * @return the BeamGeometry used by the beam
     **/
    boost::shared_ptr< BeamGeometry > getGeometry ( void ) const;
    
    
    /**
     * @brief returns the BeamObstaclePlane used by the beam
     *
     * @return the BeamObstaclePlane used by the beam
     **/
    boost::shared_ptr< BeamObstaclePlane > getObstacle ( void ) const;

    /**
     * @brief returns number of cross sections in beam
     *
     * @return number of cross sections in beam
     **/
    int getM ( void )  const;
    
    /**
     * @brief returns accuracy goal of underlying numerical methods
     *     
     * @return accuracy goal
     **/
    double getAccuracy ( void ) const;
    
    /**
     * @brief sets accuracy goal of underlying numerical methods
     *
     * @param acc accuracy goal
     **/
    void setAccuracy ( double acc );
    
    friend std::ostream& operator<< ( std::ostream& out, const ModRusselBeamBase& obj ) {
        /**
         * @brief outputs a ModRusselBeamBase to stream
         *
         * @param out the stream
         * @param obj the beam
         * @return a stream
         **/
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
    
    /*
    void setUseNoUpwardConstraint ( bool val );
    bool getUseNoUpwardConstraint(void) const { return _useNoUpwardConstraint; };
    
    void setUseHingeConstraint ( bool val );
    bool getUseHingeConstraint(void) const { return _useHingeConstraint; };
    */

    /*
    void setMuStart ( double muStart );
    double getMuStart(void) const { return _muStart; };
    
    void setMuDecrementFactor ( double decFactor );
    double getMuDecrementFactor(void) const { return _muDec; };
    */

    /**
     * @brief sets the number of integral constraints on the beam
     *
     * @param nIntegralConstraints number of integral constraints
     **/
//     void set_nIntegralConstraints ( int nIntegralConstraints );
    
    /**
     * @brief returns the number of integral constraints on the beam
     *
     * @return number of integral constraints on the beam
     **/
//     int get_nIntegralConstraints ( void ) const;

public:
    /**
     * @brief returns the plane to beam transform
     *
     * @return plane to beam transform
     **/
    rw::math::Transform3D< double > get_planeTbeam ( void ) const ;
    
    /**
     * @brief returns the angle of the beam base frame wrt. the associated obstacle plane
     *
     * @return angle of beam base in radians
     **/    
    double get_thetaTCP ( void ) const ;

    /**
     * @brief returns the height of the beam baseframe over the associated obstacle plane
     *
     * @return height above the obstacle in millimeters
     **/    
    double get_yTCP ( void ) const ;
    
    /**
     * @brief return the x-component of the y-axis of the beam baseframe, for the current plane to beam transform
     *
     * @return x-component of the y-axis of the beam baseframe
     **/
    double get_uxTCPy ( void ) const ;
    
    /**
     * @brief return the y-component of the y-axis of the beam baseframe, for the current plane to beam transform
     *
     * @return x-component of the y-axis of the beam baseframe
     **/
    double get_uyTCPy ( void ) const ;
    
    /**
     * @brief return the x-component of the y-axis of the beam baseframe
     * 
     * @param planeTbeam the plane to beam transform to use
     * @return x-component of the y-axis of the beam baseframe
     **/
    static double get_uxTCPy ( const rw::math::Transform3D<> planeTbeam )  ;
    
    /**
     * @brief return the y-component of the y-axis of the beam baseframe
     *
     * @param planeTbeam the plane to beam transform to use
     * @return x-component of the y-axis of the beam baseframe
     **/
    static double get_uyTCPy ( const rw::math::Transform3D<> planeTbeam )  ;
    
    /**
     * @brief return the discretization step for the beam cross sections
     *
     * @return discretization step
     **/
    double get_h ( void )  const ;

private:
    boost::shared_ptr< BeamGeometry > _geomPtr;
    boost::shared_ptr< BeamObstaclePlane > _obstaclePtr;
    int _M;

    double _accuracy;
    
private: 
    bool _useNoUpwardConstraint;

//     int _nIntegralConstraints;

    bool _useHingeConstraint;
    
    std::vector<int> _integralConstraintIdxList;

//     double _muStart;
//     double _muDec;
};
/*@}*/
}
};

#endif // MODRUSSELBEAMBASE_HPP
