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

#ifndef RWLIBS_SOFTBODY_BEAMGEOMETRY_HPP
#define RWLIBS_SOFTBODY_BEAMGEOMETRY_HPP

#include <vector>
#include <ostream>
#include <sstream>

#include <rw/math/Transform3D.hpp>

namespace rwlibs {
namespace softbody {
/** @addtogroup softbody */
/*@{*/

/**
* @brief Base class for discrete beam geometries
*/
class BeamGeometry
{
public:
    /**
    * @brief Constructor for the BeamGeometry class
    * 
    * @note For a Mod Russel beam in a typical coordinate frame, the vector of gravitiy \em G should be defined as to make the gravitational potential energy
    * \em increase for increased values of y, e.g. \f$ G=(0.0, 9.82, 0.0) \f$
    *
    * @param L length of object
    * @param Exvec vector of Young's modulus for each cross beam cros section, in MPa
    * @param vxvcec vector of Poisson's ratio for each cross beam cros section
    * @param rhovec vector of mass density for each cross beam cros section, in kg/mm^3
    * @param wTb world to base of beam transform
    * @param G vector of gravity, m/s^2
    */
    BeamGeometry(double L,
                 const std::vector<double> &Exvec,
                 const std::vector<double> &vxvec,
                 const std::vector<double> &rhovec,
                 const rw::math::Transform3D<> &wTb,
                 const rw::math::Vector3D<> &G
                );
    
    virtual ~BeamGeometry();
    
public:
    /**
    * @brief Sets the homogeneous transform of the beam
    *
    * @param T world to beam base transform
    */
    void setTransform(const rw::math::Transform3D<> &T);


    /**
    * @brief Retrieves the world to beam base transform of the beam
    *
    * @return world to beam base transform
    */
    rw::math::Transform3D<> getTransform(void) const;
    
    /**
     * @brief sets the directional vector of gravity
     *
     * @param G directional vector of gravity
     **/
    
    void setG(const rw::math::Vector3D<> &G);
    /**
     * @brief retrieves the directional vector of gravity
     *
     * @return directional vector of gravity
     **/
    rw::math::Vector3D<> getG(void) const;

public:
    /**@name Methods for retrieving material parameters */
    //@{
    /**
     * @brief returns the value of Young's modulus at x = i * h
     *
    * @param i index at which to return the value
     * @return Young's modulus at x = i * h
     **/
    double Ex(const int i) const;
    
    /**
     * @brief returns the value of Poisson's ratio at x = i * h
     *
    * @param i index at which to return the value
     * @return Poisson's ratio at x = i * h
     **/
    double vx(const int i) const;
    
    /**
     * @brief returns the value of the mass density at x = i * h
     *
    * @param i index at which to return the value
     * @return mass density at x = i * h
     **/
    double rho(const int i) const;

    /**
     * @brief returns the kappa coefficient at x = i * h
     * 
     * Kappa is the coefficient relating the i,k = (1,1) coefficients of strain to stress in a plane stress situation, i.e.
     * 
     * \f[
     *  \sigma_{11} = \frac{E}{1 - \nu^2} u_{11}
     * \f]
     * 
     * with \f$ \kappa = \frac{E}{1 - \nu^2} \f$
     *
     * @param i index at which to return the value
     * @return kappa at x = i * h
     **/
    double kappa(const int i) const;
    
    //@}
    
    /**
     * @brief returns the x-component of vector of gravity for the current configuration of the beam
     *
     * @return x-component of gravity
     **/
    double g1 ( void ) const;
    
    /**
     * @brief returns the y-component of vector of gravity for the current configuration of the beam
     *
     * @return y-component of gravity
     **/
    double g2 ( void ) const;

    
public:
    /**@name Integrals to be evaluate by derived classes */
    //@{
    
    
    /**
     * @brief Evaluate the \f$b_0\f$ integral
     * 
     * Evaluate the \f$b_0\f$ integral, defined as
     * 
     * \f[
     *     b_0 = \int \int_{A(x)} \rho (x, y, z) y^0 dz dy
     * \f]
     * 
     * at \f$x = i h\f$
     *
     * @param i index at which to evaluate the integral
     * @return the integral value
     **/    
    virtual double b0(const int i) const = 0;
    
    /**
     * @brief Evaluate the \f$b_1\f$ integral
     * 
     * Evaluate the \f$b_1\f$ integral, defined as
     * 
     * \f[
     *     b_1 = \int \int_{A(x)} \rho (x, y, z) y^1 dz dy
     * \f]
     * 
     * at \f$x = i h\f$
     *
     * @param i index at which to evaluate the integral
     * @return the integral value
     **/    
    virtual double b1(const int i) const = 0;
    
    /**
     * @brief Evaluate the \f$ c_2 \f$ integral
     * 
     * Evaluate the \f$ c_2 \f$ integral, defined as
     * 
     * \f[
     *     c_2 = \int \int_{A(x)} \frac{K(x,y,z)}{8} y^2 dz dy
     * \f]
     * 
     * at \f$ x = i h \f$
     *
     * @param i index at which to evaluate the integral
     * @return the integral value
     **/    
    virtual double c2(const int i) const = 0;
    
    /**
     * @brief Evaluate the \f$ c_3 \f$ integral
     * 
     * Evaluate the \f$ c_3 \f$ integral, defined as
     * 
     * \f[
     *     c_3 = \int \int_{A(x)} \frac{K(x,y,z)}{8} y^3 dz dy
     * \f]
     * 
     * at \f$ x = i h \f$
     *
     * @param i index at which to evaluate the integral
     * @return the integral value
     **/   
    virtual double c3(const int i) const = 0;
    
    /**
     * @brief Evaluate the \f$ c_4 \f$ integral
     * 
     * Evaluate the \f$ c_4 \f$ integral, defined as
     * 
     * \f[
     *     c_4 = \int \int_{A(x)} \frac{K(x,y,z)}{8} y^4 dz dy
     * \f]
     * 
     * at \f$ x = i h \f$
     *
     * @param i index at which to evaluate the integral
     * @return the integral value
     **/   
    virtual double c4(const int i) const = 0;

    /**
     * @brief evaluate the \f$ B_0 \f$ constant
     * 
     * \f[
     *    B_0(x) = B_0^*(L) - B_0^*(x)
     * \f]
     * 
     * with
     * 
     * \f[
     *    B_0^*(x) = \int_0^x b_0(s) ds
     * \f]
     *
     * @param i index at which to evaluate the integral
     * @return the constant value
     **/
    virtual double B0(const int i) const = 0;

    //@}
public:
    /**
     * @brief return left boundary of domain
     *
     * @return left boundary
     **/
    double get_a(void) const {
        return _a;
    };
    
    /**
     * @brief return right boundary of domain
     *
     * @return right boundary
     **/
    double get_b(void) const {
        return _b;
    };
    
    /**
     * @brief return stepsize
     *
     * @return stepsize
     **/   
    double get_h(void) const {
        return _h;
    };

    /**
         * @brief return length of beam
         *
         * @return double
     **/
    double getL(void) const {        
        return _L;
    };
    
    /**
     * @brief return number of cross sections in beam
     *
     * @return number of cross sections
     **/
    int getM(void) const {
        return _M;
    };
    
public:
	friend std::ostream& operator<<(std::ostream& out, const BeamGeometry& obj) {
        /**
         * @brief outputs the geometry to stream
         *
         * @param out the stream
         * @param obj the geometry
         * @return the geometry as a stream
         **/
        std::stringstream str;
	        
	    str << "BeamGeometry {a: " << obj.get_a() << ", b: " << obj.get_b() << ", h: " << obj.get_h() << ", L: " << obj.getL() << "}";
	    
	    return out << str.str();
	};

private:
    double _L; // Length of beam in the x-direction 

    int _M; // number of slices
    double _a, _b; // domain endpoints, see constructor initilization list
    double _h; // stepsize
private:
    std::vector<double> _Exvec; // vector holding the values of Young's modulus for each slice
    std::vector<double> _vxvec; // vector holding the values of Poisson's ratio for each slice
    std::vector<double> _rhovec; // vector holding the values of the mass density for each slice

    rw::math::Transform3D<> _wTb; // world to beam transform
    rw::math::Vector3D<> _G; // directional vector of gravity
};
/*@}*/
}
}

#endif // BEAMGEOMETRY_HPP
