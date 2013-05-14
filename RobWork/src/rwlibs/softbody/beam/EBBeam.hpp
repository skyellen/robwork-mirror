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


#ifndef RWLIBS_SOFTBODY_EBBEAM_HPP
#define RWLIBS_SOFTBODY_EBBEAM_HPP

namespace rwlibs {
namespace softbody {
        /** @addtogroup softbody */
/*@{*/
    /**
     * @brief Class for calculating the analytical solution to the Euler-Bernoulli beam model in fixed-free configuration
     * 
     * Implementation of the classical analytical solution to the Euler-Bernoulli beam model in fixed-free configuration, for a cuboid beam, i.e.
     * 
     * \f[
     *   \eta(x) = \frac{q x^2 (6 L^2 - 4 L x + x^2)}{24 E J}
     * \f]
     * 
     * where \f$ q = g \rho(x) A(x) \f$, where \f$ g \f$ is the gravitational acceleration, \f$ \rho \f$ the mass density and \f$ A(x) = H * K \f$ the area for the cross 
     * section,
     * 
     * where \f$ L \f$ is the length of the object, \f$ E \f$ is Young's modulus of elasticity and
     * 
     * where \f$ J = \frac{K H^3}{12} \f$ is the second moment of area for the cuboid with the neutral line in the centroid.
     **/
    class EBBeam {
	public:
        /**
         * @brief constructor
         *
         * @param H the thickness in the z-direction
         * @param K the thickness in the y-direction
         * @param L the length of the beam, in the z-direction
         * @param E Young's modulus
         * @param rho the mass density
         * @param h stepsize used in the beam
         * @param g2 vertical component of the gravity direction vector
         **/
        EBBeam(
		const double H, 
		   const double K, 
		   const double L, 
		   const double E, 
		   const double rho,
		   const double h,
         const double g2
		   );
		
        /**
         * @brief returns the deformation at x = i * h
         *
         * @param i the index at which to return the deformation
         * @return the deformation
         **/
        double operator() (const int i) const;
		
        /**
         * @brief returns the first derivative of the deformation at x = i * h
         *
         * @param i the index at which to return the deformation
         * @return the first derivative of the deformation
         **/
        double d(const int i) const;
    
    private:   
		const double _H, _K, _L, _E, _rho, _h;
		
		double _J;
		double _q;
};
/*@}*/
}};

#endif // EBBEAM_HPP
