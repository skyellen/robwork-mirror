/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef rw_interpolator_CubicSplineInterpolator_HPP
#define rw_interpolator_CubicSplineInterpolator_HPP

/**
 * @file CubicSplineInterpolator.hpp
 */

#include "Interpolator.hpp"
#include "CubicSegment.hpp"

namespace rw { namespace interpolator {

    /** @addtogroup interpolator */
    /*@{*/

    /**
     * @brief The cubic spline interpolation is a piecewise continuous curve,
     * passing through each of a number of via points. The current implementation 
     * support generation of either free cubic spline or clamped cubic spline.
     * 
     * The free or natural cubic spline assume that the second derivatives at start and end 
     * time is zero 
     * 
     * To use the clamped cubic spline the user need to specify start and end values 
     * of the first derivative.
     * 
     * See http://www.physics.arizona.edu/~restrepo/475A/Notes/sourcea/node35.html
     * or
     * http://online.redwoods.cc.ca.us/instruct/darnold/laproj/Fall98/SkyMeg/Proj.PDF
     * 
     * TODO: implement support for more than just the natural spline
     * 
     */
    class CubicSplineInterpolator : public Interpolator
    {
    private:
        /**
         * @brief This function recalculates the spline segments from the viaPoints.
         */
        void calculateSpline();
        
        void calculateNaturalSpline();
        
        double _length;
        
        std::vector<std::pair<rw::math::Q, double> > _viaPoints;
        std::vector<FunctionSegment*> _segments;
        rw::math::Q _dqStart,_dqEnd;

    public:

        /**
         * @brief Constructor - constructs a free/natural cubic spline
         * @param qStart [in] Start configuration to time 0.
         * @param viapoints [in] a list of viaPoints and associated times
         */
        CubicSplineInterpolator(
        		rw::math::Q& qStart, 
        		const std::vector<std::pair<rw::math::Q, double> >& viapoints);

        /**
         * @brief Constructor - constructs a clamped cubic spline with the specified 
         * start and end derivative values.
         * @param qStart [in] Start configuration to time 0.
         * @param viapoints [in] a list of viaPoints and associated times
         */
        CubicSplineInterpolator(
                rw::math::Q& qStart, 
                const std::vector<std::pair<rw::math::Q, double> >& viapoints,
                const rw::math::Q& dqStart,
                const rw::math::Q& dqEnd);
        
        
        /**
         * @brief Deconstructor
         */
        virtual ~CubicSplineInterpolator();


        /**
         * @copydoc Interpolator::getLength
         */
        double getLength() const 
        { 
            return _length; 
        }

        /**
         * @copydoc Interpolator::getX
         */ 
        virtual rw::math::Q getX(double d) const;
        
        /**
         * @brief returns the set of viaPoints that this interpolator
         * interpolates over, including start and end point.
         * @return a vector of via points
         */
        virtual std::vector<rw::math::Q> getViaPoints() const ;

        /**
         * @brief The array of segments can be used if efficient random access is nessesary.
         * @return vector of pointers to FunctionSegments.
         */
        virtual const std::vector<FunctionSegment*>& getSegments() const {
            return _segments;
        }

    };

    /*@}*/
}} // end namespaces

#endif // end include guard
