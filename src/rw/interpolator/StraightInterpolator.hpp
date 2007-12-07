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

#ifndef rw_interpolator_StraightInterpolator_HPP
#define rw_interpolator_StraightInterpolator_HPP

/**
 * @file StraightInterpolator.hpp
 */

#include "StraightSegment.hpp"
#include "Interpolator.hpp"

#include <rw/math/Metric.hpp>
#include <rw/math/Q.hpp>

namespace rw { namespace interpolator {

    /** @addtogroup interpolator */
    /*@{*/

    /**
     * @brief This class interpolates between a number of viapoints using
     * straight line segments.
     *
     * Here is a usage example
     * \code
     *  boost_vector A(2),B(2),C(2),D(2);
     *      A[0] = 1; A[1] = 1;
     *  B[0] = 10; B[1] = 5;
     *  StraightInterpolator interp(A,B);
     *
     *  C[0] = 3; C[1] = 7;
     *  interp.addVia(C);
     *
     *  D[0] = 7; D[1] = 4;
     *  assert(interp.insertVia(D,10)==false); // ==false
     *  assert(interp.insertVia(D,2)==true); // ==true
     *
     *  assert(interp.removeVia(10)==false); // ==false
     *  assert(interp.removeVia(2)==true); // ==true
     * \endcode
     */
    class StraightInterpolator : public Interpolator
    {
    private:
        double _length;
        
        std::vector<rw::math::Q> _viaPoints;
        std::vector<double> _times;
        std::vector<FunctionSegment*> _segments;

    public:
        /**
         * @brief Constructs StraightInterpolator
         * 
         * Constructs a StraightInterpolator with \a qStart as start configuration and 
         * with the specified via-points
         * @param qStart [in] Start configuration at time 0
         * @param viapoints [in] set of viapoints and associated times         
         */
        StraightInterpolator(const rw::math::Q& qStart,
                             const std::vector<std::pair<rw::math::Q, double> >& viapoints);
        
        /**
         * @brief Constructs StraightInterpolator
         * 
         * Constructs a StraightInterpolator with \a qStart and start configuration
         * and \a qEnd as end configuration reached a time \a tEnd
         * 
         * @param qStart [in] Start configuration
         * @param qEnd [in] End configuration
         * @param tEnd [in] Time the end is reached
         */
        StraightInterpolator(const rw::math::Q& qStart,
                             const rw::math::Q& qEnd, 
                             double tEnd);


        /**
         * @brief Destructor
         */
        virtual ~StraightInterpolator();

        /**
         * @brief Adds a viapoint at the time t
         * @param via [in] vector that defines the via point that is to be added
         * @param t [in] the absolute time - needs to be after the last via point added
         */
        void addVia(const rw::math::Q &via, 
        			double t);

        /**
         * @brief Inserts a viapoint at the absolute time t in the path.
         *
         * The current via point at i'th will be pushed to the i+1 index.
         *
         * @param via [in] vector that defines the via point that is to be
         * inserted
         *
         * @param t [in] the absolute time of where the viapoint is placed 
         */
        bool insertVia(const rw::math::Q &via, double t);

        /**
         * @brief Removes the i'th viapoint
         *
         * @param i [in] integer that defines the index of the viapoint
         */
        bool removeVia(unsigned int i);

        /**
         * @copydoc Interpolator::getLength
         */
        double getLength() const {
            return _length;
        }

        /**
         * @copydoc Interpolator::getX
         */
        virtual rw::math::Q getX(double d) const;
        

        /**
         * @copydoc Interpolator::getSegments
         */
        virtual const std::vector<FunctionSegment*>& getSegments() const {
            return _segments;
        }
        
        /**
         * @copydoc Interpolator::getViaPoints
         */
        virtual std::vector<rw::math::Q> getViaPoints() const {
            return _viaPoints;
        }
        
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
