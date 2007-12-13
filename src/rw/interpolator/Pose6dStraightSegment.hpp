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

#ifndef rw_interpolator_Pose6dStraightSegment_HPP
#define rw_interpolator_Pose6dStraightSegment_HPP

/**
 * @file Pose6dStraightSegment.hpp
 */

#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Quaternion.hpp>

namespace rw { namespace interpolator {

    /**
     * @brief Represents a straight segment in 6D Cartesian space
     */
    class Pose6dStraightSegment
    {
    private:
        std::pair<double,double> _interval;

        /**
         * @brief The parameters of the straight line function
         */
        math::Transform3D<> _a;
        math::VelocityScrew6D<> _b;

    public:
        /**
         * @brief Constructor
         */
        Pose6dStraightSegment(const math::Transform3D<>& a,
                              const math::VelocityScrew6D<>& b)
            :_interval(0.0,1.0),_a(a),_b(b){};          

        /**
         * @brief Constructor
         */
        Pose6dStraightSegment(const math::Transform3D<>& a,
                              const math::VelocityScrew6D<>& b,
                              const std::pair<double,double> &interval)
            :_interval(interval),_a(a),_b(b){};


        /**
         * @brief copy constructor
         * @param segment [in] segment to copy
         */
        Pose6dStraightSegment(const Pose6dStraightSegment& segment) {
            _a = segment._a;
            _b = segment._b;
            _interval = segment._interval;
        };


        /**
         * @brief sets all the parameters of this StraightLine segment
         * @param a [in] the a parameter in the straight line equation
         * @param b [in] the b parameter in the straight line equation
         * @param interval [in] the interval in which the segment is valid
         */
        void setParameters(const math::Transform3D<>& a,
                           const math::VelocityScrew6D<>& b,
                           const std::pair<double,double> &interval )
        {
            _a=a;_b=b;_interval=interval;
        }

        /**
         * @brief Deconstructor
         */
        virtual ~Pose6dStraightSegment(){};

        /**
         * @copydoc FunctionSegment::getX
         */
        math::Transform3D<> getX(double t) const {
            //return _a+t*_b;
            math::VelocityScrew6D<> b = _b*t;
            math::Vector3D<double> pos = _a.P()+b.linear();
            
            
            
            math::Rotation3D<double> orin = b.angular().toRotation3D();
            math::Rotation3D<> o = _a.R()*orin;    
            
            
            
            //(LPE) This old stuff is just plain wrong  
            /*math::Rotation3D<double> orin = b.angular().toRotation3D();
            std::cout<<"orin = "<<orin<<std::endl;
            // Calculate
            //math::Rotation3D<double> o = orin*( inverse(orin)*_a.R() );
            math::Quaternion<> q(math::Quaternion<>(_a.R())+math::Quaternion<>(orin));
            std::cout<<"q = "<<q<<std::endl;
            q.normalize();
            math::Rotation3D<double> o = q.toRotation3D();*/
            
            
            return math::Transform3D<>(pos,o);
        };

        /**
         * @copydoc FunctionSegment::getXd
         */
        math::VelocityScrew6D<> getXd(double t) const {
            return _b;
        };

        /**
         * @copydoc FunctionSegment::getXdd
         */
        math::Transform3D<> getXdd(double t) const {
            return math::Transform3D<>::Identity();
        }

        /**
         * @brief the length of the interpolated path is defined as the complete
         * length all the pathsegments.
         * @return the interval in which this function segment is valid.
         */
        virtual std::pair<double,double> getInterval() const {return _interval;};

        /**
         * @brief the length of the interpolated path is defined as the complete
         * length all the pathsegments.
         * @return the length of the interval of this Pose6dFunctionSegment
         */
        virtual double getIntervalLength() const {return _interval.second - _interval.first;};


        /**
         * @brief assigment operator
         * @param segment [in] segment whose values to copy.
         */
        Pose6dStraightSegment& operator=(const Pose6dStraightSegment& segment) {
            this->_a = segment._a;
            this->_b = segment._b;
            this->_interval = segment._interval;

            return *this;
        };

    };

}} // end namespaces

#endif // end include guard
