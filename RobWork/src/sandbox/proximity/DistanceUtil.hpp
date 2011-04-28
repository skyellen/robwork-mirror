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

#ifndef RW_GEOMETRY_DISTANCEUTIL_HPP_
#define RW_GEOMETRY_DISTANCEUTIL_HPP_

namespace rw {
namespace proximity {

    /**
     * @brief a class for performing distance calculations between different
     * geometric primitives
     */
    class DistanceUtil {
    public:

        static double distanceLineLineSqr(const rw::math::Vector3D<>& p1,
                                          const rw::math::Vector3D<>& p2,
                                          const rw::math::Vector3D<>& q1,
                                          const rw::math::Vector3D<>& q2)
        {
            using namespace rw::math;
            Vector3D<> c1,c2;
            double s,t;
            const double EPSILON = 0.0000001;
            Vector3D<> d1 = p2 - p1;
            Vector3D<> d2 = q2 - q1;
            Vector3D<> r = p1-q1;

            double a = dot(d1,d1);
            double e = dot(d2,d2);
            double f = dot(d2,r);

            if( a<=EPSILON && e<=EPSILON ){
                // both segments degenerate into points
                s = t = 0.0;
                c1 = p1;
                c2 = q1;
                return dot(c1-c2,c1-c2);
            }

            if(a<= EPSILON){
                s = 0.0;
                t = f/e;
                t = Math::clamp(t,0.0,1.0);
            } else {
                double c = dot(d1,r);
                if(e<=EPSILON){
                    t = 0.0;
                    s = Math::clamp(-c/a,0.0,1.0);
                } else {
                    double b = dot(d1,d2);
                    double denom = a*e-b*b;

                    if(denom != 0.0){
                        s = Math::clamp((b*f-c*e)/denom,0.0,1.0);
                    } else {
                        s = 0.0;
                    }

                    t = (b*s + f)/e;

                    if( t<0.0 ){
                        t = 0.0;
                        s = Math::clamp(-c/a,0.0,1.0);
                    } else if(t>1.0){
                        t = 1.0;
                        s = Math::clamp( (b-c)/a,0.0,1.0 );
                    }
                }
            }

            c1 = p1 + d1*s;
            c2 = q1 + d2*t;
            return dot(c1-c2,c1-c2);
        }


        static double distanceLineLine(const rw::math::Vector3D<>& p1, const rw::math::Vector3D<>& p2,
                                const rw::math::Vector3D<>& q1, const rw::math::Vector3D<>& q2)
        {
            return sqrt( distanceLineLineSqr(p1,p2,q1,q2) );
        }



    };

}
}

#endif /* DISTANCEUTIL_HPP_ */
