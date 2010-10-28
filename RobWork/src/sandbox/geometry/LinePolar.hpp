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

#ifndef RWSIM_UTIL_LINEPOLAR_HPP
#define RWSIM_UTIL_LINEPOLAR_HPP

#include "Pose2D.hpp"
#include <rw/math/Line2D.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/foreach.hpp>

namespace rw {
namespace geometry {

	class LinePolar
	{
	public:


		// rho * (cos(theta), sin(theta)) is the point on the line nearest to origo.
	    LinePolar(double rho = 0, double theta = 0);

		// 'pnt' is any point on the line, and theta is the usual angle.
		static LinePolar make(const rw::math::Vector2D<>& pnt, double theta);

		// The line moving through the segment from 'start' to 'end'.
		static LinePolar make(const rw::math::Vector2D<>& start, const rw::math::Vector2D<>& end);

		static LinePolar make(const std::vector< rw::math::Vector2D<> >& points);

		rw::math::Line2D toLine2D();

		double getRho() const { return _rho; }
		double getTheta() const { return _theta; }
		const rw::math::Vector2D<>& getNormal() const { return _normal; }

		// The L_2 distance from 'pnt' to the line.
		double dist2(const rw::math::Vector2D<>& pnt);

		// The point for the projection of 'pnt' onto 'line'.
		static
		rw::math::Vector2D<> projectionPoint(const LinePolar& line, const rw::math::Vector2D<>& pnt);

		// A supporting point on the line (equal to rho * normal).
		static
		rw::math::Vector2D<> linePoint(const LinePolar& line);

		// The vector for the projection of 'pnt' onto the normal of 'line'.
		static
		rw::math::Vector2D<> normalProjectionVector(const LinePolar& line, const rw::math::Vector2D<>& pnt);

		// Print the line to stdout.
		static
		void print(const LinePolar& line);

		// An ublas vector of (rho, theta).
		static
		boost::numeric::ublas::vector<double> toUblas(const LinePolar& line);

		// 'line' given relative to the coordinate frame of 'pose'.
		static
		LinePolar lineToLocal(
			const Pose2D& pose,
			const LinePolar& line);

	    typedef std::vector<rw::math::Vector2D<> >::const_iterator const_iterator;
	    typedef std::pair<const_iterator, const_iterator> const_iterator_pair;

		static LinePolar fitSVD(const_iterator_pair range);
	    static LinePolar fit(const_iterator_pair range);
	    static LinePolar fit(const_iterator a, const_iterator b);
	    static LinePolar fit(const std::vector<rw::math::Vector2D<> >& pnts);

	    ///// ModelInterface

	    static int getMinReqData(){ return 2; };

        bool same( LinePolar& model, double thres){
            return fabs(model.getTheta()-_theta)<thres && fabs(model.getRho()-_rho)<3;
        }

        bool invalid(){
            return false;
        }

        double fitError(const rw::math::Vector2D<>& point){
            return dist2(point);
        }

        double refit(const std::vector<rw::math::Vector2D<> >& pnts){
            LinePolar line = fit(pnts);
            _rho = line.getRho();
            _theta = line.getTheta();
            _normal = line.getNormal();
            double error= 0;
            BOOST_FOREACH(const rw::math::Vector2D<>& pnt, pnts){
                error += line.dist2(pnt);
            }
            return error/pnts.size();
        }

	public:
		double _rho;
		double _theta;
		rw::math::Vector2D<> _normal;
	};
}
}

#endif
