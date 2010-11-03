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

#ifndef RW_GEOMETRY_CONTOUR2D_HPP_
#define RW_GEOMETRY_CONTOUR2D_HPP_

#include <vector>
#include <rw/math/Vector2D.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/common/macros.hpp>
#include <fstream>
#include <iostream>

namespace rw {
namespace geometry {
	//! @addtogroup geometry
	// @{

	/**
	 * @brief class representing a 2d contour
	 */
	class Contour2D
	{
	public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<Contour2D> Ptr;


	    /**
	     * @brief the point description of the contour
	     */
	    class Point
	    {
	    public:
	        //! @brief constructor
	        Point(){};

	        //! @brief constructor
	        Point(
	            const rw::math::Vector2D<>& position,
	            const rw::math::Vector2D<>& normal) :
	            _position(position),
	            _normal(normal)
	        {}

	        /**
	         * @brief constructor
	         * @param position
	         * @param magnitude
	         * @param orientation
	         * @return
	         */
	        Point(
	            const rw::math::Vector2D<>& position,
	            double magnitude,
	            double orientation) :
	            _position(position),
	            _normal(
	                -magnitude * cos(orientation),
	                -magnitude * sin(orientation))
	        {}

	        //! @brief get position of this contour point
	        const rw::math::Vector2D<>& P() const { return _position; }
	        //! @brief get position of this contour point
	        rw::math::Vector2D<>& P() { return _position; }

	        //! @brief get normal of this contour point
	        const rw::math::Vector2D<>& N() const { return _normal; }
	        //! @brief get normal of this contour point
            rw::math::Vector2D<>& N(){return _normal;}

            //! @note deprecated
            const rw::math::Vector2D<>& getDirection() const { return _normal; }
            //! @note deprecated
            void setDirection(const rw::math::Vector2D<>& dir){_normal = dir;}

	    private:
	        rw::math::Vector2D<> _position;
	        rw::math::Vector2D<> _normal;
	    };

	public:
	    /**
	     * @brief constructor
	     */
		Contour2D(){};

		/**
		 * @brief constructor
		 * @param center
		 * @param contour
		 */
		Contour2D(
			const rw::math::Vector2D<>& center,
			const std::vector<Point>& contour)
			:
			_center(center),
			_points(contour)
		{}

		/**
		 * @brief get nr of conout points on this contour
		 * @return
		 */
		size_t size() const {
			return _points.size();
		};

        /**
         * @brief get i'th contour point
         * @param i
         * @return the i'th contour point
         */
        Point& operator[](size_t i) { return _points[i]; }

        //! @copydoc operator[]
		const Point& operator[](size_t i) const { return _points[i]; }

		//! @brief calculates the area of this contour
		double calcArea();

		//! get contour center
		rw::math::Vector2D<>& center(){ return _center; };
		//! get contour center
		const rw::math::Vector2D<>& center() const { return _center; };

		//! get contour point list
		std::vector<Point>& points(){ return _points; };
		//! get contour point list
		const std::vector<Point>& points() const { return _points; };

		/**
		 * @brief writes a contour to file
		 * @param objC [in] contour to write to file
		 * @param file [in] name of file
		 */
		static void write(Contour2D& objC, std::string file);

		/**
		 * @brief reads a contour from file
		 * @param file
		 * @return a contour
		 */
		static Contour2D read(std::string file);

	private:
        rw::math::Vector2D<> _center;
        std::vector<Point> _points;

	};

#ifdef RW_USE_DEPRECATED
	//! smart pointer of contour2d
	typedef rw::common::Ptr<Contour2D> Contour2D::Ptr;
#endif
	//! @}
}
}

#endif /*CONTOUR2D_HPP_*/
