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

#ifndef CONTOUR2D_HPP_
#define CONTOUR2D_HPP_

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


	class ContourPoint
	{
	public:
		ContourPoint(){}

		ContourPoint(
			const rw::math::Vector2D<>& position,
			const rw::math::Vector2D<>& direction) :
			_position(position),
			_direction(direction)
		{}

		ContourPoint(
			const rw::math::Vector2D<>& position,
			double magnitude,
			double orientation) :
			_position(position),
			_direction(
				-magnitude * cos(orientation),
				-magnitude * sin(orientation))
		{}

		const rw::math::Vector2D<>& getPosition() const { return _position; }
		const rw::math::Vector2D<>& getDirection() const { return _direction; }
		void setDirection(const rw::math::Vector2D<>& dir){_direction = dir;}
	private:
		rw::math::Vector2D<> _position;
		rw::math::Vector2D<> _direction;
	};

	struct Contour2D
	{
		rw::math::Vector2D<> center;
		std::vector<ContourPoint> contour;

		Contour2D(){};

		Contour2D(
			const rw::math::Vector2D<>& center,
			const std::vector<ContourPoint>& contour)
			:
			center(center),
			contour(contour)
		{}

		size_t size() const {
			return contour.size();
		};

		const ContourPoint& operator[](size_t i) const { return contour[i]; }
		ContourPoint& operator[](size_t i) { return contour[i]; }

		static void write(Contour2D& objC, std::string file){

			std::ofstream ostr(file.c_str());
			if (!ostr.is_open())
				RW_THROW("Can't read file " << rw::common::StringUtil::quote(file));

			ostr << "ObjectContour \n";
			ostr << "Size " << objC.size() << "\n";
			ostr << "Center " << objC.center(0) << " " << objC.center(1) << "\n";
			//std::cout << "size: " << objC.size() << std::endl;
			//std::cout << "center: " << objC.center << std::endl;
			for(size_t i=0; i<objC.size(); i++){
				ContourPoint &point = objC[i];
				rw::math::Vector2D<> pos = point.getPosition();
				rw::math::Vector2D<> dir = point.getDirection();
				ostr << "Pos " << pos(0) << " " << pos(1) << "\n";
				ostr << "Dir " << dir(0) << " " << dir(1) << "\n";
			}
			ostr.close();
		}

		static Contour2D read(std::string file){
			std::ifstream inp(file.c_str());
			if (!inp.is_open())
				RW_THROW("Can't read file " << rw::common::StringUtil::quote(file));
			std::string strToken;
			inp >> strToken;

			int size = 0;
			rw::math::Vector2D<> center;
			inp >> strToken >> size;
			inp >> strToken >> center(0) >> center(1);
			//std::cout << "size: " << size << std::endl;
			Contour2D objC;
			objC.center = center;
			//std::cout << "center: " << center<< std::endl;
			objC.contour.resize(size);
			for(size_t i=0; i<objC.size(); i++){
				rw::math::Vector2D<> pos, dir;

				inp >> strToken >> pos(0) >> pos(1);
				inp >> strToken >> dir(0) >> dir(1);
				objC[i] = ContourPoint(pos,dir);
			}
			inp.close();
			return objC;
		}

	};
	//! @}
}
}

#endif /*CONTOUR2D_HPP_*/
