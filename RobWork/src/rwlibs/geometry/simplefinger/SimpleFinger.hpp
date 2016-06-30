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
 
#ifndef _SIMPLEFINGER_HPP
#define _SIMPLEFINGER_HPP

#include <rw/geometry/Primitive.hpp>

namespace rwlibs {
namespace geometry {
namespace simplefinger {

/**
 * Defines customized geometry for a simple gripper finger shape.
 */
class SimpleFinger: public rw::geometry::Primitive {
public:
	//! Smart pointer to this type of class.
	typedef rw::common::Ptr<SimpleFinger> Ptr;

public:
	/**
	 * Constructor.
	 * 
	 * Creates a basic finger shape -- box (0.2 x 0.025 x 0.01) with
	 * no extra geometric features.
	 */
	SimpleFinger();
	
	/**
	 * Constructor.
	 * 
	 * @param initQ [in] vector of parameters (length, width, depth, chflength, chfdepth, cutpos, cutdepth, cutangle, cuttilt).
	 */
	SimpleFinger(const rw::math::Q& initQ);
	
	virtual ~SimpleFinger();
	
	//! @copydoc Primitive::getParameters
	virtual rw::geometry::TriMesh::Ptr createMesh(int resolution=0) const;

	//! @copydoc Primitive::getParameters
	virtual rw::math::Q getParameters() const;
	
	//! @copydoc Primitive::setParameters
	virtual void setParameters(const rw::math::Q& q);

	//! @copydoc GeometryData::getType
	GeometryType getType() const { return UserType; }
	
	double getLength() const { return _length; }
	void setLength(double value) { _length = value; }
	double getWidth() const { return _width; }
	void setWidth(double value) { _width = value; }
	double getDepth() const { return _length; }
	void setDepth(double value) { _depth = value; }
	double getChamferLength() const { return _chflength; }
	void setChamferLength(double value) { _chflength = value; }
	double getChamferDepth() const { return _chfdepth; }
	void setChamferDepth(double value) { _chfdepth = value; }
	double getCutPosition() const { return _cutpos; }
	void setCutPosition(double value) { _cutpos = value; }
	double getCutDepth() const { return _cutdepth; }
	void setCutDepth(double value) { _cutdepth = value; }
	double getCutAngle() const { return _cutangle; }
	void setCutAngle(double value) { _cutangle = value; }
	double getCutTilt() const { return _cuttilt; }
	void setCutTilt(double value) { _cuttilt = value; }

private:
	/* general dimensions */
	double _length;
	double _width;
	double _depth;
	
	/* chamfering */
	double _chflength;
	double _chfdepth;
	
	/* cutout */
	double _cutpos;
	double _cutdepth;
	double _cutangle;
	double _cuttilt;
};

} /* simplefinger */
} /* geometry */
} /* rwlibs */

#endif
