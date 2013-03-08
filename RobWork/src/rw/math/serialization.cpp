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

#include "serializeutil.hpp"

#include <rw/math/Q.hpp>

#include "Serializable.hpp"
#include "InputArchive.hpp"
#include "OutputArchive.hpp"

using namespace rw::common;
using namespace rw::common::serialization;

namespace {

	template <class ARR>
	std::vector<double> toStdVector(const ARR& tmp, int size){
		std::vector<double> qvec(size);
        for(int i=0;i<size;i++){
        	qvec[i] = tmp[i];
        }
        return qvec;
	}

	template <class MAT>
	std::vector<double> toStdVector(const MAT& tmp, int size1, int size2){
		std::vector<double> qvec(size1*size2);
        for(int i=0;i<size1;i++){
        	for(int j=0;j<size2;j++){
        		qvec[i] = tmp(i,j);
        	}
        }
        return qvec;
	}

	template <class ARR>
	ARR fromStdVector(std::vector<double>& data, ARR& tmp){
        for(size_t i=0;i<data.size();i++){
        	tmp[i] = data[i];
        }
        return tmp;
	}

	template <class MAT>
	MAT fromStdVectorToMat(std::vector<double>& data, MAT& tmp, int size1, int size2){
        for(size_t i=0;i<size1;i++){
        	for(size_t j=0;j<size2;j++){
        		tmp(i,j) = data[i];
        	}
        }
        return tmp;
	}

}

/// WRITING
void write(const rw::math::Q& tmp, OutputArchive& oar, const std::string& id){
	oar.write( toStdVector(tmp, tmp.size()), id );
}
void write(const rw::math::Vector2D<>& tmp, OutputArchive& oar, const std::string& id){
	oar.write( toStdVector(tmp, 2), id );
}
void write(const rw::math::Vector3D<>& tmp, OutputArchive& oar, const std::string& id){
	oar.write( toStdVector(tmp, 3), id );
}
void write(const rw::math::Rotation2D<>& tmp, OutputArchive& oar, const std::string& id){
	oar.write( toStdVector(tmp, 2, 2), id );
}
void write(const rw::math::Rotation3D<>& tmp, OutputArchive& oar, const std::string& id){
	oar.write( toStdVector(tmp, 3, 3), id );
}
void write(const rw::math::Transform2D<>& tmp, OutputArchive& oar, const std::string& id){
	oar.write( toStdVector(tmp, 3, 2), id );
}
void write(const rw::math::Transform3D<>& tmp, OutputArchive& oar, const std::string& id){
	oar.write( toStdVector(tmp, 4, 3), id );
}
void write(const rw::math::Pose2D<>& tmp, OutputArchive& oar, const std::string& id){
	oar.write( toStdVector(tmp, 3), id );
}

void write(const rw::math::Jacobian& tmp, OutputArchive& oar, const std::string& id){
	oar.write( toStdVector(tmp, tmp.size1(), tmp.size2()), id );
}
void write(const rw::math::VelocityScrew6D<>& tmp, OutputArchive& oar, const std::string& id){
	oar.write( toStdVector(tmp, 6), id );
}
void write(const rw::math::Quaternion<>& tmp, OutputArchive& oar, const std::string& id){
	oar.write( toStdVector(tmp, 4), id );
}
void write(const rw::math::EAA<>& tmp, OutputArchive& oar, const std::string& id){
	oar.write( toStdVector(tmp, 3), id );
}
void write(const rw::math::RPY<>& tmp, OutputArchive& oar, const std::string& id){
	oar.write( toStdVector(tmp, 3), id );
}


//// READING

void read(rw::math::Q& tmp, InputArchive& iar, const std::string& id){
	std::vector<double> arr;
	iar.read(arr, id);
	tmp = rw::math::Q(arr.size());
	fromStdVector(arr, tmp);
}
void read(rw::math::Vector2D<>& tmp, InputArchive& iar, const std::string& id){
	std::vector<double> arr;
	iar.read(arr, id);
	fromStdVector(arr, tmp);
}
void read(rw::math::Vector3D<>& tmp, InputArchive& iar, const std::string& id){
	std::vector<double> arr;
	iar.read(arr, id);
	fromStdVector(arr, tmp);
}
void read(rw::math::Pose2D<>& tmp, InputArchive& iar, const std::string& id){
	std::vector<double> arr;
	iar.read(arr, id);
	fromStdVector(arr, tmp);
}

void read(rw::math::VelocityScrew6D<>& tmp, InputArchive& iar, const std::string& id){
	std::vector<double> arr;
	iar.read(arr, id);
	fromStdVector(arr, tmp);
}
void read(rw::math::Quaternion<>& tmp, InputArchive& iar, const std::string& id){
	std::vector<double> arr;
	iar.read(arr, id);
	fromStdVector(arr, tmp);
}
void read(rw::math::EAA<>& tmp, InputArchive& iar, const std::string& id){
	std::vector<double> arr;
	iar.read(arr, id);
	fromStdVector(arr, tmp);
}
void read(rw::math::RPY<>& tmp, InputArchive& iar, const std::string& id){
	std::vector<double> arr;
	iar.read(arr, id);
	fromStdVector(arr, tmp);
}

void read(rw::math::Rotation2D<>& tmp, InputArchive& iar, const std::string& id){
	std::vector<double> arr;
	iar.read(arr, id);
	fromStdVectorToMat(arr, tmp, 2, 2);
}
void read(rw::math::Rotation3D<>& tmp, InputArchive& iar, const std::string& id){
	std::vector<double> arr;
	iar.read(arr, id);
	fromStdVectorToMat(arr, tmp, 3, 3);
}
void read(rw::math::Transform2D<>& tmp, InputArchive& iar, const std::string& id){
	std::vector<double> arr;
	iar.read(arr, id);
	fromStdVectorToMat(arr, tmp, 3, 2);
}
void read(rw::math::Transform3D<>& tmp, InputArchive& iar, const std::string& id){
	std::vector<double> arr;
	iar.read(arr, id);
	fromStdVectorToMat(arr, tmp, 4, 3);
}
void read(rw::math::Jacobian& tmp, InputArchive& iar, const std::string& id){
	std::vector<double> arr;
	iar.read(arr, id);
	fromStdVectorToMat(arr, tmp, tmp.size1(), tmp.size2());
}

