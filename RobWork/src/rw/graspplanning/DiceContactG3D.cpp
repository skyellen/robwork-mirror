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

#include "DiceContactG3D.hpp"
#include "ContactValidateFilter.hpp"
#include "Grasp3D.hpp"

#include <rw/math/Random.hpp>
#include <rw/common/Timer.hpp>
#include <rw/geometry/Triangle.hpp>
#include <rw/geometry/TriMesh.hpp>

using namespace rw::geometry;
using namespace rw::common;
using namespace rw::math;
using namespace rw::graspplanning;

namespace {

    Vector3D<> calcRandomPtInTriangle(const Triangle<>& tri){
        double b0 = Random::ran(0.0,1.0);
        double b1 = ( 1.0f - b0 ) * Random::ran(0.0,1.0);
        double b2 = 1 - b0 - b1;

        Vector3D<> vertex1 = tri[1]-tri[0];
        Vector3D<> vertex2 = tri[2]-tri[1];
        Vector3D<> vertex3 = tri[0]-tri[2];

        return tri[0] + vertex1 * b0 + vertex2 * b1 + vertex3 * b2;
    }

    Vector3D<> calcCenterPtInTriangle(const Triangle<>& tri){
        return (tri[2]+tri[1]+tri[0])/3;
    }

}

DiceContactG3D::DiceContactG3D():
	_obj(NULL),
	_mu(0.5),
	_nrOfContacts(3)
{

}

void DiceContactG3D::initialize(const TriMesh& obj, int nrOfContacts, double mu){
	_nrOfContacts = nrOfContacts;
	_mu = mu;
	_obj = &obj;
	_surfNormals.resize( obj.getSize() );
	// TODO: create the vector of surface normals
	for(size_t i=0;i<obj.getSize();i++){
		_surfNormals[i] = obj.getTriangle(i).calcFaceNormal();
		//std::cout << "Surf normal: " << _surfNormals[i] << std::endl;
	}
}

void DiceContactG3D::setContactFilter(ContactValidateFilter* filter){
    _cfilter = filter;
}

std::vector<Grasp3D> DiceContactG3D::generateContactSet(int maxNrOfContacts, double timeout)
{
	std::vector<Grasp3D> grasps;
    const int min = 0;
    const int max = (int)_surfNormals.size();
    const double avgScale = 1.0/_nrOfContacts;
    Grasp3D tmpGrasp(_nrOfContacts);

    //std::vector<Vector3D<> > normals(_nrOfContacts);
    //std::vector<Vector3D<> > points(_nrOfContacts);
    //std::vector<int> faceIdxs(_nrOfContacts);

    double atanMU = atan(_mu);
    Timer timer;
    int nrOfTries = 0;
    while(grasps.size()<(size_t)maxNrOfContacts && timer.getTime()<timeout){
        nrOfTries++;
		// generate a randomly choosen grasp contact
		for(int i=0; i<_nrOfContacts;i++){
            int idx = Random::ranI(min,max);
            int idx2 = Random::ranI(min,max);
            double area1 = _obj->getTriangle(idx).calcArea();
            double area2 = _obj->getTriangle(idx2).calcArea();
            if(area2>area1)
                idx = idx2;

            tmpGrasp.contacts[i]._faceIdx = idx;
            tmpGrasp.contacts[i].n = _transform.R() * _surfNormals[idx];
            tmpGrasp.contacts[i].p = _transform * calcCenterPtInTriangle( _obj->getTriangle( idx ) );
            //tmpGrasp.contacts[i].p = _obj->getTriangle( idx )[0];

            if(_cfilter){
                if(! _cfilter->isValid(tmpGrasp.contacts[i]) ){
                    i--;
                    continue;
                }
            }
			//std::cout << "N" << i << ": " << normals[i] << std::endl;
		}

		// calculate the average of all contact normals and take the opposite
		// as guess for Fext
		Vector3D<> fext2(0,0,0);
		for(int i=0; i<_nrOfContacts;i++){
			fext2 += tmpGrasp.contacts[i].n;
		}
		fext2 = -(fext2*avgScale);

		// now check if the approximated fext is able to break the force-closure
		bool isGoodGrasp = false;
		for(int i=0; i<_nrOfContacts;i++){
			double angle = acos( dot(tmpGrasp.contacts[i].n,fext2) );
			//std::cout << "fabs(angle) < Pi/2+atanMU  --> " << fabs(angle) <<" < "<<  Pi/2+atanMU << std::endl;
			if( fabs(angle) < Pi/2+atanMU ){
				isGoodGrasp = true;
				break;
			}
		}

		if(isGoodGrasp){
			// add the grasp to the candidate grasps
			grasps.push_back(tmpGrasp);
		}
    }
    std::cout << "NrofTries: " << nrOfTries << std::endl;
    return grasps;
}

Grasp3D DiceContactG3D::generateNext(){
    Grasp3D tmpGrasp(_nrOfContacts);
    const int min = 0;
    const int max = (int)_surfNormals.size();
    const double avgScale = 1.0/_nrOfContacts;
    double atanMU = atan(_mu);
    Timer timer;

    while(timer.getTime()<1.0){

        // generate a randomly choosen grasp contact
        for(int i=0; i<_nrOfContacts;i++){
            int idx = Random::ranI(min,max);
            int idx2 = Random::ranI(min,max);
            double area1 = _obj->getTriangle(idx).calcArea();
            double area2 = _obj->getTriangle(idx2).calcArea();
            if(area2>area1)
                idx = idx2;

            tmpGrasp.contacts[i]._faceIdx = idx;
            tmpGrasp.contacts[i].n =_transform.R()* _surfNormals[idx];
            tmpGrasp.contacts[i].p = _transform * calcCenterPtInTriangle( _obj->getTriangle( idx ) );
            //tmpGrasp.contacts[i].p = _obj->getTriangle( idx )[0];

            if(_cfilter){
                if(! _cfilter->isValid(tmpGrasp.contacts[i]) ){
                    i--;
                    continue;
                }
            }
            //std::cout << "N" << i << ": " << normals[i] << std::endl;
        }

        // calculate the average of all contact normals and take the opposite
        // as guess for Fext
        Vector3D<> fext2;
        for(int i=0; i<_nrOfContacts;i++){
            fext2 += tmpGrasp.contacts[i].n;
        }
        fext2 = -(fext2*avgScale);

        // now check if the approximated fext is able to break the force-closure
        bool isGoodGrasp = false;
        for(int i=0; i<_nrOfContacts;i++){
            double angle = acos( dot(tmpGrasp.contacts[i].n,fext2) );
            //std::cout << "fabs(angle) < Pi/2+atanMU  --> " << fabs(angle) <<" < "<<  Pi/2+atanMU << std::endl;
            if( fabs(angle) < Pi/2+atanMU ){
                isGoodGrasp = true;
                break;
            }
        }

        if(isGoodGrasp){
            // add the grasp to the candidate grasps
            return tmpGrasp;
        }
    }
    return tmpGrasp;
}

