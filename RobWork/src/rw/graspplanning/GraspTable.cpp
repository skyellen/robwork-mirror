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

#include "GraspTable.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/math/Pose6D.hpp>

#include <fstream>

using namespace rw::math;
using namespace rw::graspplanning;

GraspTable::GraspTable(const std::string& handName,  const std::string& objectId):
    _handName(handName),_objectId(objectId),_calibForceIndex(-1)
{

}

void GraspTable::addGrasp(GraspData& data)
{
//	std::cout << "Adding grasp";
    _graspData.push_back(data);
}

rw::common::Ptr<GraspTable> GraspTable::load(const std::string& filename){
	const int linesize = 1000;
	char line[linesize],chunk[100],handname[100],objectId[100];
	handname[0] = 0; // make sure that if data has noname it will have no name
	objectId[0] = 0;
	//std::cout << filename << std::endl;
	std::ifstream istr( filename.c_str() );
	if( !istr.is_open() )
		RW_THROW("Could not open file: "<< filename);
	//std::cout << "File openned!" << std::endl;
	unsigned int tablesize, handdof, nrquality,version;
	istr.getline(line, linesize);
	//std::cout << "Got line!" << std::endl;
	sscanf (line,"%s %i", chunk, &version);
	//std::cout << "Scanning line!" << std::endl;
	//std::cout << "GraspTableVersion: " << version << std::endl;
	if(version!=GTABLE_VERSION)
		RW_THROW("VERSION incompatibility, version of file is: "<<version<<", version of loader is: "<< GTABLE_VERSION);

	istr.getline(line, linesize);
	sscanf (line,"%s %s",chunk, handname);
	istr.getline(line, linesize);
	sscanf (line,"%s %s",chunk, objectId);
	istr.getline(line, linesize);
	sscanf (line,"%s %i",chunk,&tablesize);
    // fstr << "TableSize: " << _graspData.size() << "\n";

    GraspTable *gtable = new GraspTable(handname,objectId);

    //std::cout << "hand: " << handname << std::endl;
    //std::cout << "object: " << objectId << std::endl;
    //std::cout << "TableSize: " << tablesize << std::endl;

	if(tablesize==0)
		return gtable;

	istr.getline(line, linesize);
	sscanf (line,"%s %i",chunk,&handdof);
	istr.getline(line, linesize);
	sscanf (line,"%s %i",chunk,&nrquality);

	//std::cout << "HandDOF: " << handdof << "\n";
    //std::cout << "NrOfQualityMeasures: " << nrquality << "\n";

    int calibForceIndex;
    istr.getline(line, linesize);
    sscanf (line,"%s %i",chunk,&calibForceIndex);
    //std::cout << "CalibrationForceIndex: " << calibForceIndex << "\n";
    gtable->setCalibForceIndex(calibForceIndex);
    char tmpC;

    for(size_t i=0;i<tablesize;i++){
    	GraspData data;
		float a[3]; 
    	
    	float hp[6], op[6];
    	istr >> a[0] >> tmpC >> a[1] >> tmpC >> a[2] >> tmpC;
    	data.approach = Vector3D<>(a[0],a[1],a[2]);
    	//std::cout << "1";

    	if(nrquality>0){
			float qual;
			data.quality = Q(nrquality);
			for(size_t j=0;j<nrquality;j++){
				istr >> qual>> tmpC;
				data.quality[j] = qual;
			}			
    	}
    	//std::cout << "2";
    	data.pq = Q(handdof);
    	float pq;
		for(size_t j=0;j<handdof;j++){
    		istr >> pq>> tmpC;
    		data.pq[j] = pq;
    	}
    	//std::cout << "3";
    	istr >> hp[0] >> tmpC >> hp[1] >> tmpC >> hp[2] >> tmpC
			 >> hp[3] >> tmpC >> hp[4] >> tmpC >> hp[5] >> tmpC;
    	data.hp = Pose6D<>(hp[0],hp[1],hp[2],hp[3],hp[4],hp[5]);

    	//std::cout << "4";
    	istr >> op[0] >> tmpC >> op[1] >> tmpC >> op[2] >> tmpC
			 >> op[3] >> tmpC >> op[4] >> tmpC >> op[5] >> tmpC;
    	data.op = Pose6D<>(op[0],op[1],op[2],op[3],op[4],op[5]);
    	//std::cout << "5";
    	size_t cqsize;
    	istr >> cqsize >> tmpC;
    	//std::cout << "6";
    	data.cq = Q(cqsize);
    	float cq;
    	for(size_t j=0;j<cqsize;j++){
    		istr >> cq >> tmpC;
    		data.cq[j] = cq;
    	}
    	//std::cout << "7";
    	size_t consize;
    	istr >> consize >> tmpC;
    	//std::cout << "8";
    	data.grasp = Grasp3D((int)consize);
    	float cs[3];
    	for(size_t j=0;j<consize;j++){
    		for(size_t m=0;m<3;m++){
    			istr >> cs[m] >> tmpC;
    			data.grasp.contacts[j].p[m] = cs[m];
    		}
    		for(size_t m=0;m<3;m++){
    			istr >> cs[m] >> tmpC;
    			data.grasp.contacts[j].n[m] = cs[m];
    		}
    	}
    	//std::cout << "9" << std::endl;
    	// read tactile data
        // and now the tactile data
    	size_t nrsensors;
    	istr >> nrsensors >> tmpC;
    	//std::cout << "Nr sensors: " << nrsensors << std::endl;
    	data._tactiledata.resize(nrsensors);
        for(size_t j=0;j<nrsensors;j++){
        	// write dimensions of pad
        	size_t xdim, ydim;
        	istr >> xdim >> tmpC;
        	istr >> ydim >> tmpC;
        	rw::sensor::TactileArray::ValueMatrix mat(xdim,ydim);
        	for(size_t x=0;x<xdim;x++)
        		for(size_t y=0;y<ydim;y++){
        			float val;
        			istr >> val >> tmpC;
        			mat(x,y) = val;
        		}
        	//std::cout << xdim << ";" << ydim << ";"<<
        	data._tactiledata[j] = mat;
        }

    	size_t nrsencon;
    	istr >> nrsencon >> tmpC;
    	//std::cout << "Nr sensors: " << nrsensors << std::endl;
    	data.tactileContacts.resize(nrsencon);
        for(size_t j=0;j<nrsencon;j++){
        	// write dimensions of pad

        	size_t len;
        	istr >> len >> tmpC;
        	data.tactileContacts[j].resize(len);
        	for(size_t x=0;x<len;x++){
        		rw::sensor::Contact3D &con = (data.tactileContacts[j])[x];
				istr >> con.p(0) >> tmpC >> con.p(1) >> tmpC >> con.p(2) >> tmpC;
				istr >> con.n(0) >> tmpC >> con.n(1) >> tmpC >> con.n(2) >> tmpC;
				istr >> con.f(0) >> tmpC >> con.f(1) >> tmpC >> con.f(2) >> tmpC;
			}
        	//std::cout << xdim << ";" << ydim << ";"<<
        }
    	gtable->addGrasp(data);
		
    }

	return rw::common::ownedPtr( gtable );
}

void GraspTable::save(const std::string& filename){
    std::ofstream fstr(filename.c_str());
    //std::cout << "saving grasp stuff" << std::endl;
    fstr << "GraspTableVersion: " << GTABLE_VERSION << "\n";
    fstr << "hand: " << _handName << "\n";
    //std::cout << "hand: " << _handName << "\n";
    fstr << "object: " << _objectId << "\n";
    //std::cout << "object: " << _objectId << "\n";
    fstr << "TableSize: " << _graspData.size() << "\n";
    //std::cout << "TableSize: " << _graspData.size() << "\n";
    if(_graspData.size()==0){
    	fstr.close();
    	return;
    }
    fstr << "HandDOF: " << _graspData[0].pq.size() << "\n";
    //std::cout << "HandDOF: " << _graspData[0].pq.size() << "\n";
    fstr << "NrOfQualityMeasures: " << _graspData[0].quality.size() << "\n";
    //std::cout << "NrOfQualityMeasures: " << _graspData[0].quality.size() << "\n";
    fstr << "CalibrationForceIndex: " << _calibForceIndex << "\n";

    // each data entry is printed on a line
    for(size_t i=0;i<_graspData.size();i++){
        GraspData &data = _graspData[i];

        // first we print the approach vector
        fstr << data.approach[0] << ";" << data.approach[1] << ";" << data.approach[2] << ";";
        //std::cout << data.approach[0] << ";" << data.approach[1] << ";" << data.approach[2] << ";";
        // then we print the quality
        for(size_t j=0;j<data.quality.size();j++){
            fstr << data.quality[j] << ";";
            //std::cout << data.quality[j] << ";";
        }

        // then the preshape configuration
        for(size_t j=0;j<data.pq.size();j++){
            fstr << data.pq[j] << ";";
            //std::cout << data.pq[j] << ";";
        }

        // hand pose
        for(size_t j=0;j<6;j++){
            fstr << data.hp.get(j) << ";";
            //std::cout << data.hp.get(j) << ";";
        }

        // object pose
        for(size_t j=0;j<6;j++){
            fstr << data.op.get(j) << ";";
            //std::cout <<data.op.get(j) << ";";
        }

        // then we print the contact configuration
        fstr << data.cq.size() << ";";
        //std::cout << data.cq.size() << ";";
        for(size_t j=0;j<data.cq.size();j++){
            fstr << data.cq[j] << ";";
            //std::cout << data.cq[j] << ";";
        }

        // now comes the actual contacts
        fstr << data.grasp.contacts.size() << ";";
        for(size_t j=0;j<data.grasp.contacts.size();j++){
            fstr << data.grasp.contacts[j].p[0] << ";";
            fstr << data.grasp.contacts[j].p[1] << ";";
            fstr << data.grasp.contacts[j].p[2] << ";";
            fstr << data.grasp.contacts[j].n[0] << ";";
            fstr << data.grasp.contacts[j].n[1] << ";";
            fstr << data.grasp.contacts[j].n[2] << ";";
        }

        // and now the tactile data
        fstr << data._tactiledata.size() << ";";

        for(size_t j=0;j<data._tactiledata.size();j++){
        	// write dimensions of pad
        	fstr << data._tactiledata[j].rows() << ";";
        	fstr << data._tactiledata[j].cols() << ";";
        	for(Eigen::DenseIndex x=0;x<data._tactiledata[j].rows();x++)
        		for(Eigen::DenseIndex y=0;y<data._tactiledata[j].cols();y++)
        			fstr << data._tactiledata[j](x,y) << ";";

        }

        // now for the tactile contacts, if there is any
        fstr << data.tactileContacts.size() << ";";

        for(size_t j=0;j<data.tactileContacts.size();j++){
        	// write dimensions of pad
        	fstr << data.tactileContacts[j].size() << ";";
        	for(size_t x=0;x<data.tactileContacts[j].size();x++){
        		// pos, norm, force
                fstr << (data.tactileContacts[j])[x].p[0] << ";";
                fstr << (data.tactileContacts[j])[x].p[1] << ";";
                fstr << (data.tactileContacts[j])[x].p[2] << ";";
                fstr << (data.tactileContacts[j])[x].n[0] << ";";
                fstr << (data.tactileContacts[j])[x].n[1] << ";";
                fstr << (data.tactileContacts[j])[x].n[2] << ";";
                fstr << (data.tactileContacts[j])[x].f[0] << ";";
                fstr << (data.tactileContacts[j])[x].f[1] << ";";
                fstr << (data.tactileContacts[j])[x].f[2] << ";";
        	}
        }


        // we end the data entry with a endline
        fstr << std::endl;

        //std::cout << std::endl;
    }

    fstr.close();
}


int GraspTable::nrTactileArrayGrasp(){
	if(_graspData.size()==0)
		return 0;
	return (int)_graspData[0]._tactiledata.size();
}

std::pair<int,int> GraspTable::getTactileArrayDim(int i){
	if(_graspData.size()==0)
		return std::pair<int,int>(0,0);
	if( i>=(int)_graspData[0]._tactiledata.size() )
		RW_THROW("Index i out of range! " << i << ">=" << _graspData[0]._tactiledata.size());
	int s1 = (int)_graspData[0]._tactiledata[i].rows();
	int s2 = (int)_graspData[0]._tactiledata[i].cols();
	return std::make_pair(s1,s2);
}

bool GraspTable::hasCalibForce(){
	return _calibForceIndex!=-1;
}

int GraspTable::getCalibForceIndex(){
	return _calibForceIndex;
}


