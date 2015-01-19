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

#include "MaterialDataMap.hpp"

#include <rw/common/macros.hpp>
#include <boost/foreach.hpp>

using namespace rwsim::dynamics;


MaterialDataMap::MaterialDataMap():
	_matCnt(0)
{
    _defaultFrictionData.type = Coulomb;
    FrictionParam data;
    data.first = "mu";
    data.second = rw::math::Q(1,0.4);

    _defaultFrictionData.parameters.push_back(data);
}

MaterialDataMap::~MaterialDataMap()
{
}

void MaterialDataMap::add(const std::string& name, const std::string& desc) {
	if( _matToMatID.find(name)==_matToMatID.end() ){
		_matToMatID[name] = _matCnt;
		_mat.push_back("");
		_matCnt++;
	}

	int mat = getDataID( name );
	_matToDesc[mat] = desc;
	_mat[mat] = name;
}

int MaterialDataMap::getDataID( const std::string& material ) const {
	std::map<std::string, int>::const_iterator foundMat = _matToMatID.find(material);
	if(foundMat ==_matToMatID.end() ){
		RW_THROW("Material \"" << material << "\" does not exist!");
	}
	return foundMat->second;
}

const std::string& MaterialDataMap::getMaterialName( int id ) const {
	return _mat[id];
}

const std::vector<std::string>& MaterialDataMap::getMaterials(){
	return _mat;
}

int MaterialDataMap::getMaxMatID() const {
	return _matCnt;
}

bool MaterialDataMap::hasFrictionData(int matAID, int matBID, int dataType) const {
    MatIDPair pair(matAID,matBID);
    FrictionMap::const_iterator res = _frictionMap.find(pair);
    if( res == _frictionMap.end() )
    	return false;
    BOOST_FOREACH(const FrictionData& data, (*res).second){
        if(data.type==dataType)
        	return true;
    }
    return false;
}

bool MaterialDataMap::hasFrictionData(const std::string& matAID, const std::string& matBID, int dataType) const {
    return hasFrictionData(getDataID(matAID), getDataID(matBID), dataType);
}

const FrictionData& MaterialDataMap::getFrictionData(int materialA, int materialB, int type) const {
    //std::cout << "GET friction" << std::endl;
    MatIDPair pair(materialA,materialB);
    FrictionMap::const_iterator res = _frictionMap.find(pair);
    if( res == _frictionMap.end() ){
        RW_WARN("No FrictionData for material pair: ("<<materialA << ";" << materialB << ") or ("
                << getMaterialName(materialA) << ";" << getMaterialName(materialB) << ")");
        return getDefaultFriction(type);
    }
    BOOST_FOREACH(const FrictionData& data, (*res).second){
        if(data.type==type){
            return data;
        }
    }
    return getDefaultFriction(type);
}

const std::vector<FrictionData> MaterialDataMap::getFrictionDatas(int matAID, int matBID) const {
    const MatIDPair pair(matAID,matBID);
    const FrictionMap::const_iterator res = _frictionMap.find(pair);
    if(res != _frictionMap.end())
    	return res->second;
    return std::vector<FrictionData>();
}

const FrictionData& MaterialDataMap::getFrictionData(const std::string& materialA, const std::string& materialB, int type) const {
    return getFrictionData(getDataID(materialA), getDataID(materialB), type);
}

const std::vector<FrictionData> MaterialDataMap::getFrictionDatas(const std::string& matAID, const std::string& matBID) const {
    return getFrictionDatas(getDataID(matAID), getDataID(matBID));
}

void MaterialDataMap::addFrictionData(const std::string& materialA, const std::string& materialB, const FrictionData& data) {
    int matAID = getDataID(materialA);
    int matBID = getDataID(materialB);
    MatIDPair pairA(matAID,matBID);
    MatIDPair pairB(matBID,matAID);
    _frictionMap[pairA].push_back(data);
    if (matAID != matBID)
    	_frictionMap[pairB].push_back(data);
}

const FrictionData& MaterialDataMap::getDefaultFriction(int type) const {
	return _defaultFrictionData;
}
