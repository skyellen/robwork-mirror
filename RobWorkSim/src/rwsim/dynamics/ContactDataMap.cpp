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

#include "ContactDataMap.hpp"

#include <rw/common/macros.hpp>
#include <boost/foreach.hpp>
using namespace rwsim::dynamics;

ContactDataMap::ContactDataMap():
    _objectCnt(0)
{
}

ContactDataMap::~ContactDataMap()
{
}

int ContactDataMap::getDataID(const std::string& objType) const {
	const std::map<std::string, int>::const_iterator it = _nameToID.find(objType);
    if (it == _nameToID.end())
        RW_THROW("Object type \"" << objType << "\" does not exist!");
    else
    	return it->second;
}

void ContactDataMap::addNewtonData(const std::string &nameA,
                                   const std::string &nameB,
                                   const NewtonData& data)
{
    int idA = getDataID(nameA);
    int idB = getDataID(nameB);
    _newtonDataMap[std::make_pair(idA,idB)] = data;
    _newtonDataMap[std::make_pair(idB,idA)] = data;
}


void ContactDataMap::addChatterjeeData(const std::string &nameA,
                                       const std::string &nameB,
                                       const ChatterjeeData& data)
{
    int idA = getDataID(nameA);
    int idB = getDataID(nameB);
    _chatterjeeDataMap[std::make_pair(idA,idB)] = data;
    _chatterjeeDataMap[std::make_pair(idB,idA)] = data;
}

const ContactDataMap::NewtonData&
    ContactDataMap::getNewtonData(const std::string &nameA, const std::string &nameB) const
{
    int idA = getDataID(nameA);
    int idB = getDataID(nameB);
    return getNewtonData(idA,idB);
}



const ContactDataMap::NewtonData& ContactDataMap::getNewtonData(int idA, int idB) const
{
    NewtonMap::const_iterator res = _newtonDataMap.find( std::make_pair(idA,idB) );
    if( res == _newtonDataMap.end() ){
        typedef std::map<std::string, int>::value_type BVal;
        BOOST_FOREACH(BVal val, _nameToID){
            std::cout << val.first << " --> " << val.second << std::endl;
        }
        RW_THROW("NewtonData not available! idA:"<< idA << ", idB:" << idB);
    }
    return (*res).second;
}

const ContactDataMap::ChatterjeeData& ContactDataMap::getChatterjeeData(const std::string &nameA,
                                                 const std::string nameB) const
{
    int idA = getDataID(nameA);
    int idB = getDataID(nameB);
    const ChatterjeeMap::const_iterator res = _chatterjeeDataMap.find(std::make_pair(idA,idB));
    if (res == _chatterjeeDataMap.end())
        RW_THROW("Data not available!");
    else
    	return (*res).second;

}
