#include "ContactDataMap.hpp"

#include <rw/common/macros.hpp>

using namespace dynamics;

ContactDataMap::ContactDataMap():
    _objectCnt(0)
{
}

ContactDataMap::~ContactDataMap()
{
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
    ContactDataMap::getNewtonData(const std::string &nameA, const std::string &nameB)
{
    int idA = getDataID(nameA);
    int idB = getDataID(nameB);
    return getNewtonData(idA,idB);
}

const ContactDataMap::NewtonData& ContactDataMap::getNewtonData(int idA, int idB)
{
    NewtonMap::iterator res = _newtonDataMap.find( std::make_pair(idA,idB) );
    if( res == _newtonDataMap.end() )
        RW_THROW("Data not available!");
    return (*res).second;
}

const ContactDataMap::ChatterjeeData& ContactDataMap::getChatterjeeData(const std::string &nameA,
                                                 const std::string nameB)
{
    int idA = getDataID(nameA);
    int idB = getDataID(nameB);
    ChatterjeeMap::iterator res = _chatterjeeDataMap.find(std::make_pair(idA,idB));
    if( res == _chatterjeeDataMap.end() )
        RW_THROW("Data not available!");
    return (*res).second;

}
