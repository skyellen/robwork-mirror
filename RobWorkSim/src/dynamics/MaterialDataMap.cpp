#include "MaterialDataMap.hpp"

#include <rw/common/macros.hpp>
#include <boost/foreach.hpp>

using namespace dynamics;


MaterialDataMap::MaterialDataMap():
	_matCnt(0)
{
}

MaterialDataMap::~MaterialDataMap()
{
}

const FrictionData&
    MaterialDataMap::getFrictionData(int materialA,
                                     int materialB,
                                     int type)
{
    //std::cout << "GET friction" << std::endl;
    MatIDPair pair(materialA,materialB);
    FrictionMap::iterator res = _frictionMap.find(pair);
    if( res == _frictionMap.end() ){
        RW_WARN("No FrictionData for material pair: ("<<
                 materialA << ";" << materialB << ")");
        return getDefaultFriction(type);
    }
    BOOST_FOREACH(FrictionData& data, (*res).second){
        if(data.type==type){
            return data;
        }
    }
    return getDefaultFriction(type);
}

void MaterialDataMap::addFrictionData(const std::string& materialA,
                        const std::string& materialB,
                        const FrictionData& data)
{
    int matAID = getDataID(materialA);
    int matBID = getDataID(materialB);
    MatIDPair pairA(matAID,matBID);
    MatIDPair pairB(matBID,matAID);
    _frictionMap[pairA].push_back(data);
    _frictionMap[pairB].push_back(data);
}
