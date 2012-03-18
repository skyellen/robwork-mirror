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


#include "ProximityStrategyFactory.hpp"
#include <RobWorkConfig.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>

using namespace rw::proximity;
using namespace rw::common;

namespace {
    typedef std::pair<std::string, ProximityStrategyFactory::makeCollisionStrategyFunctor> CollisionStrategyPair;
    std::vector<CollisionStrategyPair> _collisionStrategies;

    typedef std::pair<std::string, ProximityStrategyFactory::makeDistanceStrategyFunctor> DistanceStrategyPair;
    std::vector<DistanceStrategyPair> _distanceStrategies;
}


std::vector<std::string> ProximityStrategyFactory::getCollisionStrategyIDs(){
    std::vector<std::string> ids;
    BOOST_FOREACH(CollisionStrategyPair& engine, _collisionStrategies){
        ids.push_back(engine.first);
    }
    return ids;
}

rw::proximity::CollisionStrategy::Ptr ProximityStrategyFactory::makeCollisionStrategy(const std::string& id){

    BOOST_FOREACH(CollisionStrategyPair& engine, _collisionStrategies){
        if(id == engine.first){
            return ownedPtr( engine.second() );
        }
    }
    RW_THROW("No support for collision strategy with ID=" << StringUtil::quote(id));
    return NULL;
}

rw::proximity::CollisionStrategy::Ptr ProximityStrategyFactory::makeDefaultCollisionStrategy(){
    if(_collisionStrategies.empty())
        RW_THROW("There are no registered collision strategies!");
    return _collisionStrategies[0].second();
}




std::vector<std::string> ProximityStrategyFactory::getDistanceStrategyIDs(){
    std::vector<std::string> ids;
    BOOST_FOREACH(DistanceStrategyPair& engine, _distanceStrategies){
        ids.push_back(engine.first);
    }
    return ids;
}

rw::proximity::DistanceStrategy::Ptr ProximityStrategyFactory::makeDistanceStrategy(const std::string& id){

    BOOST_FOREACH(DistanceStrategyPair& engine, _distanceStrategies){
        if(id == engine.first){
            return ownedPtr( engine.second() );
        }
    }
    RW_THROW("No support for distance strategy with ID=" << StringUtil::quote(id));
    return NULL;
}

rw::proximity::DistanceStrategy::Ptr ProximityStrategyFactory::makeDefaultDistanceStrategy(){
    if(_distanceStrategies.empty())
        RW_THROW("There are no registered distance strategies!");
    return _distanceStrategies[0].second();
}



void ProximityStrategyFactory::addCollisionStrategy(const std::string& ID, ProximityStrategyFactory::makeCollisionStrategyFunctor constructor){
    std::cout << "1";
    BOOST_FOREACH(CollisionStrategyPair& engine, _collisionStrategies){
        std::cout << "2";
        if(ID == engine.first){
            engine.second = constructor;
            return;
        }
    }
    std::cout << "3";
    _collisionStrategies.push_back( std::make_pair(ID, constructor) );
}


void ProximityStrategyFactory::addDistanceStrategy(const std::string& ID, ProximityStrategyFactory::makeDistanceStrategyFunctor constructor){
    std::cout << "5";
    BOOST_FOREACH(DistanceStrategyPair& engine, _distanceStrategies){
        std::cout << "6";
        if(ID == engine.first){
            engine.second = constructor;
            return;
        }
    }
    std::cout << "7";
    _distanceStrategies.push_back( std::make_pair(ID, constructor) );
}








