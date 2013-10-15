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


#include "CollisionStrategy.hpp"

#include <list>
#include <stack>
#include "ProximityStrategyData.hpp"

using namespace rw::proximity;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;

namespace
{
    class ToleranceWrapper: public rw::proximity::CollisionStrategy
    {
    private:
		CollisionToleranceStrategy::Ptr _strategy;
        double _tolerance;
        rw::kinematics::FrameMap<double> _toleranceMap;
        //std::map<ProximityModel*, double> _toleranceMap;

    public:
        ToleranceWrapper(
			CollisionToleranceStrategy::Ptr strategy,
            double tolerance)
            :
            _strategy(strategy),
            _tolerance(tolerance)
        {}

        ToleranceWrapper(
            CollisionToleranceStrategy::Ptr strategy,
            rw::kinematics::FrameMap<double> toleranceMap,
            double tolerance)
            :
            _strategy(strategy),
            _tolerance(tolerance),
            _toleranceMap(toleranceMap)
        {


        }

        void setFirstContact(bool b) {}

		ProximityModel::Ptr createModel(){ return _strategy->createModel();};

        void destroyModel(ProximityModel* model){ _strategy->destroyModel(model);};

        virtual bool addGeometry(ProximityModel* model, const rw::geometry::Geometry& geom)
        { return _strategy->addGeometry(model,geom);};

        virtual bool addGeometry(ProximityModel* model, rw::geometry::Geometry::Ptr geom, bool forceCopy=false)
        { return _strategy->addGeometry(model,geom,forceCopy);};
        virtual bool removeGeometry(ProximityModel* model, const std::string& geomId)
        { return _strategy->removeGeometry(model, geomId);}

        virtual std::vector<std::string> getGeometryIDs(ProximityModel* model)
        { return _strategy->getGeometryIDs(model);}

		void getCollisionContacts(std::vector<CollisionStrategy::Contact>& contacts, ProximityStrategyData& data){
			// TODO: we need to get contacts from tolerance collision checks
		}

		bool inCollision(ProximityModel::Ptr a,
			const rw::math::Transform3D<>& wTa,
			ProximityModel::Ptr b,
			const rw::math::Transform3D<>& wTb,
			ProximityStrategyData& data)
        {
		    //double tolerance = _tolerance;
		    //if( _toleranceMap.has(a->getFrame()) ){
		    //    tolerance = _toleranceMap[a->getFrame()];
		    //}



            return _strategy->inCollision(a, wTa, b, wTb, _tolerance, data);
        }

        void clear()
        {
            _strategy->clear();
        }

    };
}

CollisionStrategy::CollisionStrategy() {}
CollisionStrategy::~CollisionStrategy() {}

CollisionStrategy::Ptr CollisionStrategy::make(
	CollisionToleranceStrategy::Ptr strategy,
    double tolerance)
{
    return ownedPtr(new ToleranceWrapper(strategy, tolerance));
}

CollisionStrategy::Ptr CollisionStrategy::make(
    CollisionToleranceStrategy::Ptr strategy,
    const rw::kinematics::FrameMap<double>& frameToTolerance,
    double defaultTolerance)
{
    return ownedPtr(new ToleranceWrapper(strategy, frameToTolerance, defaultTolerance));
}

bool CollisionStrategy::inCollision(
    const Frame* a, const Transform3D<>& wTa,
    const Frame *b, const Transform3D<>& wTb,
    QueryType type)
{
    if( getModel(a)==NULL || getModel(b)==NULL)
        return false;
    ProximityStrategyData data;

    return inCollision(getModel(a), wTa, getModel(b), wTb, data);
}

bool CollisionStrategy::inCollision(
    const Frame* a, const Transform3D<>& wTa,
    const Frame *b, const Transform3D<>& wTb,
    ProximityStrategyData& data,
    QueryType type)
{
    if( getModel(a)==NULL || getModel(b)==NULL)
        return false;
    return inCollision(getModel(a), wTa, getModel(b), wTb, data);
}
