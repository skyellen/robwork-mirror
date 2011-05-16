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

#ifndef RW_PROXIMITY_COLLISIONDATA_HPP_
#define RW_PROXIMITY_COLLISIONDATA_HPP_

#include "ProximityModel.hpp"
#include "ProximityCache.hpp"

namespace rw {
namespace proximity {
    //! @addtogroup proximity
    // @{
    //! @file rw/proximity/CollisionData.hpp


    typedef enum{FirstContact, AllContacts} CollisionQueryType;

    /**
     * @brief result of a single collision pair
     *
     * A collision result is one or all colliding triangles between two objects which may have
     * several geometries attached.
     * The collision result does not have access to the actual triangle meshes of the geometries
     * so to extract the actual contact location the user has to supply the triangles meshes of
     * the geometries himself.
     *
     */
    struct CollisionResult
    {
        ProximityModel::Ptr a;

        //! @brief reference to the second model
        ProximityModel::Ptr b;

        //! @brief a collision pair of
        struct CollisionPair {
            //! @brief geometry index
            int geoIdxA, geoIdxB;
            /**
             *  @brief indices into the geomPrimIds array, which means that inidicies [_geomPrimIds[startIdx];_geomPrimIds[startIdx+size]]
             *  are the colliding primitives between geometries geoIdxA and geoIdxB
             */
            int startIdx, size;
        };

        rw::math::Transform3D<> _aTb;

        std::vector<CollisionPair> _collisionPairs;

        /**
         * @brief indices of triangles/primitives in geometry a and b that are colliding
         * all colliding triangle indices are in this array also those that are from different geometries
         */
        std::vector<std::pair<int, int> > _geomPrimIds;


        void clear(){
            _collisionPairs.clear();
            _geomPrimIds.clear();
        }
    };

    /**
     * @brief DistanceResult contains basic information about the distance
     * result between two frames.
     */
    struct DistanceResult {
         //! @brief reference to the first frame
        const kinematics::Frame* f1;
        ProximityModel::Ptr a;

        //! @brief reference to the second frame
        const kinematics::Frame* f2;
        ProximityModel::Ptr b;

        //! Closest point on f1 to f2, described in f1 reference frame
        math::Vector3D<double> p1;

        //! Closest point on f2 to f1, described in >>>> \b f1 <<<<< reference frame
        math::Vector3D<double> p2;

        //! @brief distance between frame f1 and frame f1
        double distance;

        //! @brief geometry index
        int geoIdxA, geoIdxB;

        //! @brief index to the two faces/triangles that is the closest feature
        unsigned int idx1,idx2;

        void clear(){

        }
    };

    /**
     * @brief DistanceResult contains basic information about the distance
     * result between two frames.
     */
    struct MultiDistanceResult {
         //! @brief reference to the first frame
        //const kinematics::Frame* f1;
        ProximityModel::Ptr a;

        //! @brief reference to the second frame
        //const kinematics::Frame* f2;
        ProximityModel::Ptr b;

        //! Closest point on f1 to f2, described in f1 reference frame
        math::Vector3D<double> p1;

        //! Closest point on f2 to f1, described in f2 reference frame
        math::Vector3D<double> p2;

        //! @brief distance between frame f1 and frame f2
        double distance;

        //! Closest points on f1 to f2, described in f1 reference frame
        std::vector< math::Vector3D<> > p1s;

        //! IMPORTANT! NOTICE! VERY UGLY: Closest point on f2 to f1, described in >>>> \b f1 <<<<< reference frame
        std::vector< math::Vector3D<> > p2s;

        //! distances between contact points
        std::vector< double > distances;

        void clear(){
            p1s.clear();
            p2s.clear();
            distances.clear();
        }
    };


    /***
     * @brief A generic object for containing data that is essential in
     * proximity queries between two ProximityModels.
     *
     * The ProximityData object is used for Collision queries, tolerance and distance queries between
     * two ProximityModels.
     * example: collision result, cached variables for faster collision detection,
     *
     */
    class ProximityStrategyData
    {
    public:
        typedef enum{CollisionData=1, TolleranceData=2, DistanceData=4} DataType;

        ProximityStrategyData():
            rel_err(0),
            abs_err(0),
            _colQueryType(FirstContact),
            _collides(false)
        {}

        bool has(DataType type){ return _dataType && type;};
        ProximityCache::Ptr& getCache(){ return _cache; }
        ProximityModel::Ptr& getA(){ return _a; }
        ProximityModel::Ptr& getB(){ return _b; }
        rw::math::Transform3D<> aTb(){ return _aTb; }


        // CollisionData interface
        CollisionResult& getCollisionData(){ return _collisionData;}
        bool inCollision(){ return _collides; }
        void setCollisionQueryType(CollisionQueryType qtype){ _colQueryType = qtype; }
        CollisionQueryType getCollisionQueryType(){ return _colQueryType; };

        // Distance query interfaces
        DistanceResult& getDistanceData(){ return _distanceData;}
        //double getDistance(){ return _distanceData.distance; }

        // For Multi distance interface
        MultiDistanceResult& getMultiDistanceData(){ return _multiDistanceData;}
        //double getMultiDistance(){ return _multiDistanceData.distance; }


        DistanceResult _distanceData;
        MultiDistanceResult _multiDistanceData;

        /*
        * @param rel_err [in] relative acceptable error
        *
        * @param abs_err [in] absolute acceptable error
        */
        double rel_err;
        double abs_err;

    private:
        //! @brief transform from model a to model b
        rw::math::Transform3D<> _aTb;
        //! @brief the two models that where tested
        ProximityModel::Ptr _a, _b;

        CollisionQueryType _colQueryType;

        // Following belongs to CollisionData interface
        //! true if the models are colliding
        bool _collides;
        //! @brief the features that where colliding
        CollisionResult _collisionData;

        //! @brief proximity cache
        ProximityCache::Ptr _cache;

        int _dataType;
    };
    // @}
}
}

#endif /* COLLISIONDATA_HPP_ */
