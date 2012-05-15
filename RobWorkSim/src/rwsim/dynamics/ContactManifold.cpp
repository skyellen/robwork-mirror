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

#include "ContactManifold.hpp"

using namespace rwsim::dynamics;

/*
namespace {

    struct Bucket {
    public:
        Bucket(int initSize):points(initSize),_top(0),_normal(0,0,0){}
        virtual Bucket( ) {}

        void addPoint(ContactPoint *point, Vector3D<> normal){
            RW_ASSERT(_top< _points.size() );
            _points[++_top] = point;
            _avgNormal += normal;
        }

        Vector3D<> getAvgNormal(){
            if(_top==0)
                return Vector3D<>(0,0,0);
            return _avgNormal/_top;
        }

        int size(){return _top;};

    private:
        int _top;
        Vector3D<> _avgNormal;
        std::vector<ContactPoint*> _points;
    };

    int getHashValue64(Vector3D<>& n){
        int index = 0;
        if( n(0)>0 ) index |= 1;
        if( fabs(n(0))>0.5 ) index |= 2;
        if( n(1)>0 ) index |= 4;
        if( fabs(n(1))>0.5 ) index |= 8;
        if( n(2)>0 ) index |= 16;
        if( fabs(n(2))>0.5 ) index |= 32;
        return index;
    }

}
*/
void ContactManifold::genThresContactManifolds(
    std::vector<ContactPoint>& src,
    std::vector<ContactManifold*>& manifolds,
    double thres)
{
    // 1. clustering locate the deepest penetrating point
    std::vector<ContactPoint> dst(src.size());
    std::vector<int> srcIdx(src.size());
    std::vector<int> dstIdx(src.size());

}

void ContactManifold::generateContactManifolds(
    std::vector<ContactPoint*>& points,
    std::vector<ContactManifold*>& manifolds)
{
/*    std::vector<int> validBuckets(64);
    int nrOfValidBuckets = 0;

    // 1. cluster points together using their normal
    std::vector<Bucket> buckets(64);
    BOOST_FOREACH(ContactPoint *point, points){
        int index = getHashValue64( point->n );
        if( buckets[index].size()==0 ) {
            validBuckets[++nrOfValidBuckets] = index;
        }
        buckets[index].addPoint( point, point->n);
    }
    // and now use the position to split the buckets
    //BOOST_FOREACH(Bucket &bucket, buckets){
    //}

    // 2. for each cluster identify if any manifold is close
    for(int i=0; i<nrOfValidBuckets; i++){

    }
    BOOST_FOREACH(ContactManifold *manifold, manifolds){
        int index = getHashValue64( manifold->getNormal() );

        if( buckets[index].size()>0 ){

        }

    }

    // 3. update all manifolds with clusters that are close
*/
}
