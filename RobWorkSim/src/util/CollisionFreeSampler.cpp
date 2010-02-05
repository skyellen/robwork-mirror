#include "CollisionFreeSampler.hpp"

using namespace rw::proximity;

CollisionFreeSampler::CollisionFreeSampler(StateSamplerPtr sampler,
                                           CollisionDetectorPtr detector, int n) :
    _sampler(sampler), _detector(detector), _n(n)
{

}


CollisionFreeSampler::~CollisionFreeSampler(){}


bool CollisionFreeSampler::sample(rw::kinematics::State& state){
    bool colfree = false;
    int maxTries = _n;
    do{

        if(!_sampler->sample(state))
            return false;

        colfree = _detector->inCollision(state,NULL,true);

        if(maxTries>0)
            maxTries--;

    } while(colfree && maxTries!=0);

    if(maxTries == 0)
        return false;

    return true;
}

