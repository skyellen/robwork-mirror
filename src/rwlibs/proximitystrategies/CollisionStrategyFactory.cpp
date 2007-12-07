#include "CollisionStrategyFactory.hpp"

using namespace rw::proximity;
using namespace rwlibs::proximitystrategies;
/*
CollisionStrategyFactory::CollisionStrategyFactory()
{
}

CollisionStrategyFactory::~CollisionStrategyFactory()
{
}
*/

namespace {
    
    class ToleranceWrapper: public rw::proximity::CollisionStrategy {
    private:
        CollisionToleranceStrategy *_tstrategy;
        double _tolerance;
    public:
        ToleranceWrapper(CollisionToleranceStrategy* tstrategy, double tolerance):
            _tstrategy(tstrategy),_tolerance(tolerance)
        {
            
        }
        
        bool addModel(const rw::kinematics::Frame *frame){
            return _tstrategy->addModel(frame);
        }

        bool addModel(const rw::kinematics::Frame* frame, const std::vector<rw::geometry::Face<float> >& faces){
            return _tstrategy->addModel(frame,faces);
        }

        void setFirstContact(bool b){
            
        }

        bool inCollision(
            const rw::kinematics::Frame* a,
            const rw::math::Transform3D<>& wTa,
            const rw::kinematics::Frame *b,
            const rw::math::Transform3D<>& wTb){
            
            return _tstrategy->inCollision(a,wTa,b,wTb,_tolerance);
        }
        
        void clear(){
            _tstrategy->clear();
        }   
        
        bool hasModel(const rw::kinematics::Frame* frame) {
            return _tstrategy->hasModel(frame);
        }
    };
}


CollisionStrategy* CollisionStrategyFactory::NewCollisionStrategy(CollisionToleranceStrategy* tstrat, double tolerance) {
    ToleranceWrapper *wrapper = new ToleranceWrapper(tstrat,tolerance);
    return wrapper;
}
