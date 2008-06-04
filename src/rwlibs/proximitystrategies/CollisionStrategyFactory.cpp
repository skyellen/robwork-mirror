#include "CollisionStrategyFactory.hpp"

using namespace rw::proximity;
using namespace rwlibs::proximitystrategies;

namespace
{
    class ToleranceWrapper: public rw::proximity::CollisionStrategy
    {
    private:
        CollisionToleranceStrategy* _strategy;
        double _tolerance;

    public:
        ToleranceWrapper(
            CollisionToleranceStrategy* strategy,
            double tolerance)
            :
            _strategy(strategy),
            _tolerance(tolerance)
        {}

        bool addModel(const rw::kinematics::Frame *frame)
        {
            return _strategy->addModel(frame);
        }

        bool addModel(
            const rw::kinematics::Frame* frame,
            const std::vector<rw::geometry::Face<float> >& faces)
        {
            return _strategy->addModel(frame, faces);
        }

        void setFirstContact(bool b) {}

        bool inCollision(
            const rw::kinematics::Frame* a,
            const rw::math::Transform3D<>& wTa,
            const rw::kinematics::Frame *b,
            const rw::math::Transform3D<>& wTb)
        {
            return _strategy->inCollision(a, wTa, b, wTb, _tolerance);
        }

        void clear()
        {
            _strategy->clear();
        }

        void clearFrame(const rw::kinematics::Frame* frame)
        {
            _strategy->clearFrame(frame);
        }

        bool hasModel(const rw::kinematics::Frame* frame)
        {
            return _strategy->hasModel(frame);
        }
    };
}

CollisionStrategy* CollisionStrategyFactory::NewCollisionStrategy(
    CollisionToleranceStrategy* strategy,
    double tolerance)
{
    return new ToleranceWrapper(strategy, tolerance);
}
