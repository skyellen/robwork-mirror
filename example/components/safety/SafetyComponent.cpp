#include "SafetyComponent.hpp"

#include <core/math/Transform3D.hpp>
#include <core/math/Vector3D.hpp>
#include <collisionstrategies/CDStrategyOpcode.hpp>
#include <core/collision/CollisionSetup.hpp>
#include <core/models/util.hpp>

using namespace rw::core::models;
using namespace rw::core::kinematics;
using namespace rw::core::math;
using namespace rw::core::collision;
using namespace rw::collisionstrategies;

SafetyComponent::SafetyComponent(const std::string& name, Device* model, WorkCell* workcell, double dt):
    RTT::TaskContext(name),
    _jointConfig("JointCurrent"),
    _jointVel("JointVelCurrent"),
    _warningPort("SafetyWarning", false),
    _errorPort("SafetyError", false),
    _errorRecovery("ErrorRecovery"),
    _model(model),
    _workcell(workcell),
    _state(workcell->getDefaultState()),
    _dt(dt),
    _jointLimits(model->getBounds())
{
    assert(dt > 0);
    ports()->addPort(&_jointConfig);
    ports()->addPort(&_jointVel);
    ports()->addPort(&_warningPort);
    ports()->addPort(&_errorPort);
    ports()->addPort(&_errorRecovery);

    _zwarning.first = 0.50;
    _zwarning.second = 3.0;

    _zerror.first = 0.4;
    _zerror.second = 3;

    _radiusWarning.first = 0.50;
    _radiusWarning.second = 1.95;

    _radiusError.first = 0.45;
    _radiusError.second = 2.0;


    _thetaWarning.first = -55*M_PI/180;
    _thetaWarning.second = 235*M_PI/180;

    _thetaError.first = -67*M_PI/180;
    _thetaError.second = 247*M_PI/180;

    _deltaq = 3*M_PI/180.0; //resolution of collision checking


    const CollisionSetup& setup = collisionSetupAccessor().get(*workcell->getWorldFrame());
    
    CDStrategy* strategy = new CDStrategyOpcode();
    
    boost::shared_ptr<CollisionDetector> detector(new CollisionDetector(workcell->getWorldFrame(),
									setup,
									strategy,
									_state));

    detector->setFirstContact(true);

    
}

SafetyComponent::~SafetyComponent() {
    
}


bool SafetyComponent::pathInCollision(const rw::core::models::Q& qgoal) {
    Q q = _jointConfig.Get();
    if (inCollision(qgoal)) {
	return true;
    }
    return inCollision(q, qgoal);
}

bool SafetyComponent::inCollision(const rw::core::models::Q& qinit, const rw::core::models::Q& qgoal) {
    if (norm_2(qinit-qgoal) > _deltaq) {
	Q q = (qinit+qgoal)/2.0;
	if (inCollision(q))
	    return true;
	else {
	    return inCollision(qinit, q) || inCollision(q, qgoal);
	}
    }
    return false;
}


bool SafetyComponent::inCollision(const Q& q) {
    _model->setQ(q, _state);
    return _detector->inCollision(_state);
}



    bool SafetyComponent::startup() {
    std::cout<<"SafetyComponent Startup"<<std::endl;
    _last = _jointConfig.Get();
    _warningPort.Set(false);
    _errorPort.Set(false);
    return true;
    }

    void SafetyComponent::update() {
    bool warning = false;
    bool error = false;

    _warningPort.Set(false);
    _errorPort.Set(false);
    return;
    Q q = _jointConfig.Get();
    if (q.size() != _model->getDOF()) {
        _warningPort.Set(true);
        _errorPort.Set(true);
    }
    _model->setQ(q, _state);

    //Get tool pose
    Transform3D<> bTe = _model->baseTend(_state);
    Vector3D<> pos = bTe.P();

    //Test for valid z
    if (pos(2) < _zwarning.first || pos(2) > _zwarning.second) {
        warning = true;
        std::cout<<"Z warning = "<<pos(2)<<std::endl;
    }
    if (pos(2) < _zerror.first || pos(2) > _zerror.second) {
        error = true;
        std::cout<<"Z error = "<<pos(2)<<std::endl;
    }

    //Test for valid radius
    double r = sqrt(pos(0)*pos(0)+pos(1)*pos(1));
    if (r < _radiusWarning.first || r > _radiusWarning.second) {
        warning = true;
        std::cout<<"Radius warning "<<r<<std::endl;
    }
    if (r < _radiusError.first || r > _radiusError.second) {
        error = true;
        std::cout<<"Radius error "<<r<<std::endl;
    }

    //Test for valid angle
    double th = atan2(pos(1), pos(0));
    if (th <= -M_PI/2)
        th = 2*M_PI+th;
    if (th < _thetaWarning.first || th > _thetaWarning.second) {
        warning = true;
        std::cout<<"Theta Warning = "<<th<<std::endl;
    }
    if (th < _thetaError.first || th > _thetaError.second) {
        error = true;
        std::cout<<"Theta Error = "<<th<<std::endl;
    }

    double EPS = 1e-1;
    //Test for valid joint limits
    for (size_t i = 0; i<q.size(); i++) {
        if (q(i)+EPS<_jointLimits.first(i) || q(i)-EPS > _jointLimits.second(i)) {
        error = true;
        std::cout<<"Joint "<<i<<" error "<<q(i)<<std::endl;
        }
    }


    if (_errorRecovery.Get()) {
        _errorPort.Set(false);
    } else
        _errorPort.Set(error);

    _warningPort.Set(warning);


    }

    void SafetyComponent::shutdown() {
    _warningPort.Set(true);
    _errorPort.Set(true);
    }


