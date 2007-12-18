#include "VirtualDevice.hpp"

#include <core/math/Transform3D.hpp>

#include <sstream>

using namespace rw::core::models;
using namespace rw::core::math;

VirtualDevice::VirtualDevice(const std::string& name,
                             WorkCell* workcell,
                             Device* model,
                             double dt,
                             double updateRate):
    RTT::TaskContext(name),
    _qPathIn("JointPathTarget"),
    _qOut("JointCurrent"),
    _qdotOut("JointVelCurrent"),
    _errorPort("SafetyError", true),
    _state(workcell->getDefaultState()),
    _dt(dt),
    _updateRate(updateRate)
 {
     assert(model != NULL);
     _device = model;
     ports()->addPort(&_qPathIn);
     ports()->addPort(&_qOut);
     ports()->addPort(&_qdotOut);
     ports()->addPort(&_errorPort);

}


VirtualDevice::~VirtualDevice() {

}

bool VirtualDevice::startup() {
    std::cout<<"VirtualDevice::startup"<<std::endl;
    _qOut.Set(_device->getQ(_state));
    _qdotOut.Set(boost::numeric::ublas::zero_vector<double>(_device->getDOF()));
    _qdotlast = boost::numeric::ublas::zero_vector<double>(_device->getDOF());
    _acclimits = _device->getAccelerationLimits();
    std::cout<<"VirtualDevice::startup finished"<<std::endl;
    _log.open("VirtualDevice.log");
    return true;
}

void VirtualDevice::update() {
    if (_errorPort.Get()) {
        std::cout<<"VirtualDevice recieved Error Message"<<std::endl;
        //To what is takes to stop the robot
    }

    Q q = _device->getQ(_state);
    
    std::vector<Q> path = _qPathIn.Get();

    if(_lastPath.size()!=0){
        if( (norm_inf(_lastPath[0]-q) > norm_inf(_currStepSize)*2/3) || path.size()==0 ){
            path = _lastPath;
        } else if( path.size()!=0) {
            _currStepSize = path[0]-q;
            std::cout << "NEW PATH" << std::endl;
        } else {
            path = _lastPath;
        }
    } else {
        if(path.size()!=0)
            _currStepSize = path[0]-q;
    }

    if(path.size() == 0) {
        //std::cout << "EMPTY PATH" << std::endl;
        return;
    }

    if (path[0].size() != _device->getDOF()) {
        std::ostringstream msg;
        msg<<__LINE__<<":"<<__FILE__<<" Length of configurations does not match";
        std::cout<<msg<<std::endl;
        assert(true);
        //	throw msg.str();
    }

    Q dq = _currStepSize*2; // calculate the velocity
    //std::cout << "Step size" << _currStepSize << std::endl;
    // which is bounded by some upper velocity, but we don't handle that yet TODO
    for (size_t i = 0; i<q.size(); i++){
        if( dq(i)>M_PI/4) dq(i)=M_PI/4;
        else if( dq(i)<-M_PI/4) dq(i)=-M_PI/4;
    }

    // calculate the next pos
    q = q+dq*_updateRate;
    // remember to bound it
    for(size_t j=0; j<q.size();j++){
        if( (dq(j)>=0) && (q(j)>path[0](j)) ) q(j) = path[0](j);
        else if ( (dq(j)<0) && (q(j)< path[0](j)) ) q(j) = path[0](j);
    }

//    q = path[0];
    _device->setQ(q, _state);
    _qOut.Set(q);
    _qdotlast = dq;
    _qdotOut.Set( _qdotlast );
    _lastPath = path;

//    std::cout << "BUM" << std::endl;
//    Transform3D<> toolpos = _device->baseTend(_state);
//    std::cout << toolpos << std::endl;
//    _log<<toolpos.P()(0)<<" "<<toolpos.P()(1)<<" "<<toolpos.P()(2)<<" ";
//    for (size_t i = 0; i<q.size(); i++)
//        _log << q(i) <<" ";
//    _log<<std::endl;
}


void VirtualDevice::shutdown() {
    _log.close();
}


Q VirtualDevice::getQ() {
    return _device->getQ(_state);
}
