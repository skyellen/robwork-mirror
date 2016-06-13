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

#include "ODEKinematicDevice.hpp"

#include "ODEUtil.hpp"
#include "ODESimulator.hpp"
#include <ode/ode.h>
#include <boost/foreach.hpp>
#include <rw/math/Math.hpp>
#include <rwsim/dynamics/KinematicDevice.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rwsim::dynamics;
using namespace rwsim::simulator;

namespace {

    bool equalSign(double v1, double v2 ){
        return (v1<0 && v2<0) || (v1>0 && v2>0);
    }

}

ODEKinematicDevice::ODEKinematicDevice(rwsim::dynamics::KinematicDevice *kdev,
                                       const rw::kinematics::State& state,
                                       ODESimulator *sim):
                                       _kdev(kdev)
{
    // ODE does not support kinematic joints.. instead all bodies are kinematic
    // and their velocity must be controlled based on the
    dSpaceID space = dHashSpaceCreate( sim->getODESpace() );
    BOOST_FOREACH(Body::Ptr kbody, kdev->getLinks() ){
        _bodies.push_back( ODEBody::makeKinematicBody(kbody, space, sim) );
        _kbodies.push_back( _bodies.back()->getBodyID() );
        sim->addODEBody( _bodies.back() );
    }
}

ODEKinematicDevice::~ODEKinematicDevice(){};

void ODEKinematicDevice::reset(rw::kinematics::State& state){
    std::vector<Body::Ptr> bodies = _kdev->getLinks();
    for(size_t i = 0; i<_kbodies.size(); i++){
        Transform3D<> wTb = rw::kinematics::Kinematics::worldTframe( bodies[i]->getBodyFrame(), state);
        wTb.P() += wTb.R()*bodies[i]->getInfo().masscenter;
        dBodyID bodyId = _kbodies[i];
        ODEUtil::setODEBodyT3D( bodyId, wTb );
        dBodyEnable( bodyId );
        dBodySetAngularVel( bodyId, 0, 0, 0 );
        dBodySetLinearVel( bodyId, 0, 0, 0 );
    }
}
namespace {

    Jacobian subJacobian(const Jacobian& jac, int toCol){
        int m = (int)jac.size1();
        RW_ASSERT(toCol<=(int)jac.size2());
        Jacobian jsub(m,toCol);
        for(int x=0;x<m;x++)
            for(int y=0;y<m;y++)
                jsub(x,y) = jac(x,y);
        return jsub;
    }

    /**
     * @brief Calculates velocity vector of the i'th joint
     * @param Jq [in] the jacobian @f$ \mathbf{J}_{\mathbf{q}} @f$
     * @param dq [in] the joint velocity vector @f$ \dot{\mathbf{q}} @f$
     * @return the velocity vector @f$ \mathbf{\nu} @f$
     * @relates Jacobian
     */
    VelocityScrew6D<> mult(const Jacobian& Jq, const Q& dq, int column)
    {
        using namespace boost::numeric::ublas;
        int m = (int)Jq.size1();
        RW_ASSERT(column<(int)Jq.size2());

        matrix<double> subjac(m,column+1);
        matrix_range<matrix<double> > mr (subjac, range (0, m), range (0, column+1));
        for (unsigned i = 0; i < mr.size1 (); ++ i)
            for (unsigned j = 0; j < mr.size2 (); ++ j)
                mr (i, j) = Jq(i,j);

        vector<double> subdq(column+1);
        vector_range<vector<double> > vr (subdq, range(0, column+1));
        for (unsigned i = 0; i < vr.size (); ++ i)
            vr (i) = dq(i);

        return VelocityScrew6D<>(prod(subjac, subdq));
    }


}

void ODEKinematicDevice::update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state){
    double dt = info.dt;
	// Update velocity of the ode kinematic bodies
    std::cout << "get velocity!" << std::endl;
    Q velQ = _kdev->getVelocity(state);
    std::cout << "= "<< velQ << std::endl;
    std::cout << _kdev->getLinks().size()<< "==" << velQ.size() << std::endl;
    std::cout << _kdev->getLinks().size()<< "==" <<_kbodies.size() << std::endl;

    RW_ASSERT(_kdev->getLinks().size()==velQ.size());
    RW_ASSERT(_kdev->getLinks().size()==_kbodies.size());
    std::cout << "KinematicDevice update.." << std::endl;

    std::cout << "Update state!" << std::endl;
    Q q1 = _kdev->getModel().getQ(state)+velQ*dt;
    std::cout << "-get bounds!" << std::endl;
	Device::QBox bounds = _kdev->getModel().getBounds();
	q1 = Math::clampQ(q1, bounds.first, bounds.second);
	std::cout << "-set" << std::endl;

	Jacobian djac = _kdev->getModel().baseJend(state);
	//const Frame *baseFrame = _kdev->getModel().getBase();
	//Transform3D<> wTb = Kinematics::worldTframe(baseFrame,state);

	_kdev->getModel().setQ(q1, state);

	std::cout << "Update vel of ode bodies" << std::endl;
	//int qi = 0;
	for(size_t i = 0; i<_kbodies.size(); i++){
	    std::cout << "- Body " << i << std::endl;
/*
	    dBodyID bodyId = _kbodies[i];
	    KinematicBody *kbody = _kdev->getBodies()[i];
	    Vector3D<> centermass = kbody->getInfo().masscenter;

	    // first we get the velocity referenced from base frame
	    VelocityScrew6D<> jvel = mult(djac,velQ,i);
	    // then we change the velocity reference point
	    //jvel = centermass*jvel;
	    // and then we change the reference frame to the world frame
	    jvel = wTb.R() * jvel;

	    // and then we update the velocities of the bodies

	    dBodySetLinearVel( bodyId, jvel(0), jvel(1), jvel(2) );
	    dBodySetAngularVel( bodyId, jvel(3), jvel(4), jvel(5) );
*/


	    Body::Ptr kbody = _kdev->getLinks()[i];
        // we calculate the difference between the current pose of the joint
	    // and the destination of the joint
	    Transform3D<> wTj = ODEUtil::getODEBodyT3D(_kbodies[i]);
	    Vector3D<> offset = kbody->getInfo().masscenter;
	    wTj.P() -= wTj.R()*offset;

	    Transform3D<> wTjd = rw::kinematics::Kinematics::worldTframe( kbody->getBodyFrame(), state);
	    // we now create the velocity screw and express it in world frame
	    VelocityScrew6D<> jXjd( inverse(wTj)*wTjd );
	    VelocityScrew6D<> w_jXjd = wTj.R()*jXjd* (1.0/dt);

        dBodyID bodyId = _kbodies[i];
        dBodySetLinearVel( bodyId, w_jXjd(0), w_jXjd(1), w_jXjd(2) );
        dBodySetAngularVel( bodyId, w_jXjd(3), w_jXjd(4), w_jXjd(5) );

    }
}

void ODEKinematicDevice::postUpdate(rw::kinematics::State& state){
    //_kdev->getModel().setQ(q, state);
}
