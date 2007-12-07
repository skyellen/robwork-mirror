/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "ParallelIKSolver.hpp"

#include <vector>

#include <boost/numeric/ublas/vector.hpp>

#include <rw/math/Jacobian.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/Joint.hpp>

#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>

#include <rw/common/Property.hpp>
#include <rw/common/macros.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/LinearAlgebra.hpp>

using namespace rw;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;

using namespace boost::numeric;

using namespace rw::iksolvers;

ParallelIKSolver::ParallelIKSolver(const models::ParallelDevice* device) :
    _device(device)
{}

ParallelIKSolver::~ParallelIKSolver()
{}

std::vector<Q> ParallelIKSolver::solve(
    const Transform3D<>& dest,
    const State& wstate) const
{
    State state = wstate;
    const double maxerror = getMaxError();
    const unsigned int maxiterations = getMaxIterations();

    std::vector< ParallelLeg* > legs = _device->getLegs();

    // initialize currQ
    Q currQ(Q::ZeroBase(legs.size() * 6));

    size_t columns=0, qIndex=0;
    std::vector<ParallelLeg*>::iterator leg_iter = legs.begin();
    for(qIndex=0;leg_iter!=legs.end();++leg_iter){
        std::vector<Frame*> sChain = (*leg_iter)->getKinematicChain();
        for(size_t j=0;j<sChain.size();j++){
            Joint *joint = dynamic_cast<Joint*>(sChain[j]);
            if( joint==NULL )
                continue;
            currQ[qIndex] = *joint->getQ(state);
            qIndex++;
        }
        columns += (*leg_iter)->nrOfJoints();
    }

    // initialize deltaQ
    Q deltaQ(Q::ZeroBase(legs.size()*6));

    // initialize deltaX vector
    Q deltaX(Q::ZeroBase(legs.size()*6));

    // calculate the difference from current pose to dest pose
    Transform3D<double> curr = legs.front()->baseTend(state);
    Vector3D<> pos = dest.P() - curr.P();
    EAA<> orin = curr.R()*(EAA<>( inverse(curr.R())*dest.R() ) );

    // init deltaX
    size_t index = (legs.size()-1)*6;
    deltaX[index+0] = pos[0];
    deltaX[index+1] = pos[1];
    deltaX[index+2] = pos[2];
    deltaX[index+3] = orin[0];
    deltaX[index+4] = orin[1];
    deltaX[index+5] = orin[2];

    // calculate the jacobian
    Jacobian jacobian = _device->baseJend(state);

    size_t iterations=0;
    double error=1, last_error=1;
    if (jacobian.size1() == jacobian.size2()) {
        // The jacobian has full rank, use Newton-Raphson to solve for deltaQ
        do {
            // calculate deltaQ
            deltaQ = Q(prod(LinearAlgebra::PseudoInverse(jacobian.m()) , deltaX.m()));

            // update the state of the joints with deltaQ
            qIndex = 0;
            for(leg_iter = legs.begin();leg_iter!=legs.end();++leg_iter){
                std::vector<Frame*> sChain = (*leg_iter)->getKinematicChain();
                for(size_t j=0;j<sChain.size();j++){
                    Joint *joint = dynamic_cast<Joint*>(sChain[j]);
                    if( joint==NULL )
                        continue;
                    currQ(qIndex) += deltaQ(qIndex);
                    joint->setQ(state, &currQ(qIndex));
                    qIndex++;
                }
            }

            // update deltaX
            for(size_t i=0; i<legs.size()-1; i++){
                // Calculate quaternion and pos for i
                Transform3D<double> bTe = legs[i]->baseTend(state);
                // Calculate quaternion and pos for i+1
                Transform3D<double> bTe_1 = legs[i+1]->baseTend(state);
                // Calculate the difference between pose1 and pose2
                Vector3D<double> pos = bTe_1.P() - bTe.P();
                EAA<double> orin = bTe.R()*(EAA<>( inverse(bTe.R())*bTe_1.R() ) );
                // copy it into deltaX
                int index = i*6;
                deltaX[index+0] = pos(0);
                deltaX[index+1] = pos(1);
                deltaX[index+2] = pos(2);
                deltaX[index+3] = orin(0);
                deltaX[index+4] = orin(1);
                deltaX[index+5] = orin(2);
            }

            Transform3D<double> curr = legs.front()->baseTend(state);
            Vector3D<> pos = dest.P() - curr.P();
            EAA<> orin =
                curr.R() *
                EAA<>(inverse(curr.R()) * dest.R());

            // init deltaX
            size_t index = (legs.size()-1)*6;
            deltaX[index+0] = pos[0];
            deltaX[index+1] = pos[1];
            deltaX[index+2] = pos[2];
            deltaX[index+3] = orin[0];
            deltaX[index+4] = orin[1];
            deltaX[index+5] = orin[2];

            // recalculate the jacobian
            jacobian = _device->baseJend(state);

            error = deltaX.norm2();
            last_error = error;
            iterations++;
        } while( maxerror<error && iterations<maxiterations);
    } else if( jacobian.size1() > jacobian.size2() ){
        // The system is overdetermined, use Moore-Penrose approach to solve for
        // deltaQ
        do {
            // calculate deltaQ
            ublas::matrix<double> leftside =
                LinearAlgebra::PseudoInverse(
                    prod(trans(jacobian.m()), jacobian.m()));

            const Q::Base& rightside =
                prod(trans(jacobian.m()), deltaX.m());

            deltaQ = Q(prod(leftside , rightside));
            // update the state of the joints with deltaQ
            qIndex = 0;
            for(leg_iter = legs.begin();leg_iter!=legs.end();++leg_iter){
                std::vector<Frame*> sChain = (*leg_iter)->getKinematicChain();
                for(size_t j=0;j<sChain.size();j++){
                    Joint *joint = dynamic_cast<Joint*>(sChain[j]);
                    if( joint==NULL )
                        continue;
                    currQ(qIndex) += deltaQ(qIndex);
                    joint->setQ(state, &currQ(qIndex));
                    qIndex++;
                }
            }

            // update deltaX
            for(size_t i=0; i<legs.size()-1; i++){
                // Calculate quaternion and pos for i
                Transform3D<double> bTe = legs[i]->baseTend(state);
                // Calculate quaternion and pos for i+1
                Transform3D<double> bTe_1 = legs[i+1]->baseTend(state);
                // Calculate the difference between pose1 and pose2
                Vector3D<double> pos = bTe_1.P() - bTe.P();
                EAA<double> orin = bTe.R()*(EAA<>( inverse(bTe.R())*bTe_1.R() ) );
                // copy it into deltaX
                int index = i*6;
                deltaX[index+0] = pos(0);
                deltaX[index+1] = pos(1);
                deltaX[index+2] = pos(2);
                deltaX[index+3] = orin(0);
                deltaX[index+4] = orin(1);
                deltaX[index+5] = orin(2);
            }

            Transform3D<double> curr = legs.front()->baseTend(state);
            Vector3D<> pos = dest.P() - curr.P();
            EAA<> orin = curr.R()*(EAA<>( inverse(curr.R())*dest.R() ) );

            // init deltaX
            size_t index = (legs.size()-1)*6;
            deltaX[index+0] = pos[0];
            deltaX[index+1] = pos[1];
            deltaX[index+2] = pos[2];
            deltaX[index+3] = orin[0];
            deltaX[index+4] = orin[1];
            deltaX[index+5] = orin[2];

            // recalculate the jacobian
            jacobian = _device->baseJend(state);

            error = deltaX.norm2();
            last_error = error;
            iterations++;
        } while( maxerror<error && iterations<maxiterations );

    } else {
        RW_ASSERT(0);
    }

    Q q = _device->getQ(state);
    std::vector<Q> sol;
    sol.push_back(q);
    return sol;
}
