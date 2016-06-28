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


#include "ParallelDevice.hpp"
#include "ParallelLeg.hpp"
#include "Joint.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Jacobian.hpp>

using namespace rw::models;
using namespace rw::math;
using namespace rw::kinematics;

using boost::numeric::ublas::slice;
using boost::numeric::ublas::range;

typedef boost::numeric::ublas::matrix_vector_slice<Jacobian::BoostBase> MatrixSlice;
typedef boost::numeric::ublas::matrix_range<Jacobian::BoostBase> MatrixRange;

namespace
{
    Jacobian::BoostBase* makeZeroMatrix(int rows, int cols)
    {
		return new Jacobian::BoostBase(Jacobian::BoostZeroBase(rows, cols));
    }

    void append(std::vector<Joint*>& result, const std::vector<Joint*>& tail)
    {
        result.insert(result.end(), tail.begin(), tail.end());
    }

    std::vector<Joint*> getActuatedJoints(const std::vector<ParallelLeg*>& legs)
    {
        std::vector<Joint*> result;
        for (size_t i = 0; i < legs.size(); i++)
            append(result, legs[i]->getActuatedJoints());
        return result;
    }

    std::vector<Joint*> getUnactuatedJoints(const std::vector<ParallelLeg*>& legs)
    {
        std::vector<Joint*> result;
        for (size_t i = 0; i < legs.size(); i++)
            append(result, legs[i]->getUnactuatedJoints());
        return result;
    }
}

ParallelDevice::ParallelDevice(
    const std::vector<ParallelLeg*>& legs,
    const std::string name,
    const State& state)
    :
    JointDevice(
        name,
        legs.front()->getBase(),
        legs.front()->getEnd(),
        getActuatedJoints(legs),
        state),

    _legs(legs),
    _actuatedJoints(getActuatedJoints(legs)),
    _unActuatedJoints(getUnactuatedJoints(legs))
{
    // create the actuated joint jacobian, should be the sice of row = 6* number of legs
    // and columns = number of active joints
    _aJointJ = makeZeroMatrix(6*((int)legs.size()-1), (int)_actuatedJoints.size()) ;
    // and the unactuated joint matrix
    _uaJointJ = makeZeroMatrix(6*((int)legs.size()-1), (int)_unActuatedJoints.size());

    // construct the current joint value vectors for actuated and unactuated joints
    _lastAJVal = new Q(Q::zero((int)_actuatedJoints.size()) );
    _lastUAJVal = new Q(Q::zero((int)_unActuatedJoints.size()) );

    for (size_t i = 0; i < _actuatedJoints.size(); i++)
        (*_lastAJVal)(i) = *_actuatedJoints[i]->getData(state);

    for (size_t i = 0;i < _unActuatedJoints.size(); i++)
        (*_lastUAJVal)(i) = *_unActuatedJoints[i]->getData(state);
}

ParallelDevice::~ParallelDevice()
{
    delete _aJointJ;
    delete _uaJointJ;
    delete _lastAJVal;
    delete _lastUAJVal;
}

void ParallelDevice::setQ(const Q& q, State& s) const
{
    // default MAX_ITERATIONS.
    const int MAX_ITERATIONS = 20;

    State state(s);

    size_t i;
    for (i=0; i < _actuatedJoints.size(); i++) {
        _actuatedJoints[i]->setData(state, &q[i]);
    }
    // initialize configuration vector
    int row=0,ja_column=0,jua_column=0;
    for(i=0;i<_legs.size();i++){ // only run through L-1 legs where L is nr of legs
        Jacobian leg_jacobian = _legs[i]->baseJend(state);
        std::vector<Frame*>::const_iterator iter = _legs[i]->getKinematicChain().begin();
        for(size_t j=0; iter!=_legs[i]->getKinematicChain().end(); ++iter ){ // the columns
            Joint *joint = dynamic_cast<Joint*>(*iter);
            if( joint == NULL )
                continue;
            // determine if joint j is an active joint
            if( joint->isActive() ){
                // copy the leg_jacobian column at index j into the actuated jacobian matrix
                if(i!=_legs.size()-1){
                    MatrixSlice(
                        *_aJointJ,
                        slice(row, 1, 6),
                        slice(ja_column,0,6)) = column(leg_jacobian.m(), j);
                }
                if(i!=0){
                    MatrixSlice(
                        *_aJointJ,
                        slice(row-6, 1, 6),
                        slice(ja_column, 0, 6)) = -column(leg_jacobian.m(), j);
                }
                ja_column++;
            } else {
                // copy the jacobian row into the unactuated jacobian matrix
                if(i!=_legs.size()-1){
                    MatrixSlice(
                        *_uaJointJ,
                        slice(row, 1, 6),
                        slice(jua_column, 0, 6)) = column(leg_jacobian.m(), j);
                }
                if (i != 0) {
                    MatrixSlice(
                        *_uaJointJ,
                        slice(row - 6, 1, 6),
                        slice(jua_column,0,6)) = - column(leg_jacobian.m(), j);
                }
                jua_column++;
            }
            j++;
        }
        row += 6;
    }

    double e = 1e-6;
    Q deltaQA(Q::zero((int) _actuatedJoints.size() ));
    for(i=0;i<_actuatedJoints.size();i++){
        deltaQA(i) = q(i) - (*_lastAJVal)(i);
    }

    // calculate -aJ(q)*dQa , aJ is te actuated jacobian and dQa is difference between
    // current and desired actuated joint val
    Q aJdeltaQA(
        - prod(*_aJointJ, deltaQA.m()));

    // Calculate the initial change of the unactuated joints
    Q deltaQUA(
        prod(LinearAlgebra::pseudoInverse(*_uaJointJ), aJdeltaQA.m()));

    // Solve the equation uaJ(q)*dQua = dY , where dY = deltaPoses, for dQua
    Q deltaY(Q::zero((int)_unActuatedJoints.size()));

    int iterations = 0;
    double error = 1;
    //double lasterror = 1;
    do{
        // update the unactuated joints and unactuated joint jacobian
        for(i=0; i<_unActuatedJoints.size(); i++){
            (*_lastUAJVal)(i) = (*_lastUAJVal)(i) + deltaQUA[i];
            _unActuatedJoints[i]->setData(state, &(*_lastUAJVal)(i));
        }

        row = 0;jua_column=0;
        //update Unactuated Joint Jacobian
        for(i=0;i<_legs.size();i++){
            Jacobian leg_jacobian = _legs[i]->baseJend(state);
            std::vector<Frame*>::const_iterator iter = _legs[i]->getKinematicChain().begin();
            for(size_t j=0; iter!=_legs[i]->getKinematicChain().end(); ++iter ){
                Joint *joint = dynamic_cast<Joint*>(*iter);
                if( joint == NULL )
                    continue;
                if( !joint->isActive() ){
                    // copy the jacobian row into the unactuated jacobian matrix
                    if(i!=_legs.size()-1){
                        MatrixSlice(
                            *_uaJointJ,
                            slice(row,1,6),
                            slice(jua_column,0,6)) = column(leg_jacobian.m(), j);
                    }
                    if(i!=0){
                        MatrixSlice(
                            *_uaJointJ,
                            slice(row-6,1,6),
                            slice(jua_column,0,6)) = -column(leg_jacobian.m(), j);
                    }
                    jua_column++;
                }
                j++;
            }
            row += 6;
        }

        // update deltaY
        for(i=0; i<_legs.size()-1; i++){
            // Calculate quaternion and pos for i
            Transform3D<> bTe = _legs[i]->baseTend(state);
            // Calculate quaternion and pos for i+1
            Transform3D<> bTe_1 = _legs[i+1]->baseTend(state);
            // Calculate the difference between pose1 and pose2
            Vector3D<> pos = bTe_1.P() - bTe.P();
            EAA<> orin = bTe.R()*(EAA<>( inverse(bTe.R())*bTe_1.R() ) );
            // copy it into deltaY

            int index = (int)i*6;
            deltaY[index+0] = pos(0);
            deltaY[index+1] = pos(1);
            deltaY[index+2] = pos(2);

            deltaY[index+3] = orin(0);
            deltaY[index+4] = orin(1);
            deltaY[index+5] = orin(2);
        }

        // Calculate the new change in the unactuated joints
        error = deltaY.norm2();
        //lasterror = error;
        deltaQUA = Q(prod(LinearAlgebra::pseudoInverse(*_uaJointJ), deltaY.m()));
        iterations++;
    } while( e < error && iterations < MAX_ITERATIONS);

    if (iterations >= MAX_ITERATIONS){
        // TODO cast exception and return parallelDevice to initial state
        std::cout << "ERROR: Max iterations exeeded!!!" << std::endl;
    } else {
       s = state;
    }

    for(size_t i=0;i<_actuatedJoints.size();i++)
        (*_lastAJVal)(i) = *_actuatedJoints[i]->getData(s);

    for(size_t i=0;i<_unActuatedJoints.size();i++)
        (*_lastUAJVal)(i) = *_unActuatedJoints[i]->getData(s);
}

// Jacobians

Jacobian ParallelDevice::baseJend(const State& state) const
{
    // calculate the size of the total jacobian matrix and configuration vector
    size_t rows=0, columns=0;
    std::vector< ParallelLeg* >::const_iterator leg_iter = _legs.begin();
    for(;leg_iter!=_legs.end();++leg_iter){
        rows += 6;
        columns += (*leg_iter)->nrOfJoints();
    }


    Jacobian::BoostBase m = Jacobian::BoostZeroBase(rows, columns);

    // initialize configuration vector
    int row=0,column=0;
    for(size_t i=0;i<_legs.size();i++){
        // copy the base to endeffector jacobian of leg i to the jacobian matrix at index (row,column)

        MatrixRange(
            m,
            range(row, row+6),
            range(column, column+_legs[i]->nrOfJoints())) = _legs[i]->baseJend(state).m();

        if(i!=0){ // for all legs except the first copy -bTe jacobian of leg into (row-6,column)
            MatrixRange(
                m,
                range(row-6, row),
                range(column, column + _legs[i]->nrOfJoints())) =
                -_legs[i]->baseJend(state).m();
        }

        // update the row and column count
        row += 6; // allways for robots in 3d
        column += (int)_legs[i]->nrOfJoints();
    }

    return Jacobian( m );
}

Jacobian ParallelDevice::baseJframe(const Frame* frame, const State& state) const
{
    RW_THROW("Not implemented");

    // calculate the size of the total jacobian matrix and configuration vector
    size_t rows = 0, columns = 0;
    std::vector<ParallelLeg*>::const_iterator leg_iter = _legs.begin();
    for(;leg_iter!=_legs.end();++leg_iter){
        rows += 6;
        columns += (*leg_iter)->nrOfJoints();
    }

    Jacobian jacobian(Jacobian::BoostZeroBase(rows, columns));
    return jacobian;
}
