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

#include "Brain.hpp"

#include <rwsim/dynamics/Body.hpp>

#include <boost/foreach.hpp>

#include "Abstraction.hpp"
#include "Memory.hpp"

using rw::common::PropertyMap;
using rw::math::Transform3D;
using rwsim::dynamics::Body;

void Brain::run()
{
     _stop = false;
     _stopped = false;

     while( _stop==false ){
         /// The main brain loop, play and plan

         // -2. get sensor state
         BrainState currentstate = computeSensorState();

         // -1. extend state with the states of abstractions
         BOOST_FOREACH(Abstraction::Ptr abstraction, _abstractions){
             abstraction->update( currentstate, _memory);
         }

         // 1. look at current state

         // 2. what can i do in this state, match it with preconditions of schemas
         // 2b. does 2 iteratively (n-times) calculating predicted states based on actions in 2.
         // 3. now theaction tree is build....
         // 4. look at the tree and choose the wanted path/outcome.

         // 5. choose action, eg execute schema



         //memorystack.addState( currentstate );

         // if we choose a new action the save the state in
         _memory.addState( currentstate );


     }
     _stopped = true;
 }

BrainState Brain::computeSensorState(){
    BrainState state(_rwstate);

    PropertyMap objMap;
    Body::Ptr b;
    Transform3D<> t3d = b->getTransformW( _rwstate );
    objMap.set("name", b->getName() );
    objMap.set("transform", t3d);

    state.getMap().set("obj"+b->getName(), objMap);
    return state;
}
