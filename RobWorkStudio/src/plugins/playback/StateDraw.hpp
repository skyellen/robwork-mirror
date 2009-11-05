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


#ifndef RWS_QDRAW_HPP
#define RWS_QDRAW_HPP

#include <rw/kinematics/State.hpp>
#include <rw/use_robwork_namespace.hpp>
#include <rw/common/Ptr.hpp>

class StateDraw;
typedef rw::common::Ptr<StateDraw> StateDrawPtr;

/**
   @brief Interface for the drawing of the work cell for a given state.
*/
class StateDraw
{
public:
    /**
       @brief Draw the work cell for a state of \b state.
    */
    virtual void draw(const robwork::State& state) const = 0;

    /**
       @brief Destructor.
    */
    virtual ~StateDraw() {}

protected:
    /**
       @brief Default constructor.
    */
    StateDraw() {}

private:
    StateDraw(const StateDraw&);
    StateDraw& operator=(const StateDraw&);
};

/**
   @brief A state drawer that does not draw anything.

   Calls to the draw() method of the drawer are simply ignored.
*/
StateDrawPtr makeEmptyStateDraw();

#endif
