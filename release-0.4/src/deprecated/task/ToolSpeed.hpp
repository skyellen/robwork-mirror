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


#ifndef RW_TASK_TOOLSPEED_HPP
#define RW_TASK_TOOLSPEED_HPP

/**
   @file ToolSpeed.hpp
*/

namespace rw { namespace task {

    /** @addtogroup task */
    /*@{*/

    /**
       @brief ToolSpeed represents a speed for the tool of a device.

       A tool speed can be of type either Angular or Positional.

       Tool speeds of type Angular give a speed for the rotation of the frame of
       the tool.

       Tool speeds of type Positional give a speed for the position of frame of
       the tool.
    */
    class ToolSpeed
    {
    public:
        //! Types of tool speed specifications.
        enum SpeedType { Angular, Positional };

        /**
           Constructor
        */
        ToolSpeed(SpeedType speed_type, double tool_speed) :
            _speed_type(speed_type),
            _tool_speed(tool_speed)
        {}

        /**
           The speed of the tool.
        */
        double getToolSpeed() { return _tool_speed; }

        /**
           The type of the tool speed.
        */
        SpeedType getSpeedType() { return _speed_type; }

    private:
        SpeedType _speed_type;
        double _tool_speed;
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
