/*********************************************************************
 * RobWork Version 0.3
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
 * for detailed Actionrmation about these packages.
 *********************************************************************/

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
