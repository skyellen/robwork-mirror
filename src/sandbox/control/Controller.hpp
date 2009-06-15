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
 * for detailed information about these packages.
 *********************************************************************/

#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

class Controller {

public:

    /**
     * @brief updates/steps the controller
     */
    //virtual void update(double dt, rw::kinematics::State& state) = 0;

    /**
     * @brief reset the controller to the applied state
     * @param state
     */
    //virtual void reset(const rw::kinematics::State& state) = 0;


};



#endif /*CONTROLLER_HPP_*/
