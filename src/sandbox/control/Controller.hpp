#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

class Controller {

public:

    /**
     * @brief updates/steps the controller
     */
    virtual void update(double dt, rw::kinematics::State& state) = 0;

    /**
     * @brief reset the controller to the applied state
     * @param state
     */
    virtual void reset(const rw::kinematics::State& state) = 0;

};



#endif /*CONTROLLER_HPP_*/
