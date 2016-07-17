
#ifndef SCHEMA_HPP_
#define SCHEMA_HPP_

#include <rw/common/Ptr.hpp>

#include "BrainState.hpp"
#include "MotorProgram.hpp"

namespace rw { namespace common { class PropertyMap; } }

class Predictor {
public:
    typedef rw::common::Ptr<Predictor> Ptr;

    virtual BrainState makePrediction(const BrainState& state) = 0; // pure-function

};

class Condition {
public:
    typedef rw::common::Ptr<Condition> Ptr;

    virtual bool isConditionMet(const BrainState& state) = 0;
};

class BrainStateCondition: public Condition {
public:

    BrainStateCondition(const BrainState& goal): _goal(goal){}

    bool isConditionMet(const BrainState& state){
        return _goal.equal( state );
    }

    BrainState _goal;
};


/**
 * @brief this class handles the execution of lua commands such that they
 * are executed in a seperate thread.
 */
class Schema
{
public:
    typedef rw::common::Ptr<Schema> Ptr;

    Schema(const std::string& name,
           Condition::Ptr precondition,
           Condition::Ptr postcondition,
           Predictor::Ptr predictor,
           MotorProgram::Ptr mp) :
            _name(name)

    {
    }

    void setPreCondition( Condition::Ptr pre);
    Condition::Ptr getPreCondition( );
    bool isPreConditionMet(const BrainState& bstate ){ return _precondition->isConditionMet(bstate); }

    void setPostCondition( Condition::Ptr goalcondition);
    Condition::Ptr getPostCondition( );
    bool isPostConditionMet(const BrainState& bstate ){ return _postcondition->isConditionMet(bstate); }

    Predictor::Ptr getPredictor();
    void setPredictor( Predictor::Ptr predictor );
    BrainState makePrediction( const BrainState& bstate ){ return _predictor->makePrediction(bstate); };

    MotorProgram::Ptr getMotorProgram();
    void setMotorProgram( MotorProgram::Ptr mp);
    void executeMotorProgram( rw::common::Ptr<rw::common::PropertyMap> parameters, const BrainState& bstate  );



private:
    std::string _name;
    //rwsim::simulator::DynamicSimulator::Ptr _dsim;
    //rwsim::simulator::ThreadSimulator::Ptr _sim;
    //rwsim::dynamics::DynamicWorkCell::Ptr _dwc;
    Condition::Ptr _precondition;
    Condition::Ptr _postcondition;
    Predictor::Ptr _predictor;
    MotorProgram::Ptr _mp;

};


#endif /* LUAEDITORWINDOW_HPP_ */
