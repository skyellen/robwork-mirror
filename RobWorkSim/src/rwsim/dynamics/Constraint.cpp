#include "Constraint.hpp"

using namespace rwsim::dynamics;

Constraint::Constraint(ConstraintType type, Body* b1, Body* b2){

}

Constraint::Constraint(rw::models::RevoluteJoint* joint){

}

Constraint::Constraint(rw::models::PrismaticJoint* joint){

}

size_t Constraint::getDOF(){

}

void Constraint::setQ(const rw::math::Q& q, State& state){

}

rw::math::Q Constraint::getQ(State& state){

}
