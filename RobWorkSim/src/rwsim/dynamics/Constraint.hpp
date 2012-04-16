

namespace rwsim {
namespace dynamics {

/**
 * @brief a constraint is a mathematical model that constrain the movement
 * between two bodies in a dynamic simulation.
 *
 * The constraint may involve
 *
 * types: revolute, prismatic, fixed, contact
 */
class Constraint: public rw::kinematics::StateData {
public:
    typedef enum {Revolute, Prismatic, Fixed, Contact} ConstraintType;

    Constraint(ConstraintType type, Body* b1, Body* b2);

    Constraint(rw::models::RevoluteJoint* joint);

    Constraint(rw::models::PrismaticJoint* joint);

    size_t getDOF();

    void setQ(const rw::math::Q& q, State& state);

    rw::math::Q getQ(State& state);

};


}
}
