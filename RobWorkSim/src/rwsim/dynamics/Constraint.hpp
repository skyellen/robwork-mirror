#include "Body.hpp"

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

        typedef rw::common::Ptr<Constraint> Ptr;

        size_t getDOF();

        void setQ(const rw::math::Q& q, State& state);

        rw::math::Q getQ(State& state);

        static Constraint::Ptr makeFixedConstraint();

    protected:
        Constraint(ConstraintType type, Body* b1, Body* b2);



    };


}
}
