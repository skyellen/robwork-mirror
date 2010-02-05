#ifndef CONSTRAINTSOLVER_HPP_
#define CONSTRAINTSOLVER_HPP_

#include "ConstraintEdge.hpp"
#include <vector>

#include <rw/kinematics/State.hpp>

typedef std::vector<dynamics::ConstraintEdge*> CEdgeGroup;

struct SolverInfo {
public:
    SolverInfo(double timestep):
        dt(timestep){}
    double dt;
};

/**
 * @brief Abstraction over a constraint solver for dynamics simulations.
 *
 * The constraint solver changes forces, velocities and/or position to correct
 * constraints between objects in a scene. Most commonly contact constraints
 * and especially multiple coupled contact constraints are handled.
 *
 */
class ConstraintSolver {

protected:
    ConstraintSolver(){};

public:

    virtual ~ConstraintSolver(){};

    /**
     * @brief solves the constraints forces of multiple groups of constraints
     * @return
     */
    virtual bool solve( std::vector<CEdgeGroup>& groups,
                        SolverInfo& info,
                        rw::kinematics::State& state);

    /**
     * @brief solves the constraints forces of a group of constraints
     * @return
     */
    virtual bool solveGroup( CEdgeGroup& group,
                             SolverInfo& info,
                             rw::kinematics::State& state) = 0;

};


#endif /*CONSTRAINTSOLVER_HPP_*/
