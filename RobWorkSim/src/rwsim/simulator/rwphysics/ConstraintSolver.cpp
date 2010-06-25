
#include "ConstraintSolver.hpp"

using namespace rwsim::simulator;

bool ConstraintSolver::solve( std::vector<CEdgeGroup>& groups,
            SolverInfo& info,
            rw::kinematics::State& state) {

    for(int i=0;i<groups.size();i++){
        if( !solveGroup(groups[i], info, state) )
            return false;
    }
    return true;
}
