/*
 * RWBodyPool.hpp
 *
 *  Created on: Jun 3, 2009
 *      Author: jimali
 */

#ifndef RWBODYPOOL_HPP_
#define RWBODYPOOL_HPP_

#include "RWBody.hpp"

#include <stack>

namespace rwsim {
namespace simulator {

/**
 * @brief interface for creating and deleting constraintEdges and ConstraintNodes.
 * ConstraintEdges are frequently created and deleted so efficient data structures
 * are here needed.
 */
class RWBodyPool {

public:
    /**
     * @brief initialize the node and edge buffers
     */
    RWBodyPool(int nrBodies=0);

    /**
     * @brief create a ConstraintNode
     */
    RWBody *createBody(RWBody::BodyType type);

    /**
     * @brief delete a constraint node
     */
    void deleteBody(RWBody* body);

    /**
     * @brief gets the complete list of constraint nodes
     * in the pool. NULL elements can occour.
     */
    const RWBodyList& getBodies() const ;

protected:
    std::vector<RWBody*> _bodies;
    std::stack<int> _freeBodyIDs;
};

}
}

#endif /* RWBODYPOOL_HPP_ */
