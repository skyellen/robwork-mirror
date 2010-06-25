/*
 * CollisionChecker.hpp
 *
 *  Created on: 06/04/2010
 *      Author: jimali
 */

#ifndef COLLISIONCHECKER_HPP_
#define COLLISIONCHECKER_HPP_

namespace rw {
namespace proximity {

class CollisionChecker {

	typedef enum {
    	AllContactsFullInfo, //! find all collisions and return full collision information
    	AllContactsNoInfo, //! find all collisions but without collision information
    	FirstContactFullInfo,//! return on first contact and include full collision information
    	FirstContactNoInfo //! return on first collision but without collision information
	} CollisionQueryType;


	void setCollisionQueryType(CollisionQueryType type);


    /**
     @brief Check the workcell for collisions.

     @param state [in] The state for which to check for collisions.

     @param result [out] If non-NULL, the pairs of colliding frames are
     inserted in \b result.

     @param stopAtFirstContact [in] If \b result is non-NULL and \b
     stopAtFirstContact is true, then only the first colliding pair is
     inserted in \b result. By default all colliding pairs are inserted.

     @return true if a collision is detected; false otherwise.
     */
    bool inCollision(const kinematics::State& state,
                     CollisionResult* result = 0) const;

    /**
     * @brief adds collision model describing the geometry \b geom. The collision
     * model is associated to the frame.
     */
    void addModel(rw::kinematics::Frame* frame, const rw::geometry::Geometry& geom);

    /**
     * @brief removes a geometry from the specified frame
     */
    void removeModel(rw::kinematics::Frame* frame, const std::string& geoid);

    /**
     * @brief return the ids of all the geometries of this frames.
     */
    std::vector<std::string> getGeometryIDs(rw::kinematics::Frame *frame);


};


}
}

#endif /* COLLISIONCHECKER_HPP_ */
