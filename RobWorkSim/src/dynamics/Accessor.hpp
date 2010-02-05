#ifndef RW_DYNAMICS_ACCESSOR_HPP_
#define RW_DYNAMICS_ACCESSOR_HPP_

#include <rw/kinematics/FrameProperty.hpp>
#include <rw/kinematics/FrameType.hpp>
#include <rw/models/RigidBodyInfo.hpp>



namespace dynamics{

/**
 * @brief a set of accessor functions for accessing frame properties regarding
 * Dynamic stuff.
 */
class Accessor
{
public:
    /** @brief Accessor for RigidBodyInfo
     *
     * This is used to get RigidBodyInfo
     */
	static const rw::kinematics::FrameProperty<rw::models::RigidBodyInfo>& RigidBodyInfo();


};

}
#endif /*ACCESSOR_HPP_*/
