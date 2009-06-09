/*
 * StaticListFilter.hpp
 *
 *  Created on: Apr 23, 2009
 *      Author: jimali
 */

#ifndef STATICLISTFILTER_HPP_
#define STATICLISTFILTER_HPP_

#include "BroadPhaseStrategy.hpp"
#include <rw/models/WorkCell.hpp>
#include "CollisionSetup.hpp"
#include <rw/kinematics/Frame.hpp>

namespace rw { namespace proximity { namespace sandbox {

class StaticListFilter: public BroadPhaseStrategy {
public:

	StaticListFilter(rw::models::WorkCellPtr workcell);

	StaticListFilter(rw::models::WorkCellPtr workcell, const CollisionSetup& setup);

	void addInclude(const kinematics::FramePair& framepair);

	void addExclude(const kinematics::FramePair& framepair);

	/**
	 * @brief
	 */
	virtual void reset(const rw::kinematics::State& state);

	/**
	 * @brief
	 */
	virtual void update(const rw::kinematics::State& state);

	virtual const rw::kinematics::FramePair& next();

	virtual bool hasNext();

	virtual CollisionSetup& getCollisionSetup();

};

}
}
}

#endif /* STATICLISTFILTER_HPP_ */
