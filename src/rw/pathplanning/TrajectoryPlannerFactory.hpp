/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef rw_pathplanning_TrajectoryPlannerFactory_HPP
#define rw_pathplanning_TrajectoryPlannerFactory_HPP

/**
 * @file TrajectoryPlannerFactory.hpp
 */

#include <memory>

namespace rw { namespace models { class WorkCell; class Device; }}
namespace rw { namespace kinematics { class Frame; }}

namespace rw { namespace pathplanning {



    class TrajectoryPlanner;

    class TrajectoryPlannerFactory {
    public:
        virtual TrajectoryPlanner* make(
            rw::models::WorkCell* workcell,
            rw::models::Device* device,
            rw::kinematics::State& state) = 0;

        /**
           @brief Destructor
         */
        virtual ~TrajectoryPlannerFactory() {}

    private:
        TrajectoryPlannerFactory(const TrajectoryPlannerFactory&);
        TrajectoryPlannerFactory& operator=(const TrajectoryPlannerFactory&);

    protected:
        /**
           @brief Constructor
         */
        TrajectoryPlannerFactory() {}
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
