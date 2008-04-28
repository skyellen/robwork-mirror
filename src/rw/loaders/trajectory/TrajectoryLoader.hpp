/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
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

#ifndef rw_collision_TrajectoryLoader_HPP
#define rw_collision_TrajectoryLoader_HPP

/**
 * @file TrajectoryLoader.hpp
 */

#include <string>

#include <rw/interpolator/Trajectory.hpp>

namespace rw { 
    namespace loaders {

    /** @addtogroup loaders */
    /*@{*/

    /**
     * @brief Loader of cartesean or joint trajectory from files.
     *
     * The XML format is:
\verbatim
      <Trajectory type="Joint|Cartesean">
          <Interpolator type="StraightLine">
              <Via time="0">0 0 0 0 0 0 0 0 0</Via>
              ....
          </Interpolator>
          <Blend type="LloydHayward" />
          <Interpolator type="CubicSpline">
              <Via time="">0 0 0 0 0 0 0 0 0</Via>
              ....
          </Interpolator>
          ...
      </CollisionSetup>
\endverbatim
     */
    class TrajectoryLoader
    {
    public:
        /**
         * @brief Load a trajectory from the file \b file.
         *
         * @param file [in] The file from which to load the trajectory.
         *
         * @return The collision setup.
         */
        static rw::interpolator::Trajectory load(const std::string& file);

    private:
        TrajectoryLoader();
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
