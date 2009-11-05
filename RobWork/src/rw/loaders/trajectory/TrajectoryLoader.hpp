/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
 * Faculty of Engineering, University of Southern Denmark 
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#ifndef RW_LOADERS_TRAJECTORYLOADER_HPP
#define RW_LOADERS_TRAJECTORYLOADER_HPP

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
    class XMLTrajectoryLoader
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
