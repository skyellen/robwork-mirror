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


#ifndef RW_COLLISION_COLLISIONSETUPLOADER_HPP
#define RW_COLLISION_COLLISIONSETUPLOADER_HPP

#include <string>
#include <rw/proximity/CollisionSetup.hpp>


/**
 * @file colsetup/CollisionSetupLoader.hpp
 */

namespace rw { namespace loaders {

    /** @addtogroup loaders */
    /*@{*/

    /**
     * @brief Loader of collision setups from files.
     *
     * [CollisionSetupLoader could just as well simply be a utility function.]
     *
     * The XML format is:
\verbatim
      <CollisionSetup>
          <Exclude>
              <FramePair first="first_frame" second="second_frame"/>
              ...
          </Exclude>
          <Volatile>frame1 frame2 .. </Volatile>
      </CollisionSetup>
\endverbatim
     */
    class CollisionSetupLoader
    {
    public:
        /**
         * @brief Load a collision setup from the file \b file.
         *
         * \b prefix is prepended to every frame name.
         *
         * @param prefix [in] The context in which the setup is loaded.
         *
         * @param file [in] The file from which to load the collision setup.
         *
         * @return The collision setup.
         */
         static rw::proximity::CollisionSetup load(
            const std::string& prefix,
            const std::string& file);

    private:
        CollisionSetupLoader();
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
