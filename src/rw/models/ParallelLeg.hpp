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


#ifndef RW_MODELS_PARALLELLEG_HPP
#define RW_MODELS_PARALLELLEG_HPP

/**
 * @file ParallelLeg.hpp
 */

#include <vector>

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>

namespace rw {
    namespace math {
        class Jacobian;
    }
    namespace models {
        class Joint;
    }
    namespace kinematics {
        class State;
        class Frame;
    }
} // end namespaces

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
     * @brief Class representing a single leg in a ParallelDevice
     *
     */
    class ParallelLeg
    {
    public:
        /**
         * @brief Constructs leg from frames
         * @param frames [in] list of Frame's
         */
        ParallelLeg(std::vector<kinematics::Frame*> frames);

        /**
         * @brief Destructor
         */
        virtual ~ParallelLeg();

        /**
         * @brief Returns the base to end Jacobian
         * @param state [in] State for which to calculate the Jacobian
         * @return the Jacobian
         */
        const math::Jacobian& baseJend(const kinematics::State& state);

        /**
         * @brief Returns the base to end transformation
         * @param state [in] State for which to calculate the transform
         * @return the transform
         */
        math::Transform3D<double> baseTend(const kinematics::State& state);

        /**
         * @brief Returns the kinematic chain of the leg
         * @return list of frames
         */
        const std::vector<kinematics::Frame*>& getKinematicChain();

        /**
         * @brief the base of the leg
         * @return the frame
         */
        kinematics::Frame* getBase();

        /**
         * @brief the end of the leg
         * @return the frame
         */
        kinematics::Frame* getEnd();

        /**
         * @brief Number of active joints
         * @return number of active joints
         */
        size_t nrOfActiveJoints();

        /**
         * @brief Number of passive joints
         * @return number of passive joints
         */
        size_t nrOfPassiveJoints();

        /**
         * @brief Number of joints (both active and passive)
         * @return number of joints
         */
        size_t nrOfJoints(){return _actuatedJoints.size()+_unactuatedJoints.size();};

        /**
         * @brief Returns list of the actuated (active) joints
         * @return list of joints
         */
        const std::vector<models::Joint*>& getActuatedJoints(){return _actuatedJoints;};

        /**
         * @brief Returns list of unactuated (passive) joints
         * @return list of joints
         */
        const std::vector<models::Joint*>& getUnactuatedJoints(){ return _unactuatedJoints;};

        /**
         * @brief Sets q for the leg in the state
         * @param q [in] q to set
         * @param state [in] the State to modify
         */
        void setQ(const math::Q& q, kinematics::State& state) const;

    private:
        std::vector<kinematics::Frame*> _kinematicChain;
        std::vector<models::Joint*> _actuatedJoints;
        std::vector<models::Joint*> _unactuatedJoints;
        math::Jacobian *_jacobian;
    };

/*@}*/

}} // end namespaces

#endif // end include guard
