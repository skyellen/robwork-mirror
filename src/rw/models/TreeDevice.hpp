#ifndef TREEDEVICE_HPP_
#define TREEDEVICE_HPP_

/**
 * @file TreeDevice.hpp
 */

#include "DeviceModel.hpp"
#include "BasicDevice.hpp"
#include "BasicDeviceJacobian.hpp"

#include <rw/math/Transform3D.hpp>

#include <vector>
#include <map>
#include <memory>

namespace rw { namespace math {
    class jacobian;
}} // end namespaces

namespace rw { namespace models {

    class Joint;

    /** @addtogroup models */
    /*@{*/

    /**
     * @brief A tree structured device
     *
     * @dot
     * digraph TreeDevice {
     *  node [shape=record, fontname=Helvetica, fontsize=10, style=filled];
     *  Base [ fillcolor="red"];
     *  Link1 [ label="Link1\n<Link>", fillcolor="red"];
     *  Axis1 [ label="Axis1\n<Joint>", fillcolor="red"];
     *  Link2 [ label="Link2\n<Link>",fillcolor="red"];
     *  Axis2 [ label="Axis2\n<Joint>",fillcolor="red"];
     *  Link3 [ label="Link3\n<Link>",fillcolor="red"];
     *  Axis3 [ label="Axis1\n<Joint>",fillcolor="red"];
     *  EndEffector1 [ fillcolor="red"];
     *  Link4 [ label="Link2\n<Link>",fillcolor="red"];
     *  Axis4 [ label="Axis2\n<Joint>",fillcolor="red"];
     *  Link5 [ label="Link3\n<Link>",fillcolor="red"];
     *  Axis5 [ label="Axis1\n<Joint>",fillcolor="red"]; 
     *  EndEffector2 [ fillcolor="red"];
     *  Link6 [ label="Link3\n<Link>",fillcolor="red"];
     *  Axis6 [ label="Axis1\n<Joint>",fillcolor="red"];     * 
     *  EndEffector3 [ fillcolor="red"];
     *
     * world -> object1;
     * world -> table;
     * table -> object2;
     * world -> Base;
     * Base -> Link1;
     * Link1 -> Axis1;
     * Axis1 -> Link2;
     * Link2 -> Axis2;
     * Axis2 -> Link3;
     * Link3 -> Axis3;
     * Axis3 -> EndEffector1;
     * Axis1 -> Link4
     * Link4 -> Axis4
     * Axis4 -> Link5
     * Link5 -> Axis5
     * Axis5 -> EndEffector2 
     * Axis2 -> Link6
     * Axis6 -> EndEffector3
     * }
     * @enddot
     *
     * Example of usage:
     * @code
     * std::vector< Frame* > endEffectors;
     * TreeDevice TreeDevice(base, endEffectors, activeJoints);
     * std::cout << "Jacobian: " << TreeDevice.bJe() << std::endl;
     * @endcode
     *
     * @todo document this
     */
    class TreeDevice : public DeviceModel
    {
    public:
        /**
         * @brief Creates object
         *
         * @param base [in] the base frame of the robot
         * @param last [in] the set of default endeffector of the robot
         * @param name [in] name of device
         * @param state [in] the initial state of everything
         *
         * @pre links[0] must be located below first in the chain
         *
         * @pre links[links.size()-1] mst be the last in the chain. This will
         * represent the default endeffector.
         *
         * @pre joints must be a subset of the kinematic chain between the first
         * and last link
         *
         */
        TreeDevice(
            kinematics::Frame* base,
            std::vector< kinematics::Frame* > last,
            const std::string& name,
            const kinematics::State& state);

        /**
         * @brief destructor
         */
        virtual ~TreeDevice();

        /**
         * @copydoc DeviceModel::setQ
         *
         * @pre q.size() == activeJoints.size()
         */
        void setQ(const math::Q& q, kinematics::State& state) const;

        /**
         * @copydoc DeviceModel::getQ
         */
        math::Q getQ(const kinematics::State& state) const;

        /**
         * @copydoc DeviceModel::getDOF
         */
        size_t getDOF() const{
            return _activeJoints.size();
        }

        /**
         * @copydoc DeviceModel::getBounds
         */
        std::pair<math::Q, math::Q> getBounds() const;

        /**
         * @copydoc DeviceModel::setBounds
         */
        void setBounds(const std::pair<math::Q, math::Q>& bounds);

        /**
         * @copydoc DeviceModel::getVelocityLimits
         */
        math::Q getVelocityLimits() const;

        /**
         * @copydoc DeviceModel::setVelocityLimits
         */
        void setVelocityLimits(const math::Q& vellimits);

        /**
         * @brief DeviceModel::getAccelerationLimits
         */
        math::Q getAccelerationLimits() const;

        /**
         * @brief DeviceModel::setAccelerationLimits
         */
        void setAccelerationLimits(const math::Q& acclimits);

        /**
         * @brief Returns pointer to active joint
         * @param index [in] joint index
         * @return a joint
         */
        Joint* getActiveJoint(size_t index) const
        {
            assert(index < _activeJoints.size());
            return _activeJoints[index];
        }

        /**
         * @brief Returns reference to kinematic chain
         *
         * @return a reference to the list of frames that defines the kinematic
         * chain of the serial robot
         */
        const std::vector<kinematics::Frame*>& frames() const{
            return _kinematicChain;
        }

        /**
         * @copydoc DeviceModel::getBase
         */
        kinematics::Frame* getBase(){
            return _base;
        };

        /**
         * @copydoc DeviceModel::getBase
         */
        const kinematics::Frame* getBase() const {
            return _base;
        };

        /**
         * @copydoc DeviceModel::getEnd()
         */
        virtual kinematics::Frame* getEnd() {
            return _end.front();
        };

        /**
         * @copydoc DeviceModel::getEnd() const
         */
        virtual const kinematics::Frame* getEnd() const {
            return _end.front();
        };

        /**
         * @brief a method to return all endeffectors of this TreeDevice
         * @return a list of end effector frames
         */
        virtual const std::vector<kinematics::Frame*>& getEnds() const{
            return _end;
        };        
        
        /**
         * @copydoc DeviceModel::baseJend
         */
        math::Jacobian baseJend(const kinematics::State& state) const;

        /**
         * @copydoc DeviceModel::baseJframe
         */
        math::Jacobian baseJframe(
            const kinematics::Frame *frame,
            const kinematics::State& state) const;

        /**
         * @brief like baseJend but with a jacobian calculated for all end
         * effectors.
         */
        math::Jacobian baseJends(const kinematics::State& state) const;

        /**
         * @brief like baseJframe but with a jacobian calculated for a
         * list of end frames.
         * @return a BasicDeviceJacobian that can calculate the jacobian
         * using get(..) method
         */
        boost::shared_ptr<BasicDeviceJacobian> baseJframes(
            const std::vector<kinematics::Frame*>& frames,
            const kinematics::State& state) const;
    
    private:

        kinematics::Frame* _base;

        std::vector<kinematics::Frame*> _end;

        std::vector<kinematics::Frame*> _kinematicChain;

        std::vector<Joint*> _activeJoints;

        // getQ(), getBounds(), etc. are forwarded to this object.
        BasicDevice _basicDevice;

        // Base to end Jacobians are computed here.
        std::vector<boost::shared_ptr<BasicDeviceJacobian> > _dj;
        // Base to multiple ends Jacobian are calculated here
        boost::shared_ptr<BasicDeviceJacobian> _djmulti;
        
        class JacobianImpl;
        class ForwardKinematicsImpl;
    };

    /*@}*/
}} // end namespaces

#endif /*TREEDEVICE_HPP_*/
