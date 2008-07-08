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

#ifndef rw_kinematics_QState_HPP
#define rw_kinematics_QState_HPP

/**
 * @file QState.hpp
 */

#include <boost/shared_ptr.hpp>
#include <vector>
#include <rw/math/Q.hpp>

namespace rw { namespace kinematics {
    class StateSetup;
    class StateData;

    /** @addtogroup kinematics */
    /*@{*/

    /**
     * @brief The configuration state of a work cell.
     *
     * The configuration state contains state data values for all 
     * valid StateData in a StateStructure. The validity is defined by the
     * StateSetup.
     *
     * See Frame::getTransform() for the calculation of the relative transform
     * of a frame for a given configuration state.
     *
     * Configuration states can be freely copied and assigned.
     *
     * The configuration state is a part of the StateStructure state (see
     * State).
     */
    class QState
    {
    public:
        /**
         * @brief Constructs an empty QState
         */
        QState();
        
        /**
         * @brief A configuration state.
         *
         * This constructor is not available for use outside of RobWork. Instead
         * your configuration states should be constructed via the copy
         * constructor.
         *
         * @param setup [in] The shared setup for configuration states.
         */
        explicit QState(boost::shared_ptr<StateSetup> setup);        
        
        
        virtual ~QState() {
           // std::cout<<"QState Destructor"<<std::endl;
        }
        
        /**
         * @brief An array of length frame.getDOF() containing the joint values
         * for \b frame.
         *
         * It is OK to call this method also for frames with zero degrees of
         * freedom.
         *
         * @return The joint values for the frame.
         */
        const double* getQ(const StateData& data) const;

        /**
         * @brief Assign for \b frame the frame.getDOF() joint values of the
         * array \b vals.
         *
         * The array \b vals must be of length at least frame.getDOF().
         *
         * @param frame [in] The frame for which the joint values are assigned.
         *
         * @param vals [in] The joint values to assign.
         *
         * setQ() and getQ() are related as follows:
         * \code
         * q_state.setQ(frame, q_in);
         * const double* q_out = q_state.getQ(frame);
         * for (int i = 0; i < frame.getDOF(); i++)
         *   q_in[i] == q_out[i];
         * \endcode
         */
        void setQ(const StateData& data, const double* vals);

        /**
         * @brief streaming operator
         *
         * @param os [in] output stream
         *
         * @param state [in] state to stream out
         *
         * @return the stream
         */
        friend std::ostream& operator<<(std::ostream& os, const QState& state)
        {
            os << state._contents;
            return os;
        }

        /**
         * @brief Scaling of a configuration state by a scalar.
         */
        friend QState operator*(const QState& q, double scale)
        {
            return QState(scale * q._contents, q._setup);
        }

        /**
         * @brief Scaling of a configuration state by a scalar.
         */
        friend QState operator*(double scale, const QState& q)
        {
            return QState(scale * q._contents, q._setup);
        }

        /**
         * @brief Addition of configuration states.
         */
        friend QState operator+(const QState& a, const QState& b)
        {
            // It does not matter here if we use the setup of a or b.
            // They are assumed to be the identical.
            return QState(a._contents + b._contents, a._setup);
        }

        /**
         * @brief Subtraction of configuration states.
         */
        friend QState operator-(const QState& a, const QState& b)
        {
            return QState(a._contents - b._contents, a._setup);
        }

        /**
         * @brief Unary minus operator.
         */
        QState operator-() const
        {
            return QState(-_contents, _setup);
        }
        
        /**
         * @brief returns the StateSetup
         */
        boost::shared_ptr<StateSetup> getStateSetup() const{
            return _setup;
        }
        
        //void copy(const QState& qstate);
        
        QState& operator=(const QState &rhs);

        
        /**
           @brief The dimension of the state vector.
         */
        size_t size() const { return _contents.size(); }

    private:
        QState(
            const math::Q& contents,
            boost::shared_ptr<StateSetup> setup)
            :
            _contents(contents),
            _setup(setup)
        {}

    private:
        math::Q _contents;
        boost::shared_ptr<StateSetup> _setup;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
