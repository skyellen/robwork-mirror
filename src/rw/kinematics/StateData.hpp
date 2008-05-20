#ifndef RW_KINEMATICS_STATEDATA_HPP_
#define RW_KINEMATICS_STATEDATA_HPP_

#include <string>
#include <ostream>
#include <iostream>

#include "State.hpp"

namespace rw { namespace kinematics {
    /** @addtogroup kinematics */
    /*@{*/

    /**
     * @brief the basic building block for the stateless desing using
     * the StateStructure class. A StateData represents a size,
     * a unique id, and a unique name, when inserted into the StateStructure. 
     * The size will allocate "size"-doubles in State objects originating from the 
     * StateStructure.  
     */
    class StateData {
    
    public:
        /**
         * @brief destructor
         */
        virtual ~StateData(){};
        
        /**
         * @brief An integer ID for the StateData.
         *
         * IDs are assigned to the state data upon insertion State. 
         * StateData that are not in a State have an ID of -1.
         *
         * StateData present in different trees may have identical IDs.
         *
         * IDs are used for the efficient implementation of State. Normally,
         * you should not make use of frame IDs yourself.
         *
         * @return An integer ID for the frame.
         */
        inline int getID() const { return _id; }
        
        /**
         * @brief The name of the state data.
         *
         * @return The name of the state data.
         */
        const std::string& getName() const { return _name; }
        
        /**
         * @brief The number of doubles allocated by this StateData in 
         * each State object.
         *
         * @return The number of doubles allocated by the StateData
         */        
        inline int size() const { return _size; };
                
        // The StateData values.
        /**
         * @brief An array of length size() containing the values for
         * the state data.
         *
         * It is OK to call this method also for a StateData with zero size.
         *
         * @param state [in] The state containing the StateData values.
         *
         * @return The joint values for the frame.
         */
        inline const double* getQ(const State& state) const {
            if( _size==0 ) return NULL; // stop early if we know size is 0
            return state.getQState().getQ(*this);
        }
    
        /**
         * @brief Assign for \b state data the size() of values of the array \b
         * vals.
         *
         * The array \b vals must be of length at least size().
         *
         * @param state [inout] The state to which \b vals are written.
         *
         * @param vals [in] The joint values to assign.
         *
         * setQ() and getQ() are related as follows:
         * \code
         * data.setQ(state, q_in);
         * const double* q_out = data.getQ(state);
         * for (int i = 0; i < data.getDof(); i++)
         *   q_in[i] == q_out[i];
         * \endcode
         */
        inline void setQ(State& state, const double* vals) const{
            if( _size==0 ) return; // stop early if we know size is 0
            state.getQState().setQ(*this, vals);
        }
    
    protected:
        /**
         * @brief A state with \b size number of doubles in the State vector.
         *
         * \b size must be non-negative.
         *
         * The newly created state data can be added to a structure with 
         * StateStructure::addData().
         *
         * The size of the state data in nr of doubles of the state data 
         * is constant throughout
         * the lifetime of the state data.
         *
         * @param size [in] The number of degrees of freedom of the frame.
         *
         * @param name [in] The name of the frame.
         */
        StateData(int size, const std::string& name);
    
    private:
        // The tree is responsible for the assignment of the IDs that are later
        // used in the State implementation. Tree is a friend so that IDs can
        // be modified only from there. The common advice is that the friend
        // class should be in the same header file as this class, but we break
        // that advice to allow the Tree declaration to be excluded from the
        // Doxygen documentation.
        friend class StateStructure;
    
        void setID(int id) { _id = id; }
        
    private:
        
        // An integer ID for the frame. Assignment of ID values is the
        // responsibility of the tree in which the frame is inserted. The ID of
        // a frame may change over time.
        int _id;
        
        // The size of the state memmory allocated for this state data. 
        // This value remains fixed throughout the life time of the statedata.
        const int _size;
        
        // The name of the state data. 
        std::string _name;
        
    private:
        // StateData should not be copied.
        StateData(const StateData&);
        StateData& operator=(const StateData&);
    
    };
    /*@}*/
}
}

#endif /*STATEDATA_HPP_*/
