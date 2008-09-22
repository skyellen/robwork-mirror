#ifndef RW_KINEMATICS_FRAMEMAP_HPP_
#define RW_KINEMATICS_FRAMEMAP_HPP_

#include <rw/kinematics/Frame.hpp>

#include <vector>

namespace rw {
namespace kinematics {

    /**
     * @brief a specialized mapping implementation for frames. It uses the internal
     * structure of Frames to provide fast O(1) lookup for mappings from Frame to anything.
     *
     * @note A requirement is that all frames must be registered in the same StateStructure.
     */
    template <class T>
    class FrameMap {
    public:

        /**
         * @brief creates a framemap
         * @param defaultVal [in] the default value of new instances of T
         */
        FrameMap(T defaultVal):
            _defaultVal(defaultVal)
        {}

        /**
         * @brief creates a framemap with an initial size of s
         * @param s [in] nr of elements of the types T with default value "defaultVal"
         * @param defaultVal [in] the default value of new instances of T
         */
        FrameMap(int s, T defaultVal):
            _map(s, defaultVal),
            _defaultVal(defaultVal)
        {}

        /**
         * @brief inserts a value into the frame map
         * @param frame [in] the frame for which the value is to be associated
         * @param value [in] the value that is to be associated to the frame
         */
        void insert(const rw::kinematics::Frame& frame, T& value){
            const int idx = frame.getID();
            if(idx>=_map.size())
                _map.resize(idx+1, _defaultVal);
            _map[ idx ] = value;
        }

        /**
         * @brief return a reference to the value that is associated with the frame "frame"
         * @param frame [in] the frame for which to find its associated values.
         * @return reference to the value associated to frame.
         */
        const T& operator[](const rw::kinematics::Frame& frame) const {
            const int idx = frame.getID();
            if(idx>=_map.size())
                _map.resize(idx+1, _defaultVal);
            return _map[ idx ];
        }

        /**
         * @brief return a reference to the value that is associated with the frame "frame"
         * @param frame [in] the frame for which to find its associated values.
         * @return reference to the value associated to frame.
         */
        T& operator[](const rw::kinematics::Frame& frame) {
            const int idx = frame.getID();
            if(idx>=_map.size())
                _map.resize(idx+1, _defaultVal);
            return _map[ idx ];
        }

    private:
        const T _defaultVal;
        mutable std::vector<T> _map;
    };

}
}

#endif /*FRAMEMAP_HPP_*/

