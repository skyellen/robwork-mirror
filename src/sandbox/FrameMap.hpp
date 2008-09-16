#ifndef FRAMEMAP_HPP_
#define FRAMEMAP_HPP_

#include <rw/kinematics/Frame.hpp>

#include <vector>

namespace rw {
namespace kinematics {

    /**
     * @brief a specialized mapping implementation for frames. It uses the internal
     * structure of Frames to provide fast O(1) lookup for mappings from Frame to anything.
     *
     * A requirement is that all frames must be registered in the same StateStructure.
     */
    template <class T>
    class FrameMap {
    public:

        /**
         * @brief creates a framemap
         */
        FrameMap(T defaultVal):
            _defaultVal(defaultVal)
            {}

        /**
         * @brief creates a framemap with an initial size of s
         */
        FrameMap(int s, T defaultVal):
            _map(s, defaultVal),
            _defaultVal(defaultVal)

        {

        }

        /**
         * @brief inserts a value
         */
        void insert(const rw::kinematics::Frame& frame, T& value){
            const int idx = frame.getID();
            if(idx>=_map.size())
                _map.resize(idx+1, _defaultVal);
            _map[ idx ] = value;
        }

        /**
         * @brief
         */
        const T& operator[](const rw::kinematics::Frame& frame) const {
            const int idx = frame.getID();
            if(idx>=_map.size())
                _map.resize(idx+1, _defaultVal);
            return _map[ idx ];
        }

        /**
         * @brief get the value associated with frame
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

