#ifndef RW_KINEMATICS_FRAMEMAP_HPP_
#define RW_KINEMATICS_FRAMEMAP_HPP_

#include <rw/kinematics/Frame.hpp>

#include <vector>

namespace rw { namespace kinematics {

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
        FrameMap(int s=20) :
            _defaultVal(),
            _map(s, _defaultVal),
            _has(s, false)

        {}

        /**
         * @brief creates a framemap with an initial size of s
         * @param s [in] nr of elements of the types T with default value "defaultVal"
         * @param defaultVal [in] the default value of new instances of T
         */
        FrameMap(T defaultVal, int s=20) :
            _defaultVal(defaultVal),
            _map(s, defaultVal),
            _has(s, false)
        {}

        /**
         * @brief inserts a value into the frame map
         * @param frame [in] the frame for which the value is to be associated
         * @param value [in] the value that is to be associated to the frame
         */
        void insert(const rw::kinematics::Frame& frame, const T& value)
        {
            operator[](frame) = value;
        }

        /**
           @brief True iff a value for \b frame has been inserted in the map (or
           accessed using non-const operator[]).
        */
        bool has(const rw::kinematics::Frame& frame)
        {
            const int idx = frame.getID();
            resizeIfNeeded(idx);
            return _has[idx];
        }

        /**
           @brief return a reference to the value that is associated with the
           frame \b frame.

           If no value has been inserted for \b frame, then the default value of
           \b T is returned. Use has() to see if a value has been stored for \b
           frame.

           @param frame [in] the frame for which to find its associated values.
           @return reference to the value associated to frame.
        */
        const T& operator[](const rw::kinematics::Frame& frame) const
        {
            const int idx = frame.getID();
            resizeIfNeeded(idx);
            return _map[idx];
        }

        /**
           @brief return a reference to the value that is associated with the
           frame \b frame

           If no value has been inserted for \b frame, then the default value of
           \b T is inserted in the map and returned.

           @param frame [in] the frame for which to find its associated values.
           @return reference to the value associated to frame.
        */
        T& operator[](const rw::kinematics::Frame& frame)
        {
            const int idx = frame.getID();
            resizeIfNeeded(idx);
            _has[idx] = true;
            return _map[idx];
        }

    private:
        void resizeIfNeeded(int idx) const
        {
            if (idx >= (int)_map.size()) {
                _map.resize(idx + 1, _defaultVal);
                _has.resize(_map.size(), false);
            }
        }

    private:
        T _defaultVal;
        mutable std::vector<T> _map;
        mutable std::vector<bool> _has;
    };
}}

#endif /*FRAMEMAP_HPP_*/
