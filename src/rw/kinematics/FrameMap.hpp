/*********************************************************************
 * RobWork Version 0.3
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

#ifndef RW_KINEMATICS_FRAMEMAP_HPP
#define RW_KINEMATICS_FRAMEMAP_HPP

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
         * @param s [in] the default value of new instances of T
         */
        FrameMap(int s = 20) :
            _initialSize(s),
            _defaultVal(false, T()),
            _map(_initialSize, _defaultVal)
        {}

        /**
         * @brief creates a framemap with an initial size of s
         * @param s [in] nr of elements of the types T with default value "defaultVal"
         * @param defaultVal [in] the default value of new instances of T
         */
        FrameMap(const T& defaultVal, int s = 20) :
            _initialSize(s),
            _defaultVal(false, defaultVal),
            _map(_initialSize, _defaultVal)
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
            return _map[idx].first;
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
            return _map[idx].second;
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
            OkVal& val = _map[idx];
            val.first = true;
            return val.second;
        }

        /**
           @brief Clear the frame map.
        */
        void clear()
        {
            _map.clear();
            _map.resize(_initialSize, _defaultVal);
        }

    private:
        void resizeIfNeeded(int idx) const
        {
            const int n = (int)_map.size();
            if (idx >= n) {
                const int newSize = idx >= 2 * n ? idx + 1 : 2 * n;
                _map.resize(newSize, _defaultVal);
            }
        }

    private:
        typedef std::pair<bool, T> OkVal;

        int _initialSize;
        OkVal _defaultVal;
        mutable std::vector<OkVal> _map;
    };
}}

#endif /*RW_KINEMATICS_FRAMEMAP_HPP*/
