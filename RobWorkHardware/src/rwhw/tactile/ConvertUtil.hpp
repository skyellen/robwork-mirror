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

#ifndef CONVERTUTIL_HPP_
#define CONVERTUTIL_HPP_


/**
 * @brief Range of small converting utilities.
 *
 * IMPORTANT: These method are not tested on all platform.
 */
class ConvertUtil
{
    union ToData
    {
        ToData(float val) : float_val(val) {}
        ToData(int val) : int_val(val) {}

        int int_val;
        float float_val;
        unsigned char data[4];
    };

public:
    /**
     * @brief Converts 4 chars into a float.
     */
    static float toFloat32(
        unsigned char b0,
        unsigned char b1,
        unsigned char b2,
        unsigned char b3 )
    {
        ToData tofloat(1);
        tofloat.data[0] = b0;
        tofloat.data[1] = b1;
        tofloat.data[2] = b2;
        tofloat.data[3] = b3;
        return tofloat.float_val;
    };

    /**
     * @brief Converts an array of chars into a float. The offset \b specifies where to start
     * reading the 4 bytes.
     */
    static float toFloat32(unsigned char arr[], int offset)
    {
        int i = offset;
        return toFloat32(arr[i], arr[i+1],arr[i+2], arr[i+3]);
    };

    /**
     * @brief Converts 4 chars into a 32 bit integer
     */
    static int toInt32(
        unsigned char b0,
        unsigned char b1,
        unsigned char b2,
        unsigned char b3 )
    {
        ToData toint(1);
        toint.data[0] = b0;
        toint.data[1] = b1;
        toint.data[2] = b2;
        toint.data[3] = b3;
        return toint.int_val;
    };

    /**
     * @brief Converts an array of chars into a 32bit integer. The offset \b specifies where to start
     * reading the 4 bytes.
     */
    static int toInt32(unsigned char arr[], int offset)
    {
        int i = offset;
        return toInt32(arr[i], arr[i+1],arr[i+2], arr[i+3]);
    };

    /**
     * @brief Converts 2 chars into a 16 bit integer
     */
    static int toInt16(unsigned char b0, unsigned char b1)
    {
        ToData toint(1);
        toint.data[0] = b0;
        toint.data[1] = b1;
        return toint.int_val;
    };

    /**
     * @brief Converts an array of chars into a 32bit integer. The offset \b specifies where to start
     * reading the 2 bytes.
     */
    static int toInt16(unsigned char arr[], int offset)
    {
        int i = offset;
        return toInt16(arr[i], arr[i+1]);
    };
};


#endif // end namespaces
