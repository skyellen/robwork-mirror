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

/*
  We hide it until it is documented. Also it isn't platform independent.
*/

/// @cond SHOW_ALL

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

    static float toFloat32(unsigned char arr[], int offset)
    {
        int i = offset;
        return toFloat32(arr[i], arr[i+1],arr[i+2], arr[i+3]);
    };

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

    static int toInt32(unsigned char arr[], int offset)
    {
        int i = offset;
        return toInt32(arr[i], arr[i+1],arr[i+2], arr[i+3]);
    };

    static int toInt16(unsigned char b0, unsigned char b1)
    {
        ToData toint(1);
        toint.data[0] = b0;
        toint.data[1] = b1;
        return toint.int_val;
    };

    static int toInt16(unsigned char arr[], int offset)
    {
        int i = offset;
        return toInt16(arr[i], arr[i+1]);
    };
};

/// @endcond SHOW_ALL

#endif // end namespaces
