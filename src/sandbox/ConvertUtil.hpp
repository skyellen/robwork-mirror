/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

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
