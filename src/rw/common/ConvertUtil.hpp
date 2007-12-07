#ifndef CONVERTUTIL_HPP_
#define CONVERTUTIL_HPP_

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
        
    static float ToFloat32(unsigned char b0,
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

    static int ToFloat32(unsigned char arr[], int offset){
        int i = offset;
        return ToFloat32(arr[i], arr[i+1],arr[i+2], arr[i+3]);
    };
    
    static int ToInt32(unsigned char b0,
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
    
    static int ToInt32(unsigned char arr[], int offset){
        int i = offset;
        return ToInt32(arr[i], arr[i+1],arr[i+2], arr[i+3]);
    };
    
    static int ToInt16(unsigned char b0,
                       unsigned char b1)
    {
        ToData toint(1);
        toint.data[0] = b0;
        toint.data[1] = b1;
        return toint.int_val;
    };
    
    static int ToInt16(unsigned char arr[], int offset){
        int i = offset;
        return ToInt16(arr[i], arr[i+1]);
    };
};

#endif /*CONVERTUTIL_HPP_*/
