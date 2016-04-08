#include "trigfunctions.h"

double sintable[]={
    0,
    0.0174,
    0.0348,
    0.0523,
    0.0697,
    0.0871,
    0.1045,
    0.1218,
    0.1391,
    0.1564,
    0.1736,
    0.1908,
    0.2079,
    0.2249,
    0.2419,
    0.2588,
    0.2756,
    0.2923,
    0.3090,
    0.3255,
    0.3420,
    0.3583,
    0.3746,
    0.3907,
    0.4067,
    0.4226,
    0.4383,
    0.4539,
    0.4694,
    0.4848,
    0.5,
    0.5150,
    0.5299,
    0.5446,
    0.5591,
    0.5735,
    0.5877,
    0.6018,
    0.6156,
    0.6293,
    0.6427,
    0.6560,
    0.6691,
    0.6819,
    0.6946,
    0.7071,
    0.7193,
    0.7313,
    0.7431,
    0.7547,
    0.7660,
    0.7771,
    0.7880,
    0.7986,
    0.8090,
    0.8191,
    0.8290,
    0.8386,
    0.8480,
    0.8571,
    0.8660,
    0.8746,
    0.8829,
    0.8910,
    0.8987,
    0.9063,
    0.9135,
    0.9205,
    0.9271,
    0.9335,
    0.9396,
    0.9455,
    0.9510,
    0.9563,
    0.9612,
    0.9659,
    0.9702,
    0.9743,
    0.9781,
    0.9816,
    0.9848,
    0.9876,
    0.9902,
    0.9925,
    0.9945,
    0.9961,
    0.9975,
    0.9986,
    0.9993,
    0.9998,
    1
};

double mysin(int deg){
    int degrees = deg;

    //handle negative angles
    if (degrees < 0)
        degrees += 360;

    //first quadrant
    if (degrees <= 90)
        return sintable[degrees];
    //second quadrant
    else if (degrees <= 180)
        return sintable[180-degrees];
    //third quadrant
    else if (degrees <= 270)
        return -sintable[(180+degrees)-360];
    //fourth quadrant
    else if (degrees <= 360)
        return -sintable[(-degrees)+360];
    //error
    else
        return 0.0;
}

double mycos(int deg){
    return mysin(90-deg);
}
