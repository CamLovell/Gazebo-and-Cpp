#ifndef SENSORDATA_H
#define SENSORDATA_H

struct imuMeasure{
    static double phi, theta, psi;
};

struct gpsNED{
    static double N, E, D;
};
struct gpsECEF{
    static double N, E, D;
};

#endif