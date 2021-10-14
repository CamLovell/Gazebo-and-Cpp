#ifndef LIDAR_H
#define LIDAR_H

// Put structs here to make cpp a little cleaner, allows for use in other files if need be
struct Map{
    double x0, y0, dx, dy, *x, *y, *z;
    int numX, numY;
};

struct Scanner{
    double x0, y0, h0;
    double startDeg, resDeg, maxRange;
    int numScans;
};

struct Pose{
    double x, y, h;
};

#endif