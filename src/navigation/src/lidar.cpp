#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <limits>
#include <iostream>
#include <Eigen/Core>

#include "lidar.h"
#include "map.h"
#include "helpers.h"

//----------------------------------------------------------------------------------------------------------------------------------------
// Initialise lidar parameters
//----------------------------------------------------------------------------------------------------------------------------------------
void lidatInit(Lidar& lidarParams){
    double sigma = 0.05, // Sigma for lidar range measurments in m
        wHit = 0.9, // Weight for a true/good hit (most will be good)
        wShort = 0.09, // Weight for a short hit 
        wRand = 0.01; // Weight for a random hit 

    lidarParams.x0 = 0.0; // Relative x(forwards) offset of lidar from body
    lidarParams.y0 = 0.0; // Relative y(sideways) offset of lidar from body
    lidarParams.psi0 = 0.0; // Relative angle(vetical) offset of lidar from body

    lidarParams.startDeg = 0.0; // Starting location of scan in degrees
    // lidarParams.resDeg = 360.0/10400.0; //10400 scans per revolution
    lidarParams.resDeg = 1.0; //10400 scans per revolution
    lidarParams.stopDeg = 360-lidarParams.resDeg; // Starting location of scan in degrees

    lidarParams.scanRes = 0.003; //10400 scans per revolution
    lidarParams.maxRange = 150.0; // Maximum range of lidar in m
    lidarParams.minRange = 1.0; // Minimum range of lidar in m
    lidarParams.sigma2 = sigma*sigma; // Precomputed sigma^2
    lidarParams.lambda = 0.5; // Lambda value for short hit exponential distribution
    lidarParams.lwHit = log(wHit); // log hit weight
    lidarParams.lwShort =  log(wShort); // log short weight
    lidarParams.lwRand = log(wRand); // log random weight
    // lidarParams.numScans = 10400.0; // May need to look more closely
    lidarParams.numScans = 360.0; // May need to look more closely

    lidarParams.theta.setLinSpaced(lidarParams.numScans,lidarParams.startDeg,lidarParams.stopDeg); // Vector of scan line angles
    // std::cout << lidarParams.theta(359) << std::endl;
}



//----------------------------------------------------------------------------------------------------------------------------------------
// Lidar Log Likelihood funciton
// Note:
// - Loops suck for speed but logical checks with eigen are not particularly friendly
// - May be worth looking into if speed is an issue
//----------------------------------------------------------------------------------------------------------------------------------------
void lidarLogLiklihood(const Eigen::VectorXd& y, const Eigen::MatrixXd& x, const int& M, Lidar& lidarM8, Map& map, Eigen::VectorXd& lw){
    double logHit, logShort, logRand;
    Eigen::VectorXd rangeParticle, pose, xr, yr, C;
    lw.setZero(M);

    // Loop through each particle
    for(int i = 0; i<M; i++){
        pose = x.block(0,i,3,1);

        // Check if current particle sits in occupied map cell
        int Nidx = round(((pose(0)-map.N(1))/(map.N(map.N.size()-1)-map.N(0)))*(map.N.size()-1));
        int Eidx = round(((pose(1)-map.E(1))/(map.E(map.E.size()-1)-map.E(0)))*(map.E.size()-1));
        // Limit indicies to size of vectors
        if(Nidx>(map.numY-1)){
            Nidx=(map.numY-1);
        }
        if(Eidx>(map.numX-1)){
            Eidx=(map.numX-1);
        }
        if(!map.z(Nidx,Eidx)){
            // Get expected range measurments for current particle
            findRange(map, lidarM8, pose, rangeParticle, xr, yr, C);
            
            // Loop through measurements and calculate likelihoood of current particle
            for(int k = 0; k<lidarM8.numScans; k++){
                // Probability of a true hit
                if (y(k) >= lidarM8.minRange && y(k) <= lidarM8.maxRange){
                    logHit = log(1/sqrt(2*M_PI*lidarM8.sigma2))+(-0.5*((y(k)-rangeParticle(k))*(y(k)-rangeParticle(k)))/lidarM8.sigma2);
                }
                else{
                    logHit = -std::numeric_limits<double>::infinity();
                }

                // Probability of a short hit
                if (y(k) >= 0 && y(k) <= rangeParticle(k)){
                    logShort = log(lidarM8.lambda)-lidarM8.lambda*y(k)-log(1-exp(-lidarM8.lambda*rangeParticle(k)));
                    // Precompute log(lambda)??
                }
                else{
                    logShort = -std::numeric_limits<double>::infinity();
                }
                // Probaility of a random hit

                if (y(k) >= 0 && y(k) <= lidarM8.maxRange){
                    logRand = -log(lidarM8.maxRange); // precompute??
                }
                else{
                    logRand = -std::numeric_limits<double>::infinity();
                }

                //----------------------------------------------------------------------
                // Note:
                // No max range needed as M* returns 0 for all invalid measurments
                //----------------------------------------------------------------------

                Eigen::VectorXd temp(3);
                temp << lidarM8.lwHit+logHit, lidarM8.lwShort+logShort, lidarM8.lwRand+logRand;
                lw(i) = lw(i) + logSumExponential(temp);
            }
        }
        else{
            // Set log likelihood to negative infinity of particle is in occipied space
            lw(i) = -std::numeric_limits<double>::infinity();
        }
    }
}


//----------------------------------------------------------------------------------------------------------------------------------------
// Lidar Scan Simulation
//----------------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------------------------
// NOTE:
// -Adapted from function provided in MCHA4100 and modified to take
//  eigen inputs to better integrate with the rest of the solution
//
// -Could be beneficial in terms of speed to look into vectorising
//  the actual calculations using eigen rather than using loops
//
// -For now the function will remain in this form due to time constraints
//----------------------------------------------------------------------------------------------------------------------------------------

#define eps  1e-15
#define dot(x,y)   ((x)(0) * (y)(0) + (x)[1] * (y)[1] + (x)[2] * (y)[2]) 

#define max(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
#define min(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })

// Intersecton point of two line segments
int linSegInt(const double & x1, const double & x2, const double & y1,const double & y2,const double & x3, const double & x4, const double & y3, const double & y4, double *xint, double *yint){
    double num1, num2, den, u1, u2;
    int ind1, ind2, ind3;
    
    num1 = (x4-x3)*(y1-y3) - (y4-y3)*(x1-x3);
    num2 = (x2-x1)*(y1-y3) - (y2-y1)*(x1-x3);
    den  = (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1);

    u1      = num1/den;
    u2      = num2/den;

    xint[0] = x1+(x2-x1)*u1;
    yint[0] = y1+(y2-y1)*u1;

    ind1    = (u1 >= 0.0) && (u1 <= 1.0) && (u2 >= 0.0) && (u2 <= 1.0);
    ind2    = ((num1 == 0.0) && (num2 == 0.0) && (ind1 == 1));
    ind3    = (den == 0);
    
    return (ind1==1) && (ind2==0) && (ind3==0);
}

// Get range measrements based on current map
void findRange(const Map & map,const Lidar & lidarM8,const Eigen::VectorXd& pose, Eigen::VectorXd & range, Eigen::VectorXd & xr, Eigen::VectorXd & yr, Eigen::VectorXd & C){

    int L = lidarM8.numScans;
    range.resize(L);
    xr.resize(L);
    yr.resize(L);
    C.resize(L);

    double x0, y0, psi0, x1, x2, y1 ,y2, xs, ys, xe, ye;
    double r[4], xint[4], yint[4];

    double xmin, xmax, ymin, ymax, ycur;
    int xmini, xmaxi, ymini, ymaxi, ynext;
    int N, M, j, hit, ind[4];
    
    N = map.numX; //Number of x grid lines (should be num x cells + 1)
    M = map.numY; //Number of y grid lines (should be num y cells + 1)
    
    //Map origin
    double dx    = map.dE;
    double dy    = map.dN;
    double mapx0 = map.E0;
    double mapy0 = map.N0;
    
    x0 = pose(0) - lidarM8.x0;
    y0 = pose(1) - lidarM8.y0;
    psi0 = pose(2) - lidarM8.psi0;
    
    int xsi, xei, ysi, yei, xci, yci;
    double dxse, dyse, xc, yc, xdist, ydist;
    
    //Loop over the number of scan lines
    for(int i=0;i<L;i++){
        
        //Max range to begin with and no hit
        range(i) = lidarM8.maxRange;
        hit      = 0;
                
        //Calc cos(heading) and sin(heading)
        double cosHead = cos(M_PI*(psi0+lidarM8.startDeg+i*lidarM8.resDeg)/180.0);
        double sinHead = sin(M_PI*(psi0+lidarM8.startDeg+i*lidarM8.resDeg)/180.0);
        
        //Calc end point of laser ray
        x2 = x0+lidarM8.maxRange * sinHead;
        y2 = y0+lidarM8.maxRange * cosHead;

        xr(i) = x2;
        yr(i) = y2;
            
        x1 = x2;
        y1 = y2;
        
        //Clip the line to live inside the main grid box
        xs = x0;
        ys = y0;
        
        // Set clipping up as loop or use bool check with ".select()" a few times?

        if((xs < map.E(0)) && (fabs(x1-x0) > eps)){
            xs = map.E(0);
            ys = y0 + (xs-x0)*((y1-y0)/(x1-x0));
        }
        if((ys < map.N(0)) && (fabs(y1-y0) > eps)){
            ys = map.N(0);
            xs = x0 + (ys-y0)*((x1-x0)/(y1-y0));
        }
        if((xs > map.E(N-1)) && (fabs(x1-x0) > eps)){
            xs = map.E(N-1);
            ys = y0 + (xs-x0)*((y1-y0)/(x1-x0));
        }
        if((ys > map.N(M-1)) && (fabs(y1-y0) > eps)){
            ys = map.N(M-1);
            xs = x0 + (ys-y0)*((x1-x0)/(y1-y0));
        }
         
        xe = x1;
        ye = y1;
        if((xe < map.E(0)) && (fabs(x1-x0) > eps)){
            xe = map.E(0);
            ye = y0 + (xe-x0)*((y1-y0)/(x1-x0));
        }
        if((ye < map.N(0)) && (fabs(y1-y0) > eps)){
            ye = map.N(0);
            xe = x0 + (ye-y0)*((x1-x0)/(y1-y0));
        }
        if((xe > map.E(N-1)) && (fabs(x1-x0) > eps)){
            xe = map.E(N-1);
            ye = y0 + (xe-x0)*((y1-y0)/(x1-x0));
        }
        if((ye > map.N(M-1)) && (fabs(y1-y0) > eps)){
            ye = map.N(M-1);
            xe = x0 + (ye-y0)*((x1-x0)/(y1-y0));
        }
        

        //Check that the new point is inside the box
        if (   (xs-map.E(0)>-eps) && (xs-map.E(N-1)<eps)
            && (ys-map.N(0)>-eps) && (ys-map.N(M-1)<eps)
            && (xe-map.E(0)>-eps) && (xe-map.E(N-1)<eps)
            && (ye-map.N(0)>-eps) && (ye-map.N(M-1)<eps)){
            
            //Determine x and y extremities for laser on this azimuth and polar angles
            //Calc min and max integers for x and y axes
            xsi = floor(max((double)0.0   , (double)((xs-map.E(0))/dx)));
            xsi = min((double)(N-1) , (double)xsi);
            xei = floor(max((double)0.0   , (double)((xe-map.E(0))/dx)));
            xei = min((double)(N-1) , (double)xei);
            ysi = floor(max((double)0.0   , (double)((ys-map.N(0))/dy)));
            ysi = min((double)(M-1) , (double)ysi);
            yei = floor(max((double)0.0   , (double)((ye-map.N(0))/dy)));
            yei = min((double)(M-1) , (double)yei);
            
            //Set the difference of start to end
            dxse = xe - xs;
            dyse = ye - ys;
            
            //Set the current indices to the starting ones
            xci  = xsi;
            yci  = ysi;
            xc   = xs;
            yc   = ys;

                       
            do{
                //Figure out which way to move next
                if(dxse > 0.0){
                    if(xci<N-1){
                        xdist = (map.E(xci+1) - xc)/dxse;
                    } else {
                        xdist = 1e20;
                    }
                } else if (dxse < 0.0){
                    if(xci>0){
                        xdist = (map.E(xci) - xc)/dxse;
                    } else {
                        xdist = 1e20;
                    }
                } else {
                    xdist = 1e20;
                }
                if(dyse > 0.0){
                    if(yci<M-1){
                        ydist = (map.N(yci+1) - yc)/dyse;
                    } else {
                        ydist = 1e20;
                    }
                } else if (dyse < 0.0){
                    if(yci > 0){
                        ydist = (map.N(yci) - yc)/dyse;
                    } else {
                        ydist = 1e20;
                    }
                } else {
                    ydist = 1e20;
                }
                
                if (fabs(xdist - ydist) < eps){
                    if((dxse > 0.0) && (dyse > 0.0)){
                        xci = min(N-1,xci+1);
                        yci = min(M-1,yci+1);
                        yc  = ys+(map.E(xci)-xs)*(ye-ys)/dxse;
                        xc  = map.E(xci);
                    } else if ((dxse < 0.0) && (dyse > 0.0)) {
                        xci = max(0,xci-1);
                        yci = min(M-1,yci+1);
                        yc  = ys+(map.E(xci)-xs)*(ye-ys)/dxse;
                        xc  = map.E(xci);
                    } else if ((dxse > 0.0) && (dyse < 0.0)){
                        xci = min(N-1,xci+1);
                        yci = max(0,yci-1);
                        yc  = ys+(map.E(xci)-xs)*(ye-ys)/dxse;
                        xc  = map.E(xci);
                    } else if ((dxse < 0.0) && (dyse < 0.0)) {
                        xci = max(0,xci-1);
                        yci = max(0,yci-1);
                        yc  = ys+(map.E(xci)-xs)*(ye-ys)/dxse;
                        xc  = map.E(xci);
                    } else {
                        break;
                    }
                } else if(xdist < ydist){
                    if(dxse > 0.0){
                        xci = min(N-1,xci+1);
                        yc  = ys+(map.E(xci)-xs)*(ye-ys)/dxse;
                        xc  = map.E(xci);
                    } else if (dxse < 0.0) {
                        xci = max(0,xci-1);
                        yc  = ys+(map.E(xci)-xs)*(ye-ys)/dxse;
                        xc  = map.E(xci);
                    } else {
                        break;
                    }
                } else {
                    if(dyse > 0.0){
                        yci = min(M-1,yci+1);
                        xc  = xs+(map.N(yci)-ys)*(xe-xs)/dyse;
                        yc  = map.N(yci);
                    } else if (dyse < 0.0) {
                        yci = max(0,yci-1);
                        xc  = xs+(map.N(yci)-ys)*(xe-xs)/dyse;
                        yc  = map.N(yci);
                    } else {
                        break;
                    }
                }
                
                if(map.z(yci + M*xci)>0){

                    //Figure out intersection points of ray line with four sides of box
                    ind[0]=linSegInt(map.E(xci),map.E(xci)+dx,map.N(yci)+dy,map.N(yci)+dy,xs,xe,ys,ye,&xint[0],&yint[0]);
                    ind[1]=linSegInt(map.E(xci),map.E(xci)+dx,map.N(yci),map.N(yci),xs,xe,ys,ye,&xint[1],&yint[1]);
                    ind[2]=linSegInt(map.E(xci),map.E(xci),map.N(yci),map.N(yci)+dy,xs,xe,ys,ye,&xint[2],&yint[2]);
                    ind[3]=linSegInt(map.E(xci)+dx,map.E(xci)+dx,map.N(yci),map.N(yci)+dy,xs,xe,ys,ye,&xint[3],&yint[3]);
                    
                    //Which line did we intersect with
                    int idxInt = 0;
                    
                    //Now figure out which side of the box we intersect with first (i.e. minimum range)
                    for(j=0;j<4;j++){
                        r[j] = sqrt((xint[j]-x0)*(xint[j]-x0) + (yint[j]-y0)*(yint[j]-y0));
                        
                        if ((ind[j] > 0) && (r[j] < range(i))){
                            idxInt   = j;
                            range(i) = r[j];
                            xr(i)    = xint[j];
                            yr(i)    = yint[j];
                        } 
                    }
                    hit = 1;
                    // assert(0);
                    // //Horizontal line intersection
                    // if ((idxInt==0) || (idxInt==1)){
                    //     C(i+0*L) = 0.0; //Deriv. w.r.t. x
                    //     if (sinHead != 0.0){
                    //         C(i+1*L) = -1.0/sinHead; //Deriv. w.r.t. y
                    //         C(i+2*L) = -(yr(i)-ys) * cosHead / (sinHead * sinHead); //Deriv. w.r.t. h
                    //     } else {
                    //         C(i+1*L) = 0.0; //Deriv. w.r.t. y
                    //         C(i+2*L) = 0.0; //Deriv. w.r.t. h
                    //     }
                        
                    // //Vertical line intersection
                    // } else { 
                    //     if (cosHead != 0.0){
                    //         C(i+0*L) = -1.0/cosHead; //Deriv. w.r.t. x
                    //         C(i+2*L) = (xr(i)-xs) * sinHead / (cosHead * cosHead); //Deriv. w.r.t. h
                    //     } else {
                    //         C(i+0*L) = 0.0; //Deriv. w.r.t. x
                    //         C(i+2*L) = 0.0; //Deriv. w.r.t. h
                    //     }
                    //     C(i+1*L) = 0.0; //Deriv. w.r.t. y
                    // }
                }
                
            } while (!hit && ((xci != xei) || (yci != yei)));
        }
    }
}