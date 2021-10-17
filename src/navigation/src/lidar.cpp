#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <iostream>
#include <Eigen/Core>

#include "lidar.hpp"

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

//This function returns the intersection of two line segments in 2D
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

//This function updates the maps
void findRange(Map & map, Scanner & scanner, Pose & pose, Eigen::VectorXd & range, Eigen::VectorXd & xr, Eigen::VectorXd & yr, Eigen::VectorXd & C){

    int L = scanner.numScans;
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
    double dx    = map.dx;
    double dy    = map.dy;
    double mapx0 = map.x0;
    double mapy0 = map.y0;
    
    x0 = pose.x - scanner.x0;
    y0 = pose.y - scanner.y0;
    psi0 = pose.psi - scanner.psi0;
    
    int xsi, xei, ysi, yei, xci, yci;
    double dxse, dyse, xc, yc, xdist, ydist;
    
    //Loop over the number of scan lines
    for(int i=0;i<L;i++){
        
        //Max range to begin with and no hit
        range(i) = scanner.maxRange;
        hit      = 0;
                
        //Calc cos(heading) and sin(heading)
        double cosHead = cos(M_PI*(psi0+scanner.startDeg+i*scanner.resDeg)/180.0);
        double sinHead = sin(M_PI*(psi0+scanner.startDeg+i*scanner.resDeg)/180.0);
        
        //Calc end point of laser ray
        x2 = x0+scanner.maxRange * sinHead;
        y2 = y0+scanner.maxRange * cosHead;

        xr(i) = x2;
        yr(i) = y2;
            
        x1 = x2;
        y1 = y2;
        
        //Clip the line to live inside the main grid box
        xs = x0;
        ys = y0;
        
        // Set clipping up as loop or use bool check with ".select()" a few times?

        if((xs < map.x(0)) && (fabs(x1-x0) > eps)){
            xs = map.x(0);
            ys = y0 + (xs-x0)*((y1-y0)/(x1-x0));
        }
        if((ys < map.y(0)) && (fabs(y1-y0) > eps)){
            ys = map.y(0);
            xs = x0 + (ys-y0)*((x1-x0)/(y1-y0));
        }
        if((xs > map.x(N-1)) && (fabs(x1-x0) > eps)){
            xs = map.x(N-1);
            ys = y0 + (xs-x0)*((y1-y0)/(x1-x0));
        }
        if((ys > map.y(M-1)) && (fabs(y1-y0) > eps)){
            ys = map.y(M-1);
            xs = x0 + (ys-y0)*((x1-x0)/(y1-y0));
        }
         
        xe = x1;
        ye = y1;
        if((xe < map.x(0)) && (fabs(x1-x0) > eps)){
            xe = map.x(0);
            ye = y0 + (xe-x0)*((y1-y0)/(x1-x0));
        }
        if((ye < map.y(0)) && (fabs(y1-y0) > eps)){
            ye = map.y(0);
            xe = x0 + (ye-y0)*((x1-x0)/(y1-y0));
        }
        if((xe > map.x(N-1)) && (fabs(x1-x0) > eps)){
            xe = map.x(N-1);
            ye = y0 + (xe-x0)*((y1-y0)/(x1-x0));
        }
        if((ye > map.y(M-1)) && (fabs(y1-y0) > eps)){
            ye = map.y(M-1);
            xe = x0 + (ye-y0)*((x1-x0)/(y1-y0));
        }
        

        //Check that the new point is inside the box
        if (   (xs-map.x(0)>-eps) && (xs-map.x(N-1)<eps)
            && (ys-map.y(0)>-eps) && (ys-map.y(M-1)<eps)
            && (xe-map.x(0)>-eps) && (xe-map.x(N-1)<eps)
            && (ye-map.y(0)>-eps) && (ye-map.y(M-1)<eps)){
            
            //Determine x and y extremities for laser on this azimuth and polar angles
            //Calc min and max integers for x and y axes
            xsi = floor(max((double)0.0   , (double)((xs-map.x(0))/dx)));
            xsi = min((double)(N-1) , (double)xsi);
            xei = floor(max((double)0.0   , (double)((xe-map.x(0))/dx)));
            xei = min((double)(N-1) , (double)xei);
            ysi = floor(max((double)0.0   , (double)((ys-map.y(0))/dy)));
            ysi = min((double)(M-1) , (double)ysi);
            yei = floor(max((double)0.0   , (double)((ye-map.y(0))/dy)));
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
                        xdist = (map.x(xci+1) - xc)/dxse;
                    } else {
                        xdist = 1e20;
                    }
                } else if (dxse < 0.0){
                    if(xci>0){
                        xdist = (map.x(xci) - xc)/dxse;
                    } else {
                        xdist = 1e20;
                    }
                } else {
                    xdist = 1e20;
                }
                if(dyse > 0.0){
                    if(yci<M-1){
                        ydist = (map.y(yci+1) - yc)/dyse;
                    } else {
                        ydist = 1e20;
                    }
                } else if (dyse < 0.0){
                    if(yci > 0){
                        ydist = (map.y(yci) - yc)/dyse;
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
                        yc  = ys+(map.x(xci)-xs)*(ye-ys)/dxse;
                        xc  = map.x(xci);
                    } else if ((dxse < 0.0) && (dyse > 0.0)) {
                        xci = max(0,xci-1);
                        yci = min(M-1,yci+1);
                        yc  = ys+(map.x(xci)-xs)*(ye-ys)/dxse;
                        xc  = map.x(xci);
                    } else if ((dxse > 0.0) && (dyse < 0.0)){
                        xci = min(N-1,xci+1);
                        yci = max(0,yci-1);
                        yc  = ys+(map.x(xci)-xs)*(ye-ys)/dxse;
                        xc  = map.x(xci);
                    } else if ((dxse < 0.0) && (dyse < 0.0)) {
                        xci = max(0,xci-1);
                        yci = max(0,yci-1);
                        yc  = ys+(map.x(xci)-xs)*(ye-ys)/dxse;
                        xc  = map.x(xci);
                    } else {
                        break;
                    }
                } else if(xdist < ydist){
                    if(dxse > 0.0){
                        xci = min(N-1,xci+1);
                        yc  = ys+(map.x(xci)-xs)*(ye-ys)/dxse;
                        xc  = map.x(xci);
                    } else if (dxse < 0.0) {
                        xci = max(0,xci-1);
                        yc  = ys+(map.x(xci)-xs)*(ye-ys)/dxse;
                        xc  = map.x(xci);
                    } else {
                        break;
                    }
                } else {
                    if(dyse > 0.0){
                        yci = min(M-1,yci+1);
                        xc  = xs+(map.y(yci)-ys)*(xe-xs)/dyse;
                        yc  = map.y(yci);
                    } else if (dyse < 0.0) {
                        yci = max(0,yci-1);
                        xc  = xs+(map.y(yci)-ys)*(xe-xs)/dyse;
                        yc  = map.y(yci);
                    } else {
                        break;
                    }
                }
                
                if(map.z(yci + M*xci)>0){

                    //Figure out intersection points of ray line with four sides of box
                    ind[0]=linSegInt(map.x(xci),map.x(xci)+dx,map.y(yci)+dy,map.y(yci)+dy,xs,xe,ys,ye,&xint[0],&yint[0]);
                    ind[1]=linSegInt(map.x(xci),map.x(xci)+dx,map.y(yci),map.y(yci),xs,xe,ys,ye,&xint[1],&yint[1]);
                    ind[2]=linSegInt(map.x(xci),map.x(xci),map.y(yci),map.y(yci)+dy,xs,xe,ys,ye,&xint[2],&yint[2]);
                    ind[3]=linSegInt(map.x(xci)+dx,map.x(xci)+dx,map.y(yci),map.y(yci)+dy,xs,xe,ys,ye,&xint[3],&yint[3]);
                    
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