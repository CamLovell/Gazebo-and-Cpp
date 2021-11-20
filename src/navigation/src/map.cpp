#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <iostream>
#include <iomanip>
#include <Eigen/Core>

#include "lidar.h"
#include "map.h"


void mapInit(const Eigen::VectorXd& origin, Map& map){
    // Map size variables, this will most likely be the biggest hindrance on speed
    double front = 150, back = 150, left = 150, right = 150;
    map.dE = 0.05;
    map.dN = map.dE;
    map.N0 = origin(0)-right;
    map.E0 = origin(1)-back;

    // Create vectors for N and E gridlines
    map.N.setLinSpaced((back+front)/map.dN,map.N0, map.N0+right+left);
    map.E.setLinSpaced((back+front)/map.dE,map.E0, map.E0+back+front);

    map.numY = map.N.size();
    map.numX = map.E.size();

    // Create logOdds matrix, initially zeros i.e. likelihood of 1 i.e. we know nothing
    map.logOdds.setZero(map.numY,map.numX);

    // Initialise map as zeros to avoid odd initialisation with random memory values
    // You should update this with a map update before it is used for localisation
    map.z.setZero(map.numY,map.numX);
    
}

//--------------------------------------------------------------------------------------------------------------------------
// Map update function
//--------------------------------------------------------------------------------------------------------------------------

void mapUpdate(const Eigen::VectorXd & y,const Eigen::VectorXd & x, const Lidar & lidar, Map & map){
    // Declare Variables
    double maxDist = hypot(map.N(map.N.rows()-1)-map.N(0),map.E(map.E.rows()-1)-map.E(0));
    int measRes = floor(1 + maxDist/(lidar.scanRes));
    Eigen::VectorXd measInt(measRes);
    std::vector<int> isUseable;    
    Eigen::MatrixXd N_meas(lidar.theta.rows(),measRes), E_meas(lidar.theta.rows(),measRes);
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> idxN(lidar.theta.cols(),measRes), idxE(lidar.theta.cols(),measRes);    


    // Calculate matrices of points (N and E) at which to change logOdds
    measInt.setLinSpaced(measRes,0.0,maxDist);    

    E_meas << (x(1) + ((lidar.theta.array()*M_PI/180).sin().matrix() * measInt.transpose()).array()).matrix();
    N_meas << (x(0) + ((lidar.theta.array()*M_PI/180).cos().matrix() * measInt.transpose()).array()).matrix();

    // Find index locations of these points in map matrix
    idxN = ((map.N.rows()-1)*(N_meas.array()-map.N(0))/(map.N(map.N.rows()-1)-map.N(0))).round().matrix();
    idxE = ((map.E.rows()-1)*(E_meas.array()-map.E(0))/(map.E(map.E.rows()-1)-map.E(0))).round().matrix();
    

    // Check which measurements are useable 
    for(int i=0; i<y.rows();i++){
        if(y(i) < lidar.maxRange){
            isUseable.push_back(i);
        }     
    }
    // If no useable measurements return function with no changes to logOdds
    if(isUseable.size()==0){
        return;
    }
    int count = 0, count1 = 0, count2 = 0;

    Eigen::MatrixXd checkN = (N_meas.array() < map.N(map.N.rows()-1) && N_meas.array() > map.N(0) && E_meas.array() < map.E(map.E.rows()-1) && E_meas.array() > map.E(0)).select(1, Eigen::MatrixXd::Zero(N_meas.rows(),N_meas.cols()));

    // Loop through useable measurements and measurement intervals
    for(int i=0;i<isUseable.size();i++){
        int index = isUseable[i];
        count1++;                        

        for(int k = 0; k<measInt.rows(); k++){      
                  
            if(checkN(index,k)){
                count++;                        
                int idxNk = idxN(index,k);
                int idxEk = idxE(index,k);
                //Assume pass if 0 measurement returned
                //  ! Could result in close objects being considered not there however a max range measurement is far more likely
                if(y(index) == 0){
                    map.logOdds(idxNk,idxEk) -= map.passChange/5.0;
                }
                // Decrease Logodds if ray passed through grid square
                else if(y(index) > measInt(k)){
                    map.logOdds(idxNk,idxEk) -= map.passChange;
                }
                // Increase LogOdds if ray hit in gridsquare
                else if((y(index)+ map.hitDepth) > measInt(k)){                  
                    map.logOdds(idxNk,idxEk) += map.hitChange;
                }
                // Break loop once past hit
                else{
                    break;
                }
    
            }
            // Break loop if not in map
            else{
                break;
            }
            
        }
    }
    
    count2 = count/count1;
    // Saturate logOdds to ensure their magnitude does not get too large
    map.logOdds = (map.logOdds.array() > 50).select(50, map.logOdds);
    map.logOdds = (map.logOdds.array() < -50).select(-50, map.logOdds);

    // Set to 0 if unlikely and 1 if likely
    // This could be used for other boolian comparisons, it took me way too long to understand this
    map.z = (map.logOdds.array() > 0).select(1, Eigen::MatrixXd::Zero(map.numY,map.numX)); // Wow I think I understand Eigen logical operations now


}