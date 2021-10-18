#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <Eigen/Core>
#include "map.h"
#include "lidar.h"

void mapInit(const Eigen::VectorXd& origin, Map& map){
    // Map size variables, this will most likely be the biggest hinderance on speed
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

    // Create logOdds matrix, intially zeros i.e. likelihood of 1 i.e. we know nothing
    map.logOdds.setZero(map.numY,map.numX);

    // Intialise map as zeros to avoid odd initialisation with random memory values
    // You should update this with a map update before it is used for localisation
    map.z.setZero(map.numY,map.numX);
    
}

void mapUpdate(Eigen::VectorXd & y, Eigen::VectorXd & x, Map & map, Lidar & lidar){
    // Declare Variables
    double maxDist = hypot(map.N(map.N.rows()-1)-map.N(0),map.E(map.E.rows()-1)-map.E(0));
    int measRes = floor(1 + maxDist/(lidar.resDeg*M_PI/180));
    Eigen::VectorXd measInt(measRes);
    std::vector<int> isUseable;    
    Eigen::MatrixXd N_meas(lidar.theta.rows(),measRes), E_meas(lidar.theta.rows(),measRes);
    Eigen::MatrixXd N_idx(lidar.theta.cols(),measRes), E_idx(lidar.theta.cols(),measRes);
    

    // Calculate matrix of points at which to change logOdds
    measInt.setLinSpaced(measRes,0.0,maxDist);    

    N_meas << (x(0) + (lidar.theta.array().cos().matrix() * measInt.transpose()).array()).matrix();

    E_meas << (x(1) + (lidar.theta.array().sin().matrix() * measInt.transpose()).array()).matrix();

    N_idx = ((map.N.rows()-1)*(N_meas.array()-map.N(0))/(map.N(map.N.rows()-1)-map.N(0))).round().matrix();
    E_idx = ((map.E.rows()-1)*(E_meas.array()-map.E(0))/(map.E(map.E.rows()-1)-map.E(0))).round().matrix();
    
    // Check which measurments are useable 
    // Not used right now
    for(int i=0; i<y.rows();i++){
        if(y(i) < lidar.maxRange){
            isUseable.push_back(i);
        }     
    }
    
    if(isUseable.size()==0){
        return;
    }

    // Loop through useable measurments and measrument intervals
    for(int i=0;i<isUseable.size();i++){
        int index = isUseable[i];

        for(int k = 0; k<measInt.rows(); k++){            
            if(N_meas(index,k) < map.N(map.N.rows()-1) & N_meas(index,k) > map.N(0) & E_meas(index,k) < map.E(map.E.rows()-1) & E_meas(index,k) > map.E(0)){
                
                //Assume pass if 0 measurment returned
                //  ! Could result in close objects being considered not there however a max range measurment is far more likely
                if(y(index) == 0){
                    map.logOdds(N_idx(index,k),E_idx(index,k)) -= map.passChange/5.0;
                }
                // Decrease Logodds if ray passed through grid square
                else if(y(index) > measInt(k)){
                    map.logOdds(N_idx(index,k),E_idx(index,k)) -= map.passChange;
                }
                // Increase LogOdds if ray hit in gridsquare
                else if((y(index)+ map.hitDepth) > measInt(k)){
                    map.logOdds(N_idx(index,k),E_idx(index,k)) += map.hitChange;
                    
                }
            }
            // Do nothing if outside the map
        }
    }
    
    // Saturate logOdds to ensure their magnitude does not get too large
    map.logOdds = (map.logOdds.array() > 50).select(50, map.logOdds);
    map.logOdds = (map.logOdds.array() > -50).select(-50, map.logOdds);

    map.z = map.logOdds;

    // Set to 0 if unlikely and 1 if likely
    // This could be used for other boolian comparisons, it took me way too long to understand this
    map.z = (map.z.array() > 0).select(1, Eigen::MatrixXd::Zero(map.numY,map.numX)); // Wow I think I understand Eigen logial operations now


}