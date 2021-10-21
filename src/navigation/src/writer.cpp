#include <fstream> 
#include <Eigen/Core> 

#include "ros/ros.h"
#include "navigation/state.h"
#include "gazebo_msgs/ModelStates.h"
#include "spacialDual.h"

static Eigen::VectorXd trueState(6), estimatedState(6);

void likelyCallback(const navigation::state::ConstPtr& stateMsg){
    estimatedState << stateMsg->N,stateMsg->E,stateMsg->psi,stateMsg->u,stateMsg->v,stateMsg->r;
}
void trueCallback(const gazebo_msgs::ModelStates::ConstPtr& nedMsg){
    Eigen::MatrixXd q(4,1), RPY;
    int idx = 18;
    q << nedMsg->pose[idx].orientation.w,nedMsg->pose[idx].orientation.x,nedMsg->pose[idx].orientation.y,nedMsg->pose[idx].orientation.z;
    quaterniontoRPY(q, RPY);
    trueState << nedMsg->pose[idx].position.x, nedMsg->pose[idx].position.y,RPY(2),nedMsg->twist[idx].linear.x, nedMsg->twist[idx].linear.y, nedMsg->twist[idx].angular.z;
}

int main(int argc, char **argv){
    //Initialise ROS
    ros::init(argc, argv, "dataWriter");

    // Set Up subscribers
    ros::NodeHandle handler;
    ros::Subscriber trueSub = handler.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, trueCallback);
    ros::Subscriber estimatedSub = handler.subscribe<navigation::state>("/NUMarine/state/likely", 1, likelyCallback);
    
    // Set up files to write to
    std::ofstream trueData("src/navigation/data/trueState.csv");
    std::ofstream estimated("src/navigation/data/estimatedState.csv");

    // Write collumn headings
    trueData << "N,E,psi,u,v,r\n";
    estimated << "N,E,psi,u,v,r\n";

    int count = 0;
    // Check at rate of 8Hz (approx. twice as fast as localisation is calculated)
    ros::Rate rate(8);
    while(ros::ok()){
        
        ros::spinOnce();

        // Write states to rows of csv
        trueData << trueState(0) << "," << trueState(1) << "," << trueState(2) << "," << trueState(3) << "," << trueState(4) << "," << trueState(5) << "," << "\n";
        estimated << estimatedState(0) << "," << estimatedState(1) << "," << estimatedState(2) << "," << estimatedState(3) << "," << estimatedState(4) << "," << estimatedState(5) << "," << "\n";
        
        // Visual inidiacation in terminal that files are being written
        if(count==16){
            std::cout << "\33[2K" << "Writing" << "\r"; //Reset dots
            count=0;
        }
        std::cout << "Writing" << std::string(count/2+1, '.') << "\r" ;
        std::cout.flush();
        count++;        

        rate.sleep();
        // ros::shutdown();
    }

    // Close the csv files to save them propperly after node shutdown
    trueData.close();
    estimated.close();


}