#ifndef SPACIALDUAL_H
#define SPACIALDUAL_H

#include <Eigen/Core>

void gpsInit(void);
void rotateLatLong(Eigen::VectorXd& geoCoordDeg, Eigen::MatrixXd& R);
void NEDtoGeo(const Eigen::MatrixXd& rBNn, Eigen::MatrixXd& rBOg);
void quaterniontoRPY(const Eigen::MatrixXd& quaternion, Eigen::MatrixXd& RPY);
void RPYtoQuaternion(const Eigen::MatrixXd& RPY, Eigen::MatrixXd& quaternion);


#endif