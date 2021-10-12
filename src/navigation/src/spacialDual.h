#ifndef SPACIALDUAL_H
#define SPACIALDUAL_H

#include <Eigen/Core>

void gpsInit(void);
void gpsLogLiklihood(const Eigen::VectorXd& y, const Eigen::MatrixXd& x, const int& M, Eigen::VectorXd& lw);
void imuLogLiklihood(const Eigen::MatrixXd& y, const Eigen::MatrixXd& x, const int& M, Eigen::VectorXd& lw);

void rotateLatLong(const Eigen::VectorXd& geoCoordDeg, Eigen::MatrixXd& R);
void NEDtoGeo(const Eigen::MatrixXd& rBNn, Eigen::MatrixXd& rBOg);
void geotoNED(const Eigen::MatrixXd& rBOg, Eigen::MatrixXd& rBNn);

void quaterniontoRPY(const Eigen::MatrixXd& quaternion, Eigen::MatrixXd& RPY);
void RPYtoQuaternion(const Eigen::MatrixXd& RPY, Eigen::MatrixXd& quaternion);


#endif