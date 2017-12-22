
#include <opencv2/core/core.hpp>
#include <Eigen/StdVector>
#include <string>
#include <opencv2/core/core.hpp>
#include <Eigen/LU>
#include <Eigen/StdVector>

// column size for keyframe
typedef Eigen::Matrix<double, 4, 4> KeyFrameMatrix;
typedef Eigen::Matrix<double, 1, 3> MapPointMatrix;
typedef Eigen::Matrix<double, 1, 4> KeyFrameMapPointMatrix;

//index - key
typedef std::pair<  std::pair<int, int>, KeyFrameMatrix> KeyFrame;
typedef std::pair< std::pair<int, int>, MapPointMatrix> MapPoint;

const float CameraFx = 718.856;
const float CameraFy = 718.856;
const float CameraCx = 607.1928;
const float CameraCy = 185.2157;

Eigen::Matrix<double,3,1> toVector3d(const MapPointMatrix point);
g2o::SE3Quat toSE3QuatFromMatrix(const Eigen::Matrix<double,4,4> &eigenMat);
Eigen::MatrixXd toEigenBundel(const g2o::SE3Quat &SE3);


Eigen::MatrixXd toEigenVector(const Eigen::Matrix<double,3,1> &m);

Eigen::MatrixXd keyFrameRowToMatrix(const Eigen::MatrixXd row);

Eigen::MatrixXd mappointRowToMatrix(const Eigen::MatrixXd row);


std::vector<KeyFrame> getObservations(MapPoint point ,  std::vector<KeyFrame> keyframes,  Eigen::MatrixXd worldMapPoints,Eigen::MatrixXd pointsRelation) ;

std::vector<std::pair<KeyFrame, int>> getObservationsWithRelation(MapPoint point ,  std::vector<KeyFrame> keyframes,  Eigen::MatrixXd, Eigen::MatrixXd pointsRelation ) ;