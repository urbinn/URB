
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
//#include "g2o/solvers/linear_solver_dense.h"
#include "g2o/solvers/dense/linear_solver_dense.h"

#include <opencv2/core/core.hpp>
#include <Eigen/StdVector>

#include "dep_ba.h"

using namespace std;


Eigen::Matrix<double,3,1> toVector3d(const MapPointMatrix point){
    Eigen::Matrix<double,3,1> v;
    v << point(0), point(1), point(2);
    
    return v;
}

g2o::SE3Quat toSE3QuatFromMatrix(const Eigen::Matrix<double,4,4> &eigenMat){
    Eigen::Matrix<double,3,3> R;
    R << eigenMat(0,0), eigenMat(0,1), eigenMat(0,2),
    eigenMat(1,0), eigenMat(1,1), eigenMat(1,2),
    eigenMat(2,0), eigenMat(2,1), eigenMat(2,2);
    
    Eigen::Matrix<double,3,1> t(eigenMat(0,3), eigenMat(1,3), eigenMat(2,3));
    
    return g2o::SE3Quat(R,t);
}

Eigen::MatrixXd toEigenBundel(const g2o::SE3Quat &SE3){
    Eigen::Matrix<double, 4, 4> eigMat = SE3.to_homogeneous_matrix();
    return eigMat;
}


Eigen::MatrixXd toEigenVector(const Eigen::Matrix<double,3,1> &m)
{
    Eigen::Matrix<double, 1, 3> eigMat;
    eigMat << m(0,0), m(1,0), m(2,0);
    return eigMat;
}


Eigen::MatrixXd keyFrameRowToMatrix(const Eigen::MatrixXd row)
{
    Eigen::MatrixXd frameMtrix(4, 4);
    
    frameMtrix << row(2), row(3), row(4), row(5),
    row(6), row(7), row(8), row(9),
    row(10), row(11), row(12), row(13),
    row(14), row(15), row(16), row(17);
    
    return frameMtrix;
}

Eigen::MatrixXd mappointRowToMatrix(const Eigen::MatrixXd row)
{
    Eigen::MatrixXd pointMatrix(1, 3);
    pointMatrix << row(1), row(2), row(3) ;
    return pointMatrix;
}


std::vector<KeyFrame> getObservations(MapPoint point ,  std::vector<KeyFrame> keyframes,  Eigen::MatrixXd worldMapPoints,Eigen::MatrixXd pointsRelation) {
    std::vector<KeyFrame> returnKeyframes;
    for(int r = 0; r <  pointsRelation.rows(); r++) {
        Eigen::MatrixXd currentRelation(1, pointsRelation.cols());
        currentRelation << pointsRelation.row(r);
        if (point.first.second == currentRelation(0)) {
            //found relation
            for (KeyFrame frame : keyframes) {
                if (currentRelation(1) == frame.first.second) {
                    returnKeyframes.push_back(frame);
                    break;
                }
            }
        }
    }
    
    return returnKeyframes;
}

std::vector<std::pair<KeyFrame, int>> getObservationsWithRelation(MapPoint point ,  std::vector<KeyFrame> keyframes,  Eigen::MatrixXd, Eigen::MatrixXd pointsRelation ) {
    std::vector<std::pair<KeyFrame, int>> returnKeyframes;
    for(int r = 0; r <  pointsRelation.rows(); r++) {
        Eigen::MatrixXd currentRelation(1, pointsRelation.cols());
        currentRelation << pointsRelation.row(r);
        //cout << currentRelation << endl;
        //cout << point.first.second << endl;
        if (point.first.second == currentRelation(0)) {
            //found relation
            for (KeyFrame frame : keyframes) {
                if (currentRelation(1) == frame.first.second) {
                    
                    returnKeyframes.push_back(std::make_pair(frame,r) );
                    break;
                }
            }
        }
    }
    
    return returnKeyframes;
}