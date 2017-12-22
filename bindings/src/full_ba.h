#ifndef URB_FULL_BA
#define URB_FULL_BA

#include <string>
#include <opencv2/core/core.hpp>
#include <Eigen/LU>
#include <Eigen/StdVector>

int fullBundleAdjustment(Eigen::Ref<Eigen::MatrixXd> keyframes, Eigen::Ref<Eigen::MatrixXd> mapPoints, Eigen::Ref<Eigen::MatrixXd> pointsRelation ) ;

#endif
