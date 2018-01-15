#ifndef URB_OUT_DETECTION
#define URB_OUT_DETECTION

#include <string>
#include <opencv2/core/core.hpp>
#include <Eigen/LU>
#include <Eigen/StdVector>

Eigen::MatrixXd outliersForLocalBundleAdjustment(Eigen::Ref<Eigen::MatrixXd> keyframes, Eigen::Ref<Eigen::MatrixXd> fixedKeyframes, Eigen::Ref<Eigen::MatrixXd> worldMapPoints, Eigen::Ref<Eigen::MatrixXd> pointsRelation ) ;

#endif
