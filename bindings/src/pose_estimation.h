#ifndef URB_POSE_ESTIMATION
#define URB_POSE_ESTIMATION

#include <Eigen/StdVector>

int poseOptimization(Eigen::Ref<Eigen::MatrixXd> coords, Eigen::Ref<Eigen::MatrixXd> pose);

#endif
