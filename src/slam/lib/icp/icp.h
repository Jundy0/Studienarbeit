#ifndef __ICP_H__
#define __ICP_H__

#include "Eigen/Eigen"
#include <vector>

#define N_pt 30          // # of points in the datasets
#define N_tests 100      // # of test iterations
#define noise_sigma 0.01 // standard deviation error to be added
#define translation 0.1  // max translation of the test set
#define rotation 0.1     // max rotation (radians) of the test set

typedef struct
{
    Eigen::Matrix3d trans;
    std::vector<float> distances;
    int iter;
} ICP_OUT;

typedef struct
{
    std::vector<float> distances;
    std::vector<int> indices;
} NEIGHBOR;

Eigen::Matrix3d best_fit_transform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B);

ICP_OUT icp(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, int max_iterations = 20, double tolerance = 0.001);

// throughout method
NEIGHBOR nearest_neighbot(const Eigen::MatrixXd &src, const Eigen::MatrixXd &dst);
float dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb);

#endif // __ICP_H__
