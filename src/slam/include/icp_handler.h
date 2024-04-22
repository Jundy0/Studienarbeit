
#include <iostream>
#include <vector>
#include <cmath>

#include "../lib/icp/icp.h"
#include "../lib/icp/Eigen/Dense"

struct TransformationComponents
{
    Eigen::Vector3d translation_vector;
    double rotation_angle_deg; // Rotation angle in degrees
};

class IcpHandler
{
public:
    IcpHandler();
    TransformationComponents extractTransformation(const Eigen::Matrix4d &);
    TransformationComponents call_icp(Eigen::MatrixXd, Eigen::MatrixXd);

};