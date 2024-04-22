
#include <iostream>
#include <vector>
#include <cmath>

#include "../lib/icp/icp.h"
#include "../lib/eigen/Eigen/Dense"

struct TransformationComponents
{
    Eigen::RowVector2i translation_vector;
    double rotation_angle; // Rotation angle in degrees
};

class IcpHandler
{
public:
    IcpHandler();
    TransformationComponents call_icp(Eigen::MatrixX2d, Eigen::MatrixX2d);

private:
    TransformationComponents execute_icp(Eigen::MatrixXd , Eigen::MatrixXd );
    Eigen::MatrixXd get_matrix_from_points(Eigen::MatrixX2d);
    Eigen::MatrixXd polar_to_cartesian_from_matrix(Eigen::MatrixX2d);
    TransformationComponents extractTransformation(const Eigen::Matrix4d &);
};