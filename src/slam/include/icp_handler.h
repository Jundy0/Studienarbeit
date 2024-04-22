
#include <iostream>
#include <vector>
#include <cmath>

#include "../lib/icp/icp.h"
#include "../lib/icp/Eigen/Dense"
#include "../include/load_vectors.h"

struct TransformationComponents
{
    Eigen::RowVector2i translation_vector;
    double rotation_angle; // Rotation angle in degrees
};

class IcpHandler
{
public:
    IcpHandler();
    TransformationComponents extractTransformation(const Eigen::Matrix4d &);
    TransformationComponents call_icp(Eigen::Matrix2d, Eigen::Matrix2d);
    Eigen::MatrixXd polar_to_cartesian_from_matrix(Eigen::MatrixX2d);
    void dummy_icp();

private:
    TransformationComponents execute_icp(Eigen::MatrixXd , Eigen::MatrixXd );
    Eigen::MatrixXd polar_to_cartesian(const std::vector<double> &, const std::vector<double> &);
    Eigen::MatrixXd translate_rotate_matrix(const Eigen::MatrixXd &, const std::vector<double> &, double);
    Eigen::MatrixXd get_matrix_from_points(Eigen::MatrixX2d);
    Eigen::MatrixXd apply_transformation(Eigen::MatrixXd);
};

Eigen::MatrixX2d load_points_from_file();