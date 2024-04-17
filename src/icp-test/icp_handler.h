#include <iostream>
#include <vector>
#include <cmath>

#include "../icp/Eigen/Dense"
#include "../icp/icp.h"
#include "./load_vectors.cpp"

struct TransformationComponents {
    Eigen::Vector3d translation_vector;
    double rotation_angle_deg; // Rotation angle in degrees
};

class  IcpHandler{
    public:
        IcpHandler();
        TransformationComponents extractTransformation(const Eigen::Matrix4d&);
        TransformationComponents call_icp(Eigen::MatrixXd, Eigen::MatrixXd);
        Eigen::MatrixXd polar_to_cartesian_from_matrix(Eigen::MatrixX2d);
        void dummy_icp();
    private:
        Eigen::MatrixXd polar_to_cartesian(const std::vector<double>&, const std::vector<double>&);
        Eigen::MatrixXd translate_rotate_matrix(const Eigen::MatrixXd& , const std::vector<double>&, double);
        static Eigen::MatrixX2d load_points_from_file();
        Eigen::MatrixXd get_matrix_from_points(Eigen::MatrixX2d);
        Eigen::MatrixXd apply_transformation(Eigen::MatrixXd);

};