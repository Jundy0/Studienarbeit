#define _USE_MATH_DEFINES

#include <math.h>

#include "../include/icp_handler.h"


IcpHandler::IcpHandler()
{
}

TransformationComponents IcpHandler::extractTransformation(const Eigen::Matrix4d &transformationMatrix) {
    TransformationComponents components;

    // Extract translation vector with only x and y as int
    components.translation_vector << static_cast<int>(transformationMatrix(0, 3)),
                                     static_cast<int>(transformationMatrix(1, 3));

    // Calculate rotation angle from the rotation matrix part of T
    // Assuming the rotation is around the Z-axis
 double theta_rad = std::atan2(transformationMatrix(1, 0), transformationMatrix(0, 0)); // atan2(sin, cos)
    components.rotation_angle = theta_rad; // Store in radians
    if (components.rotation_angle < 0)
        components.rotation_angle+= 2 * M_PI; // Convert negative angles to positive

    // Convert the rotation angle to degrees for display purposes
    double rotation_angle_deg = components.rotation_angle * (180.0 / M_PI);
    std::cout << "initial rotation angle " << rotation_angle_deg << " degrees" << std::endl;
    return components;
}

Eigen::MatrixXd IcpHandler::polar_to_cartesian_from_matrix(Eigen::MatrixX2d points)
{
    Eigen::MatrixXd cartesian_coords(3, points.rows());
    for (int i = 0; i < points.rows(); i++)
    {
        double x = points.row(i)[0] * cos(points.row(i)[1] * M_PI / 180);
        double y = points.row(i)[0] * sin(points.row(i)[1] * M_PI / 180);
        cartesian_coords.col(i) << x, y, 0.0; // Setting x, y, z = 0
    }
    std::cout << cartesian_coords.rows() << "\n"
              << cartesian_coords.cols() << std::endl;
    return cartesian_coords.transpose();
}

TransformationComponents IcpHandler::execute_icp(Eigen::MatrixXd initial_matrix, Eigen::MatrixXd transformed_matrix)
{
    ICP_OUT result = icp(initial_matrix, transformed_matrix, 50, 0.000000001);

    std::cout << "Transformation Matrix" << std::endl;
    std::cout << result.trans << std::endl;

    std::cout << "Iter:" << std::endl;
    std::cout << result.iter << std::endl;

    std::cout << "extracted:" << std::endl;
    TransformationComponents trans_comps = extractTransformation(result.trans);
    std::cout << trans_comps.translation_vector << std::endl;
    std::cout << trans_comps.rotation_angle << std::endl;
    return trans_comps;
}

TransformationComponents IcpHandler::call_icp(Eigen::Matrix2d scan_one, Eigen::Matrix2d scan_two)
{
    Eigen::MatrixXd initial_mattrix = get_matrix_from_points(scan_one);
    Eigen::MatrixXd transformed_matirx = get_matrix_from_points(scan_two);
    return execute_icp(initial_mattrix,transformed_matirx);
}

Eigen::MatrixXd IcpHandler::translate_rotate_matrix(const Eigen::MatrixXd &cartesian_coords, const std::vector<double> &translation_vector, double rotation_angle)
{
    // Convert rotation angle to radians
    double theta = rotation_angle * M_PI / 180.0;
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);

    // Define rotation matrix
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << cos_theta, -sin_theta, 0,
        sin_theta, cos_theta, 0,
        0, 0, 1;

    // Create a copy of the input matrix for manipulation
    Eigen::MatrixXd rotated_translated_matrix = cartesian_coords;

    // Apply rotation to each point
    rotated_translated_matrix = rotated_translated_matrix * rotation_matrix;

    // Apply translation matrix to the points
    for (int i = 0; i < rotated_translated_matrix.rows(); i++)
    {
        for (int j = 0; j < 3; j++)
        {
            rotated_translated_matrix.row(i)[j] += translation_vector[j];
        }
    }

    return rotated_translated_matrix;
}

Eigen::MatrixXd IcpHandler::apply_transformation(Eigen::MatrixXd start_matrix)
{
    std::vector<double> translation_vector = {4.0, 4.0, 0.0}; // Example translation
    double rotation_angle = 360.0;                             // Example rotation angle
    return translate_rotate_matrix(start_matrix, translation_vector, rotation_angle);
}

Eigen::MatrixXd IcpHandler::get_matrix_from_points(Eigen::MatrixX2d points = load_points_from_file())
{
    return polar_to_cartesian_from_matrix(points);
}

void IcpHandler::dummy_icp()
{
    // Convert polar coordinates to Cartesian coordinates
    Eigen::MatrixXd initial_coordinates = get_matrix_from_points();
    // Perform translation and rotation
    Eigen::MatrixXd rotated_translated_matrix = apply_transformation(initial_coordinates);

    TransformationComponents trans_comps = execute_icp(initial_coordinates, rotated_translated_matrix);

    std::cout << trans_comps.translation_vector << std::endl;
    std::cout << trans_comps.rotation_angle << std::endl;
};

Eigen::MatrixX2d load_points_from_file()
{
    return getFileData();
}